//// filepath: /home/sentry_nav2/src/rm_interfaces/src/serial.cpp
#include "rm_serial/serial_driver.hpp"
#include "rm_serial/packet_format.hpp"
#include "rm_serial/serial_communicator.hpp"
#include "rm_serial/decision_maker.hpp"
#include "rm_serial/ros_interface.hpp" // 新增的头文件
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // For toMsg
#include <chrono>
#include <cstring> // For memcpy
#include <sstream>
#include <iomanip>


SerialDriver::SerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options), 
  move_mode_(4),
  nav_data_updated_(true),
  gimbal_data_updated_(false),
  is_navigating_to_center_(false),
  is_navigating_to_start_(false),
  is_navigating_to_healing_(false), // 初始化新变量
  last_aim_color_(255) // 初始化为不可能的值
{
    init_params(); // All parameters are initialized here
    
    communicator_ = std::make_unique<rm_serial::SerialCommunicator>(
        this, 
        serial_port_, 
        baud_rate_,
        reconnect_delay_, 
        max_reconnect_attempts_
    );

    decision_maker_ = std::make_unique<rm_serial::DecisionMaker>(
        this,
        center_point_, // initialized from params
        start_point_,  // initialized from params
        helper_point_, // 传入辅助点
        low_blood_threshold_, // Pass configured threshold
        low_ammo_threshold_   // Pass configured threshold
    );

    ros_interface_ = std::make_unique<rm_serial::RosInterface>(
        this,
        base_frame_id_, 
        gimbal_frame_id_,
        default_bullet_speed_ // Pass the configured default bullet speed
    );

    // 初始化模式服务客户端
    init_mode_service_clients();

    init_state(); 
    
    RCLCPP_INFO(this->get_logger(), "RxPacket size: %zu, TxPacket size: %zu",
               PacketFormat::RX_PACKET_SIZE, PacketFormat::TX_PACKET_SIZE);
    
    if (!communicator_->open_port()) {
        RCLCPP_ERROR(this->get_logger(), "串口初始化失败 via communicator");
    }

    auto qos = rclcpp::QoS(10).best_effort();

    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/armor_solver/cmd_gimbal", qos,
        std::bind(&SerialDriver::gimbal_cmd_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/red_standard_robot1/cmd_vel", qos,
        std::bind(&SerialDriver::cmd_vel_callback, this, std::placeholders::_1));
    nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&SerialDriver::timer_callback, this));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose");

}

// 新增：初始化模式服务客户端
void SerialDriver::init_mode_service_clients()
{
    // 创建服务客户端
    auto client1 = this->create_client<rm_interfaces::srv::SetMode>(
        "armor_detector/set_mode", rmw_qos_profile_services_default);
    auto client2 = this->create_client<rm_interfaces::srv::SetMode>(
        "armor_solver/set_mode", rmw_qos_profile_services_default);
    auto client3 = this->create_client<rm_interfaces::srv::SetMode>(
        "rune_detector/set_mode", rmw_qos_profile_services_default);
    auto client4 = this->create_client<rm_interfaces::srv::SetMode>(
        "rune_solver/set_mode", rmw_qos_profile_services_default);
    
    mode_clients_ = {client1, client2, client3, client4};
    
    RCLCPP_INFO(this->get_logger(), "Mode service clients initialized (%zu clients)", mode_clients_.size());
}

// 新增：检查aim_color变化并调用服务
void SerialDriver::check_aim_color_change(uint8_t current_aim_color)
{
    static auto last_service_call_time = this->now();
    static const double MIN_CALL_INTERVAL = 420.0; // 最小调用间隔1秒
    
    // 只要aim_color是0或1
    if (current_aim_color == 0 || current_aim_color == 1) {
        auto current_time = this->now();
        double time_since_last_call = (current_time - last_service_call_time).seconds();
        
        // 检查是否需要调用服务：值变化了 或者 超过了最小调用间隔
        bool should_call = (current_aim_color != last_aim_color_) || 
                          (last_aim_color_ == 255) || 
                          (time_since_last_call > MIN_CALL_INTERVAL);
        
        if (should_call) {
            RCLCPP_INFO(this->get_logger(), "Valid aim_color: %d (last: %d, interval: %.2fs), calling set_mode services", 
                       current_aim_color, last_aim_color_, time_since_last_call);
            last_aim_color_ = current_aim_color;
            last_service_call_time = current_time;
            call_set_mode_service(current_aim_color);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "aim_color: %d, too soon to call again (%.2fs < %.2fs)", 
                        current_aim_color, time_since_last_call, MIN_CALL_INTERVAL);
        }
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Invalid aim_color: %d, ignoring", current_aim_color);
    }
}

// 新增：调用设置模式服务
void SerialDriver::call_set_mode_service(uint8_t mode)
{
    // 为所有客户端准备并发送请求
    std::vector<std::string> service_names = {
        "armor_detector/set_mode",
        "armor_solver/set_mode", 
        "rune_detector/set_mode",
        "rune_solver/set_mode"
    };

    for (size_t i = 0; i < mode_clients_.size(); ++i) {
        auto client = mode_clients_[i];
        const std::string& service_name = service_names[i];
        
        // 检查服务是否可用 - 不阻塞
        if (!client->service_is_ready()) {
            RCLCPP_DEBUG(this->get_logger(), "Service %s not available yet, skipping", service_name.c_str());
            continue;
        }
        
        // 创建请求
        auto request = std::make_shared<rm_interfaces::srv::SetMode::Request>();
        request->mode = mode;
        
        // 记录详细请求信息
        RCLCPP_INFO(this->get_logger(), "Calling service %s with mode: %d", service_name.c_str(), mode);
        
        // 异步调用服务
        auto future = client->async_send_request(
            request,
            [this, service_name](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture future) {
                this->handle_service_response(future, service_name);
            });
    }
}

// 新增：处理服务响应
void SerialDriver::handle_service_response(
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture future,
    const std::string& service_name)
{
    try {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Successfully set mode for %s: %s", 
                       service_name.c_str(), response->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode for %s: %s", 
                        service_name.c_str(), response->message.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception while getting response from %s: %s", 
                    service_name.c_str(), e.what());
    }
}

SerialDriver::~SerialDriver()
{
    // communicator_ will be destroyed automatically, closing the port in its destructor
    // close_serial(); // No longer needed here
}

void SerialDriver::init_params()
{
    // Declare all parameters that can be set via YAML
    this->declare_parameter("serial_port", "/dev/ttyUSB0"); 
    this->declare_parameter("baud_rate", 115200); 
    this->declare_parameter("cmd_timeout", 10.0);
    this->declare_parameter("timer_period", 0.01);
    this->declare_parameter("reconnect_delay", 5.0);
    this->declare_parameter("max_reconnect_attempts", 10);
    this->declare_parameter("use_fixed_goal", false);
    this->declare_parameter("base_frame_id", "odom"); 
    this->declare_parameter("gimbal_frame_id", "gimbal_link"); 
    this->declare_parameter("default_bullet_speed", 18.0); 

    // Navigation points parameters
    this->declare_parameter("navigation_points.map_frame_id", "map");
    this->declare_parameter("navigation_points.center_point.x", 5.21);
    this->declare_parameter("navigation_points.center_point.y", -4.47);
    this->declare_parameter("navigation_points.center_point.yaw", 0.0); // Yaw in radians
    this->declare_parameter("navigation_points.start_point.x", 0.0);
    this->declare_parameter("navigation_points.start_point.y", 0.0);
    this->declare_parameter("navigation_points.start_point.yaw", 0.0); // Yaw in radians

    // Decision maker parameters
    this->declare_parameter("decision_maker_params.low_blood_threshold", 150);
    this->declare_parameter("decision_maker_params.low_ammo_threshold", 0);

    // 添加helper_point参数声明
    this->declare_parameter("navigation_points.helper_point.x", -0.82);
    this->declare_parameter("navigation_points.helper_point.y", -6.05);
    this->declare_parameter("navigation_points.helper_point.yaw", 0.0);
    
    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int(); 
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
    timer_period_ = this->get_parameter("timer_period").as_double();
    reconnect_delay_ = this->get_parameter("reconnect_delay").as_double();
    max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
    use_fixed_goal_ = this->get_parameter("use_fixed_goal").as_bool();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    gimbal_frame_id_ = this->get_parameter("gimbal_frame_id").as_string();
    default_bullet_speed_ = this->get_parameter("default_bullet_speed").as_double();

    // Get navigation points
    nav_points_map_frame_id_ = this->get_parameter("navigation_points.map_frame_id").as_string();
    
    center_point_.header.frame_id = nav_points_map_frame_id_;
    center_point_.pose.position.x = this->get_parameter("navigation_points.center_point.x").as_double();
    center_point_.pose.position.y = this->get_parameter("navigation_points.center_point.y").as_double();
    center_point_.pose.position.z = 0.0; // Assuming 2D navigation
    tf2::Quaternion q_center;
    q_center.setRPY(0, 0, this->get_parameter("navigation_points.center_point.yaw").as_double());
    center_point_.pose.orientation = tf2::toMsg(q_center);

    start_point_.header.frame_id = nav_points_map_frame_id_;
    start_point_.pose.position.x = this->get_parameter("navigation_points.start_point.x").as_double();
    start_point_.pose.position.y = this->get_parameter("navigation_points.start_point.y").as_double();
    start_point_.pose.position.z = 0.0; // Assuming 2D navigation
    tf2::Quaternion q_start;
    q_start.setRPY(0, 0, this->get_parameter("navigation_points.start_point.yaw").as_double());
    start_point_.pose.orientation = tf2::toMsg(q_start);

    // 读取辅助点坐标
    helper_point_.header.frame_id = nav_points_map_frame_id_;
    helper_point_.pose.position.x = this->get_parameter("navigation_points.helper_point.x").as_double();
    helper_point_.pose.position.y = this->get_parameter("navigation_points.helper_point.y").as_double();
    helper_point_.pose.position.z = 0.0; // Assuming 2D navigation
    tf2::Quaternion q_helper;
    q_helper.setRPY(0, 0, this->get_parameter("navigation_points.helper_point.yaw").as_double());
    helper_point_.pose.orientation = tf2::toMsg(q_helper);
    
    // Get decision maker parameters
    low_blood_threshold_ = this->get_parameter("decision_maker_params.low_blood_threshold").as_int();
    low_ammo_threshold_ = this->get_parameter("decision_maker_params.low_ammo_threshold").as_int();

    RCLCPP_INFO(this->get_logger(), "--- Loaded Parameters ---");
    RCLCPP_INFO(this->get_logger(), "Serial Port: %s (Default: /dev/ttyUSB0)", serial_port_.c_str());
    RCLCPP_INFO(this->get_logger(), "Default Bullet Speed: %.2f (Default: 21.0)", default_bullet_speed_);
    RCLCPP_INFO(this->get_logger(), "Center Point X: %.2f (Default: 0.0)", center_point_.pose.position.x);
    RCLCPP_INFO(this->get_logger(), "Low Blood Threshold: %d (Default: 150)", low_blood_threshold_);
    RCLCPP_INFO(this->get_logger(), "Helper Point X: %.2f (Default: 0.0)", helper_point_.pose.position.x);
    RCLCPP_INFO(this->get_logger(), "--- End Loaded Parameters ---");
}

void SerialDriver::init_state()
{
    gimbal_data_.fill(0.0);
    latest_twist_ = geometry_msgs::msg::Twist();
    last_gimbal_cmd_time_ = this->now();
    last_nav_cmd_time_ = this->now();
}

void SerialDriver::send_nav_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
    // if (is_navigating_to_center_ || is_navigating_to_healing_) {
    //     RCLCPP_WARN(this->get_logger(), "导航已在进行中，忽略重复目标");
    //     return;
    // }

    // 等待 Action 服务器
    if (!nav_client_->wait_for_action_server(std::chrono::milliseconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "导航Action服务不可用 (等待10ms超时)");
        return;
    }
    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    // 添加反馈回调
    send_goal_options.feedback_callback =
        [this](auto, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        double remaining_distance = feedback->distance_remaining;
        RCLCPP_INFO(this->get_logger(), "导航反馈：剩余距离 = %.2f", remaining_distance);
    };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "导航目标返回结果：成功");
            
            // 修改：根据当前导航的目标切换模式
            if (is_navigating_to_center_) {
                move_mode_ = 2; // 自瞄模式
                RCLCPP_INFO(this->get_logger(), "导航到中心点完成，切换模式到自瞄模式(2)");
                is_navigating_to_center_ = false; // 重置标志
            } 
            else if (is_navigating_to_healing_) {
                // 到达回血点后不改变模式，等待decision_maker根据RFID和血量/弹药状态决定
                RCLCPP_INFO(this->get_logger(), "导航到回血点完成，等待血量/弹药恢复");
                // 保持is_navigating_to_healing_为true
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "导航目标返回结果：失败");
        }
    };

    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void SerialDriver::mode_command_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    move_mode_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到模式控制命令: %d", move_mode_);
}

void SerialDriver::check_timeouts()
{
    auto current_time = this->now();
    double time_since_last_cmd = (current_time - last_nav_cmd_time_).seconds();
    RCLCPP_DEBUG(this->get_logger(), "超时检查: 自上次命令已过%.2f秒 (阈值%.2f秒), nav_updated=%d",
               time_since_last_cmd, cmd_timeout_, nav_data_updated_);
    
    if (time_since_last_cmd > cmd_timeout_) {
        if (nav_data_updated_) {
            latest_twist_ = geometry_msgs::msg::Twist();
            nav_data_updated_ = false;
            RCLCPP_WARN(this->get_logger(), "导航命令超时，速度已重置为0");
        }
    }
    
    double gimbal_time_since_last_cmd = (current_time - last_gimbal_cmd_time_).seconds();
    if (gimbal_time_since_last_cmd > cmd_timeout_) {
        if (gimbal_data_updated_) {
            // 不清零所有数据，只标记为未更新
            // gimbal_data_.fill(0.0);  // 不再清零
            gimbal_data_updated_ = false;
            RCLCPP_WARN(this->get_logger(), "自瞄命令超时 (%.2f秒)，但保留最后的云台数据", gimbal_time_since_last_cmd);
        }
    }
}

void SerialDriver::timer_callback()
{
    // 心跳日志
    static int heartbeat_counter = 0;
    heartbeat_counter = (heartbeat_counter + 1) % 100;
    if (heartbeat_counter == 0) {
        RCLCPP_INFO(this->get_logger(), "串口驱动心跳 - 当前模式: %d", move_mode_);
    }
    
    if (communicator_) { 
        communicator_->ensure_connection();
    }
    check_timeouts();
    
    auto current_time = this->now();

    uint32_t previous_mode = move_mode_; 
    
    bool rx_success = false;
    PacketFormat::RxPacket rx_packet{}; // Still need to declare it here for memcpy
    
    if (communicator_ && communicator_->is_connected()) {
        std::vector<uint8_t> received_payload;
        if (communicator_->read_payload(received_payload, PacketFormat::RX_PACKET_SIZE)) {
            if (received_payload.size() == PacketFormat::RX_PACKET_SIZE) {
                memcpy(&rx_packet, received_payload.data(), PacketFormat::RX_PACKET_SIZE);
                rx_success = true;
                
                RCLCPP_INFO(this->get_logger(), "成功接收数据包: 帧长 %zu 字节 (payload size)", received_payload.size());

                float current_pitch_diff = gimbal_data_[0];
                float current_yaw_diff = gimbal_data_[1];
                float current_distance = gimbal_data_[2];
                uint32_t current_fire_advice = static_cast<uint32_t>(gimbal_data_[3]);

                if (decision_maker_) {
                    rm_serial::DecisionResult decision = decision_maker_->make_decision(
                        rx_packet, // Pass the successfully received packet
                        move_mode_,
                        is_navigating_to_center_,
                        is_navigating_to_start_,
                        is_navigating_to_healing_,  // 添加这个参数
                        current_time
                    );

                    previous_mode = move_mode_; 
                    move_mode_ = decision.new_move_mode;
                    is_navigating_to_center_ = decision.updated_is_nav_to_center;
                    is_navigating_to_start_ = decision.updated_is_nav_to_start;
                    is_navigating_to_healing_ = decision.updated_is_nav_to_healing;  // 更新这个变量

                    if (decision.send_goal) {
                        send_nav_goal(decision.goal_to_send);
                    }
                     if (previous_mode != move_mode_) { 
                        RCLCPP_INFO(this->get_logger(), "SerialDriver: Mode changed by DecisionMaker: %d -> %d", 
                                    previous_mode, move_mode_);
                    }
                }

                gimbal_data_[0] = current_pitch_diff;
                gimbal_data_[1] = current_yaw_diff;
                gimbal_data_[2] = current_distance;
                gimbal_data_[3] = static_cast<float>(current_fire_advice);
                // 处理接收到的数据包
                RCLCPP_INFO(this->get_logger(), 
                    "RX (processed by SerialDriver): aim_color=%d, roll=%.2f, pitch_g=%.2f, yaw_g=%.2f, hp=%d, game_status=%d",
                    rx_packet.aim_color, rx_packet.roll, 
                    rx_packet.pitch_gimbal, rx_packet.yaw_gimbal, rx_packet.blood, rx_packet.game_status);
                
                check_aim_color_change(rx_packet.aim_color); // 检查是否需要调用服务
                RCLCPP_INFO(this->get_logger(), "aim_color=%d, last_aim_color=%d", rx_packet.aim_color, last_aim_color_);
                
            } else {
                 RCLCPP_WARN(this->get_logger(), "Read payload size mismatch. Expected %zu, Got %zu", PacketFormat::RX_PACKET_SIZE, received_payload.size());
            }
        }
    }
    
    // 调用 RosInterface 发布数据
    if (ros_interface_) {
        ros_interface_->publish_feedback_data(current_time, rx_success ? &rx_packet : nullptr);
    }
    
    // 第二步：无论是否接收到数据，都准备发送数据 (This part remains in SerialDriver)
    if (communicator_ && communicator_->is_connected()) {
        PacketFormat::TxPacket tx_packet_struct{};
        tx_packet_struct.pitch_diff = gimbal_data_[0];
        tx_packet_struct.yaw_diff = gimbal_data_[1];
        tx_packet_struct.distance = gimbal_data_[2];
        tx_packet_struct.fire_advice = static_cast<uint32_t>(gimbal_data_[3]);
        tx_packet_struct.vx = latest_twist_.linear.x;
        tx_packet_struct.vy = latest_twist_.linear.y;
        tx_packet_struct.vw = latest_twist_.angular.z;
        tx_packet_struct.move_mode = move_mode_;
        
        RCLCPP_INFO(this->get_logger(), "发送速度: vx=%.2f, vy=%.2f, vw=%.2f, mode=%d, pitch=%.2f, yaw=%.2f, dist=%.2f, fire=%d", 
                tx_packet_struct.vx, tx_packet_struct.vy, tx_packet_struct.vw, tx_packet_struct.move_mode, tx_packet_struct.pitch_diff, tx_packet_struct.yaw_diff, tx_packet_struct.distance, tx_packet_struct.fire_advice);
        
        std::vector<uint8_t> tx_payload(PacketFormat::TX_PACKET_SIZE);
        memcpy(tx_payload.data(), &tx_packet_struct, PacketFormat::TX_PACKET_SIZE);
        
        if (!communicator_->write_payload(tx_payload)) {
            RCLCPP_ERROR(this->get_logger(), "TX: 发送失败 via communicator");
        }
    }
    
    // 第三步：发布消息与TF变换，无论串口是否有效 - Moved to RosInterface
    // gimbal_state_pub_->publish(gimbal_state);
    // gimbal_pub_->publish(serial_data);
    // tf_broadcaster_->sendTransform(t);
    // tf_broadcaster_->sendTransform(rectify_t);
    
    // 固定目标点发布功能（调试使用，可通过参数控制）
    bool use_fixed_goal = this->get_parameter("use_fixed_goal").as_bool();
    if (use_fixed_goal) {
        center_point_.header.stamp = current_time;
        nav_goal_pub_->publish(center_point_);
        RCLCPP_INFO(this->get_logger(), "发布固定目标点：中心点");
    }
}
// timer_callback 函数结束


void SerialDriver::gimbal_cmd_callback(std::shared_ptr<const rm_interfaces::msg::GimbalCmd> msg)
{
    // 这里电控的云台控制和正常的yaw、pitch控制是相反的

    gimbal_data_[0] = msg->yaw_diff;  // 直接使用pitch_diff字段
    gimbal_data_[1] = msg->pitch_diff;    // 直接使用yaw_diff字段
    gimbal_data_[2] = msg->distance;    // 距离字段
    gimbal_data_[3] = static_cast<float>(msg->fire_advice ? 1 : 0);  // 使用fire_advice字段
    
    last_gimbal_cmd_time_ = this->now();
    gimbal_data_updated_ = true;

    // 添加日志记录接收到的自瞄数据
    RCLCPP_DEBUG(this->get_logger(), "接收自瞄数据: pitch=%.2f, yaw=%.2f, dist=%.2f, fire=%d", 
               gimbal_data_[0], gimbal_data_[1], gimbal_data_[2], static_cast<int>(gimbal_data_[3]));
}

void SerialDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_twist_ = *msg;
    last_nav_cmd_time_ = this->now();
    nav_data_updated_ = true;
    RCLCPP_INFO(this->get_logger(), "收到速度命令: vx=%.2f, vy=%.2f, vw=%.2f, 模式=%d",
              msg->linear.x, msg->linear.y, msg->angular.z, move_mode_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialDriver>());
    rclcpp::shutdown();
    return 0;
}
