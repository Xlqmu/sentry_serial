//// filepath: /home/sentry_nav2/src/rm_interfaces/src/serial.cpp
#include "rm_serial/serial_driver.hpp"
#include <chrono>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// 数据包格式定义
struct PacketFormat {
    // 协议常量定义
    enum : uint8_t {
        FRAME_HEADER = 0xFF,  // 帧头
        FRAME_TAIL = 0x0D     // 帧尾
    };

    struct __attribute__((packed)) RxPacket {
        uint8_t decator_mode;
        float roll;
        float pitch_gimbal;
        float yaw_gimbal;
        float pitch_chassis;
        float yaw_chassis;
        uint8_t game_status;
        uint16_t remaining_time;
        uint16_t blood;
        bool is_free_rebirth;
    };
    
    struct __attribute__((packed)) TxPacket {
        float pitch_diff;
        float yaw_diff;
        float distance;
        uint32_t fire_advice;
        float vx;
        float vy;
        float vw;
        uint32_t move_mode;
    };
    
    // 数据包本身大小
    static const size_t RX_PACKET_SIZE = sizeof(RxPacket);
    static const size_t TX_PACKET_SIZE = sizeof(TxPacket);
    
    // 帧总大小 = 帧头(1字节) + 数据包 + 校验和(1字节) + 帧尾(1字节)
    static const size_t RX_FRAME_SIZE = RX_PACKET_SIZE + 3;
    static const size_t TX_FRAME_SIZE = TX_PACKET_SIZE + 3;
};


SerialDriver::SerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options), 
  serial_fd_(-1), // 串口文件描述符
  move_mode_(4),  // 默认模式为导航模式
  nav_data_updated_(true), // 导航数据更新标志
  gimbal_data_updated_(false), // 云台数据更新标志
  reconnect_attempts_(0), // 重连次数
  last_reconnect_time_(0), // 上次重连时间
  is_navigating_to_center_(false), // 是否正在导航到中心点
  is_navigating_to_start_(false) // 是否正在导航到起点
{
    init_params();
    init_state();
    
    RCLCPP_INFO(this->get_logger(), "RxPacket size: %zu, TxPacket size: %zu",
               PacketFormat::RX_PACKET_SIZE, PacketFormat::TX_PACKET_SIZE);
    
    if (!init_serial()) {
        RCLCPP_ERROR(this->get_logger(), "串口初始化失败");
    }

    auto qos = rclcpp::QoS(10).best_effort();
    
    // 初始化通信接口
    gimbal_state_pub_ = this->create_publisher<rm_interfaces::msg::GimbalState>(
        "/serial/gimbal_joint_state", qos);
    gimbal_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", qos);
    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/armor_solver/cmd_gimbal", qos,
        std::bind(&SerialDriver::gimbal_cmd_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/red_standard_robot1/cmd_vel", qos,
        std::bind(&SerialDriver::cmd_vel_callback, this, std::placeholders::_1));
    // 添加导航目标发布器
    nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);
    // 设置导航点（注意，此处坐标需与地图坐标系一致）
    center_point_.header.frame_id = "map";
    center_point_.pose.position.x = 2.90;   // 修改为实际中心点坐标
    center_point_.pose.position.y = 0.25;
    center_point_.pose.orientation.w = 1.0; // 无旋转
    
    start_point_.header.frame_id = "map";
    start_point_.pose.position.x = 0.0;    // 修改为实际起点坐标
    start_point_.pose.position.y = 0.0; 
    start_point_.pose.orientation.w = 1.0;

    // 创建定时器
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&SerialDriver::timer_callback, this));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose");

    // 初始化TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    base_frame_id_ = "odom";
    gimbal_frame_id_ = "gimbal_link";
}

SerialDriver::~SerialDriver()
{
    close_serial();
}

void SerialDriver::init_params()
{
    // 原有参数
    this->declare_parameter("serial_port", "/dev/ttyUART"); 
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("cmd_timeout", 10.0);
    this->declare_parameter("timer_period", 0.01);
    this->declare_parameter("reconnect_delay", 5.0);
    this->declare_parameter("max_reconnect_attempts", 10);
    // 新增参数：是否发布固定目标点（在调试时可单独控制目标发布）
    this->declare_parameter("use_fixed_goal", false);

    serial_port_ = this->get_parameter("serial_port").as_string();
    baud_rate_ = this->get_parameter("baud_rate").as_int();
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
    timer_period_ = this->get_parameter("timer_period").as_double();
    reconnect_delay_ = this->get_parameter("reconnect_delay").as_double();
    max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
}

void SerialDriver::init_state()
{
    gimbal_data_.fill(0.0);
    latest_twist_ = geometry_msgs::msg::Twist();
    last_gimbal_cmd_time_ = this->now();
    last_nav_cmd_time_ = this->now();
    reconnect_attempts_ = 0;
    last_reconnect_time_ = this->now().seconds();
}

bool SerialDriver::init_serial()
{
    close_serial();
    
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s", strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "获取串口属性失败: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "设置串口属性失败: %s", strerror(errno));
        close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "串口初始化成功: %s @%d", serial_port_.c_str(), baud_rate_);
    reconnect_attempts_ = 0;
    return true;
}

void SerialDriver::close_serial()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

// uint8_t SerialDriver::calculate_checksum(const uint8_t* data, size_t length)
// {
//     uint8_t checksum = 0;
//     for (size_t i = 0; i < length; i++) {
//         checksum ^= data[i];  // 异或校验
//     }
//     return checksum;
// }

bool SerialDriver::ensure_serial_connection()
{
    if (serial_fd_ >= 0) {
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) == 0) {
            return true;
        }
        
        RCLCPP_ERROR(this->get_logger(), "串口连接异常: %s", strerror(errno));
        close_serial();
    }
    
    double current_time = this->now().seconds();
    if ((current_time - last_reconnect_time_) < reconnect_delay_) {
        return false;
    }
    
    if (max_reconnect_attempts_ > 0 && reconnect_attempts_ >= max_reconnect_attempts_) {
        RCLCPP_ERROR(this->get_logger(), "已达到最大重连尝试次数 (%d)", max_reconnect_attempts_);
        return false;
    }
    
    reconnect_attempts_++;
    last_reconnect_time_ = current_time;
    
    RCLCPP_INFO(this->get_logger(), "尝试重新连接串口 (尝试 %d/%d)...",
               reconnect_attempts_, max_reconnect_attempts_ > 0 ? max_reconnect_attempts_ : -1);
    
    if (init_serial()) {
        RCLCPP_INFO(this->get_logger(), "串口重连成功!");
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "串口重连失败，将在 %.1f 秒后重试", reconnect_delay_);
        return false;
    }
}

// 修改 write_data 方法，添加帧结构
bool SerialDriver::write_data(const std::vector<uint8_t>& data)
{
    if (serial_fd_ < 0) return false;
    
    // 创建完整帧
    std::vector<uint8_t> frame;
    // 添加帧头
    frame.push_back(PacketFormat::FRAME_HEADER);
    // 添加数据部分
    frame.insert(frame.end(), data.begin(), data.end());
    // 计算并添加校验和
    uint8_t checksum = calculate_checksum(data.data(), data.size());
    frame.push_back(checksum);
    // 添加帧尾
    frame.push_back(PacketFormat::FRAME_TAIL);
    
    RCLCPP_INFO(this->get_logger(), "TX: 发送数据帧，长度 %zu 字节", frame.size());
    
    // 发送完整帧
    ssize_t total_written = 0;
    while (total_written < static_cast<ssize_t>(frame.size())) {
        ssize_t written = write(serial_fd_, frame.data() + total_written, frame.size() - total_written);
        if (written <= 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue;
            }
            
            RCLCPP_ERROR(this->get_logger(), "写入数据失败: %s", strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                close_serial();
                return false;
            }
            return false;
        }
        total_written += written;
    }
    return true;
}

bool SerialDriver::read_data(std::vector<uint8_t>& data, size_t length)
{
    if (serial_fd_ < 0) return false;
    
    data.resize(length);
    ssize_t bytes_read = read(serial_fd_, data.data(), length);
    
    if (bytes_read > 0) {
        // 调试输出
        std::stringstream ss;
        ss << "读取到数据: ";
        for (size_t i = 0; i < std::min(bytes_read, ssize_t(16)); i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(data[i]) << " ";
        }
        if (bytes_read > 16) ss << "...";
        RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
        
        data.resize(bytes_read);
        return true;
    } else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "读取数据失败: %s", strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                close_serial();
            }
        }
    }
    
    data.clear();
    return false;
}



void SerialDriver::send_nav_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
    // 等待 Action 服务器
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "导航Action服务不可用");
        return;
    }
    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

    // 添加反馈回调（反馈消息类型依据你的 nav2_msgs/action/NavigateToPose 定义）
    send_goal_options.feedback_callback =
        [this](auto, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        // 假设 feedback 中有 remaining_distance 字段（请参阅实际文档）
        double remaining = feedback->distance_remaining;
        RCLCPP_INFO(this->get_logger(), "导航反馈：剩余距离 = %.2f", remaining);
    };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "导航目标返回结果：成功");
            if (is_navigating_to_center_) {
                // 如果反馈没有触发，仍在中心点目标状态，则也切换模式
                move_mode_ = 2;
                RCLCPP_INFO(this->get_logger(), "导航到中心点完成，切换模式到自瞄模式(2)");
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
            // gimbal_data_.fill(0.0);  // 注释掉，不再清零
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
    
    // 确保串口连接 & 检查超时
    ensure_serial_connection();
    check_timeouts();
    
    auto current_time = this->now();
    
    // 准备TF变换
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = base_frame_id_;
    t.child_frame_id = gimbal_frame_id_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0;
    
    geometry_msgs::msg::TransformStamped rectify_t;
    rectify_t.header.stamp = current_time;
    rectify_t.header.frame_id = base_frame_id_;
    rectify_t.child_frame_id = base_frame_id_ + "_rectify";
    rectify_t.transform.translation.x = 0.0;
    rectify_t.transform.translation.y = 0.0;
    rectify_t.transform.translation.z = 0.0;
    rectify_t.transform.rotation.w = 1.0;
    
    // 初始化发布消息
    rm_interfaces::msg::GimbalState gimbal_state;
    gimbal_state.pitch = 0.0;
    gimbal_state.yaw = 0.0;
    
    rm_interfaces::msg::SerialReceiveData serial_data;
    serial_data.header.stamp = current_time;
    serial_data.header.frame_id = "gimbal_link";
    serial_data.mode = 0;
    serial_data.bullet_speed = 18.0;
    serial_data.roll = 0.0;
    serial_data.yaw = 0.0;
    serial_data.pitch = 0.0;
    
    auto judge_data = rm_interfaces::msg::JudgeSystemData();
    judge_data.game_status = 0;
    judge_data.remaining_time = 0;
    judge_data.blood = 400;
    judge_data.outpost_hp = 0;
    
    bool mode_changed = false;
    uint32_t previous_mode = move_mode_;
    
    // 接收数据
    std::vector<uint8_t> rx_data;
    bool rx_success = false;
    PacketFormat::RxPacket rx_packet{};
    
    // 第一步：尝试接收数据
    if (serial_fd_ >= 0) {
        // 尝试读取足够多的数据以获取至少一个完整帧
        std::vector<uint8_t> buffer;
        const size_t READ_BUFFER_SIZE = PacketFormat::RX_FRAME_SIZE * 2; // 读取缓冲区大小是帧大小的两倍
        
        if (read_data(buffer, READ_BUFFER_SIZE)) {
            if (buffer.size() >= PacketFormat::RX_FRAME_SIZE) {
                RCLCPP_DEBUG(this->get_logger(), "读取到 %zu 字节的数据", buffer.size());
                
                // 在缓冲区中查找帧结构
                for (size_t i = 0; i <= buffer.size() - PacketFormat::RX_FRAME_SIZE; i++) {
                    // 检查是否有帧头
                    if (buffer[i] == PacketFormat::FRAME_HEADER) {
                        RCLCPP_DEBUG(this->get_logger(), "找到帧头，位置 %zu", i);
                        // 检查是否有帧尾
                        if (buffer[i + PacketFormat::RX_FRAME_SIZE - 1] == PacketFormat::FRAME_TAIL) {
                            RCLCPP_DEBUG(this->get_logger(), "找到帧尾，位置 %zu", i + PacketFormat::RX_FRAME_SIZE - 1);
                            // 提取数据部分
                            std::vector<uint8_t> data_part(
                                buffer.begin() + i + 1, 
                                buffer.begin() + i + 1 + PacketFormat::RX_PACKET_SIZE
                            );
                            
                            // 获取校验和
                            uint8_t received_checksum = buffer[i + 1 + PacketFormat::RX_PACKET_SIZE];
                            uint8_t calculated_checksum = calculate_checksum(data_part.data(), data_part.size());
                            
                            // 验证校验和
                            // if (received_checksum == calculated_checksum) {
                            //     // 校验通过，解析数据包
                                memcpy(&rx_packet, data_part.data(), PacketFormat::RX_PACKET_SIZE);
                                rx_success = true;
                                
                                RCLCPP_INFO(this->get_logger(), "成功接收数据包: 帧长 %zu 字节", PacketFormat::RX_FRAME_SIZE);
                                
                                // 更新消息数据
                                gimbal_state.pitch = rx_packet.pitch_chassis;
                                gimbal_state.yaw = rx_packet.yaw_chassis;
                                
                                serial_data.mode = rx_packet.decator_mode;
                                serial_data.roll = rx_packet.roll;
                                serial_data.yaw = rx_packet.yaw_gimbal;
                                serial_data.pitch = rx_packet.pitch_gimbal;
                                judge_data.game_status = rx_packet.game_status & 0x0F;
                                judge_data.remaining_time = rx_packet.remaining_time;
                                judge_data.blood = rx_packet.blood;
                                judge_data.outpost_hp = 0;
                                
                                serial_data.judge_system_data = judge_data;
    
                                // 根据比赛状态和血量进行目标选择
                                uint8_t game_status = judge_data.game_status;
                                uint16_t blood = judge_data.blood;

                                // 确保在模式切换前保存当前的自瞄数据
                                float current_pitch_diff = gimbal_data_[0];
                                float current_yaw_diff = gimbal_data_[1];
                                float current_distance = gimbal_data_[2];
                                uint32_t current_fire_advice = static_cast<uint32_t>(gimbal_data_[3]);

                                if (is_navigating_to_center_ && rx_packet.is_free_rebirth) {
                                    move_mode_ = 2;
                                    is_navigating_to_center_ = false;
                                    RCLCPP_INFO(this->get_logger(), "检测到is_free_rebirth为真，切换到自瞄模式(2)");
                                }

                                // 优先级1: 血量低 => 立即返回起点(无论当前位置或状态)
                                if (blood <= 150) {
                                    // 血量低，返回起点
                                    if (move_mode_ != 4) {
                                        move_mode_ = 4;
                                        mode_changed = true;
                                        RCLCPP_INFO(this->get_logger(), "血量过低 (%d)，切换到导航模式(4)返回起点", blood);
                                    }
                                    start_point_.header.stamp = current_time;
                                    send_nav_goal(start_point_);
                                    is_navigating_to_start_ = true;
                                    is_navigating_to_center_ = false;
                                    RCLCPP_INFO(this->get_logger(), "发送导航目标：返回起始点（血量：%d）", blood);
                                }
                                else if (game_status == 4 && blood > 150) {
                                    // 血量充足，前往中心点
                                    if (move_mode_ != 4 && move_mode_ != 2) {  
                                        move_mode_ = 4;
                                        mode_changed = true;
                                        RCLCPP_INFO(this->get_logger(), "状态良好(血量：%d)，切换到导航模式(4)", blood);
                                    }
                                    center_point_.header.stamp = current_time;
                                    send_nav_goal(center_point_);
                                    is_navigating_to_center_ = true;
                                    is_navigating_to_start_ = false;
                                    RCLCPP_INFO(this->get_logger(), "发送导航目标：前往中心点");
                                }
                                else {
                                    // 其他情况，切换至自瞄模式
                                    if (move_mode_ != 2) {
                                        move_mode_ = 4;
                                        mode_changed = true;
                                        RCLCPP_INFO(this->get_logger(), "非正常比赛状态(状态=%d,血量=%d)，切换到自瞄模式(2)", game_status, blood);
                                    }
                                    is_navigating_to_center_ = false;
                                    is_navigating_to_start_ = false;
                                }

                                gimbal_data_[0] = current_yaw_diff;
                                gimbal_data_[1] = current_pitch_diff;
                                gimbal_data_[2] = current_distance;
                                gimbal_data_[3] = static_cast<float>(current_fire_advice);
            
                                // 记录模式变更信息
                                if (mode_changed) {
                                    RCLCPP_INFO(this->get_logger(), "模式变更: %d -> %d, 状态=%d, 血量=%d", 
                                            previous_mode, move_mode_, game_status, blood);
                                }
        
                                // 更新TF变换
                                tf2::Quaternion q;
                                q.setRPY(rx_packet.roll, -rx_packet.pitch_gimbal, rx_packet.yaw_gimbal);
                                t.transform.rotation.x = q.x();
                                t.transform.rotation.y = q.y();
                                t.transform.rotation.z = q.z();
                                t.transform.rotation.w = q.w();
                                
                                tf2::Quaternion q_rectify;
                                q_rectify.setRPY(rx_packet.roll, 0.0, 0.0);
                                rectify_t.transform.rotation.x = q_rectify.x();
                                rectify_t.transform.rotation.y = q_rectify.y();
                                rectify_t.transform.rotation.z = q_rectify.z();
                                rectify_t.transform.rotation.w = q_rectify.w();
                
                                RCLCPP_INFO(this->get_logger(), 
                                    "RX: mode=%d, roll=%.2f, pitch_g=%.2f, yaw_g=%.2f, hp=%d, game_status=%d, is_free_rebirth=%d",
                                    rx_packet.decator_mode, rx_packet.roll, 
                                    rx_packet.pitch_gimbal, rx_packet.yaw_gimbal, rx_packet.blood, rx_packet.game_status, rx_packet.is_free_rebirth);
                                break;
                            // } else {
                            //     RCLCPP_WARN(this->get_logger(), "校验和错误: 接收 %02X, 计算 %02X", received_checksum, calculated_checksum);
                            // }
                        }
                    }
                }
                
                if (!rx_success) {
                    RCLCPP_WARN(this->get_logger(), "未找到完整帧，丢弃 %zu 字节数据", buffer.size());
                }
            }
        }
    }
    
    // 第二步：无论是否接收到数据，都准备发送数据
    if (serial_fd_ >= 0) {
        // 准备发送数据包到下位机
        PacketFormat::TxPacket tx_packet{};
        tx_packet.pitch_diff = gimbal_data_[0];
        tx_packet.yaw_diff = gimbal_data_[1];
        tx_packet.distance = gimbal_data_[2];
        tx_packet.fire_advice = static_cast<uint32_t>(gimbal_data_[3]);
        tx_packet.vx = latest_twist_.linear.x;
        tx_packet.vy = latest_twist_.linear.y;
        tx_packet.vw = latest_twist_.angular.z;
        tx_packet.move_mode = move_mode_;
        
        RCLCPP_INFO(this->get_logger(), "发送速度: vx=%.2f, vy=%.2f, vw=%.2f, mode=%d, pitch=%.2f, yaw=%.2f, dist=%.2f, fire=%d", 
                tx_packet.vx, tx_packet.vy, tx_packet.vw, tx_packet.move_mode, tx_packet.pitch_diff, tx_packet.yaw_diff, tx_packet.distance, tx_packet.fire_advice);
        
        std::vector<uint8_t> tx_data(PacketFormat::TX_PACKET_SIZE);
        memcpy(tx_data.data(), &tx_packet, PacketFormat::TX_PACKET_SIZE);
        
        if (!write_data(tx_data)) {
            RCLCPP_ERROR(this->get_logger(), "TX: 发送失败");
        }
    }
    
    // 第三步：发布消息与TF变换，无论串口是否有效
    gimbal_state_pub_->publish(gimbal_state);
    gimbal_pub_->publish(serial_data);
    tf_broadcaster_->sendTransform(t);
    tf_broadcaster_->sendTransform(rectify_t);
    
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
    gimbal_data_[0] = msg->pitch_diff;  // 直接使用pitch_diff字段
    gimbal_data_[1] = msg->yaw_diff;    // 直接使用yaw_diff字段
    gimbal_data_[2] = msg->distance;    // 距离字段
    gimbal_data_[3] = static_cast<float>(msg->fire_advice ? 1 : 0);  // 使用fire_advice字段
    
    last_gimbal_cmd_time_ = this->now();
    gimbal_data_updated_ = true;

    // 添加日志记录接收到的自瞄数据
    RCLCPP_INFO(this->get_logger(), "接收自瞄数据: pitch=%.2f, yaw=%.2f, dist=%.2f, fire=%d", 
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
