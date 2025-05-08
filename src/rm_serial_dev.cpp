#include "rm_serial/rm_serial.hpp" // 应包含 rclcpp, packet_typedef.hpp, crc8_crc16.hpp, 及所有消息类型
#include "rm_serial/packet_typedef.hpp"
#include "rm_serial/crc8_crc16.hpp"   

#include <chrono>
#include <cstring> // For memcpy
#include <vector>
#include <string>    // For strerror
#include <sstream>   // For debug
#include <iomanip>   // For debug
#include <fcntl.h>   // For open
#include <termios.h> // For tcgetattr, tcsetattr
#include <unistd.h>  // For write, read, close
#include <algorithm> // For std::find, std::min
#include <cmath>     // For NAN

// SerialDriver 构造函数
SerialDriver::SerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options),
  serial_fd_(-1),
  reconnect_attempts_(0),
  last_reconnect_time_(0),
  is_navigating_to_center_(false),
  is_navigating_to_start_(false),
  game_status_valid_(false),
  all_robot_hp_valid_(false),
  robot_gimbal_pose_valid_(false),
  robot_chassis_pose_valid_(false),
  if_reborn_data_valid_(false)
  // event_data_valid_(false) // 如果添加 ReceiveEventData
{
    init_params();
    init_state_variables();

    RCLCPP_INFO(this->get_logger(), "SendRobotCmdData size: %zu", sizeof(rm_serial::SendRobotCmdData));
    RCLCPP_INFO(this->get_logger(), "ReceiveGameStatusData size: %zu", sizeof(rm_serial::ReceiveGameStatusData));
    // 可以为其他重要的数据包类型添加日志

    if (!init_serial()) {
        RCLCPP_ERROR(this->get_logger(), "串口初始化失败，节点将尝试重连。");
    }

    auto qos = rclcpp::QoS(10).best_effort();

    gimbal_state_pub_ = this->create_publisher<rm_interfaces::msg::GimbalState>(
        "/serial/gimbal_joint_state", qos);
    gimbal_pub_ = this->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", qos);
    gimbal_cmd_sub_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "/armor_solver/cmd_gimbal", qos,
        std::bind(&SerialDriver::gimbal_cmd_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/red_standard_robot1/cmd_vel", qos, // TODO: Consider making robot name a parameter
        std::bind(&SerialDriver::cmd_vel_callback, this, std::placeholders::_1));
    nav_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10);

    // 导航点配置
    center_point_.header.frame_id = "map";
    center_point_.pose.position.x = 2.90;
    center_point_.pose.position.y = 0.25;
    center_point_.pose.orientation.w = 1.0;

    start_point_.header.frame_id = "map";
    start_point_.pose.position.x = 0.0;
    start_point_.pose.position.y = 0.0;
    start_point_.pose.orientation.w = 1.0;

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&SerialDriver::timer_callback, this));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/red_standard_robot1/navigate_to_pose"); // TODO: Parameterize action server name

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // base_frame_id_ 和 gimbal_frame_id_ 在 init_params 中获取或设置默认值
}

SerialDriver::~SerialDriver()
{
    close_serial();
}

void SerialDriver::init_params()
{
    this->declare_parameter("serial_port", "/dev/ttyUART");
    this->declare_parameter("baud_rate", 115200); // 虽然直接在init_serial中使用，但声明参数是好习惯
    this->declare_parameter("cmd_timeout", 10.0);
    this->declare_parameter("timer_period", 0.01);
    this->declare_parameter("reconnect_delay", 5.0);
    this->declare_parameter("max_reconnect_attempts", 10);
    this->declare_parameter("use_fixed_goal", false);
    this->declare_parameter("base_frame_id", "odom"); // 或 "base_link"
    this->declare_parameter("gimbal_frame_id", "gimbal_link");
    this->declare_parameter("self_robot_id_for_hp", "red_7"); // 例如 "red_7" 或 "blue_1"

    serial_port_ = this->get_parameter("serial_port").as_string();
    // baud_rate_ = this->get_parameter("baud_rate").as_int(); // 在 init_serial 中直接使用 B115200
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
    timer_period_ = this->get_parameter("timer_period").as_double();
    reconnect_delay_ = this->get_parameter("reconnect_delay").as_double();
    max_reconnect_attempts_ = this->get_parameter("max_reconnect_attempts").as_int();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    gimbal_frame_id_ = this->get_parameter("gimbal_frame_id").as_string();
    self_robot_id_for_hp_ = this->get_parameter("self_robot_id_for_hp").as_string();

}

void SerialDriver::init_state_variables()
{
    // 初始化发送命令结构体
    latest_send_cmd_.frame_header.sof = rm_serial::SOF_SEND;
    latest_send_cmd_.frame_header.id = rm_serial::ID_ROBOT_CMD_DATA;
    // len 和 crc 会在发送前计算
    latest_send_cmd_.time_stamp = 0;
    latest_send_cmd_.data.speed_vector.vx = 0.0f;
    latest_send_cmd_.data.speed_vector.vy = 0.0f;
    latest_send_cmd_.data.speed_vector.vw = 0.0f;
    latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 默认导航模式
    latest_send_cmd_.data.gimbal_vector.pitch_diff = 0.0f;
    latest_send_cmd_.data.gimbal_vector.yaw_diff = 0.0f;
    latest_send_cmd_.data.move_vector.distance = 0.0f;
    latest_send_cmd_.data.fire_vector.fire_advice = 0;

    // 初始化时间戳和重连计数
    last_gimbal_cmd_time_ = this->now();
    last_nav_cmd_time_ = this->now();
    reconnect_attempts_ = 0;
    last_reconnect_time_ = this->now().seconds();

    // 初始化数据有效性标志
    game_status_valid_ = false;
    all_robot_hp_valid_ = false;
    robot_gimbal_pose_valid_ = false;
    robot_chassis_pose_valid_ = false;
    if_reborn_data_valid_ = false;
    // event_data_valid_ = false; // 如果添加 ReceiveEventData
}

bool SerialDriver::init_serial()
{
    close_serial();
    serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口 '%s': %s", serial_port_.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "获取串口属性失败: %s", strerror(errno));
        close_serial();
        return false;
    }

    cfsetospeed(&tty, B115200); // 设置波特率
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);    // 忽略调制解调器控制线, 启用接收
    tty.c_cflag &= ~CSIZE;              // 清除数据位掩码
    tty.c_cflag |= CS8;                 // 8数据位
    tty.c_cflag &= ~PARENB;             // 无奇偶校验
    tty.c_cflag &= ~CSTOPB;             // 1停止位
    tty.c_cflag &= ~CRTSCTS;            // 无硬件流控

    // 输入模式标志
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    // 本地模式标志 (关闭回显、规范模式、信号字符、扩展输入处理)
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    // 输出模式标志 (关闭实现定义的输出处理)
    tty.c_oflag &= ~OPOST;

    // 控制字符 (非阻塞读取)
    tty.c_cc[VMIN] = 0;  // 读取的最小字节数
    tty.c_cc[VTIME] = 0; // 读取超时时间 (单位: 0.1秒)

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "设置串口属性失败: %s", strerror(errno));
        close_serial();
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "串口 '%s' 初始化成功 @115200bps", serial_port_.c_str());
    reconnect_attempts_ = 0; // 重置重连尝试次数
    return true;
}

void SerialDriver::close_serial()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(this->get_logger(), "串口已关闭");
    }
}

bool SerialDriver::ensure_serial_connection()
{
    if (serial_fd_ >= 0) {
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) == 0) {
            return true; // 连接正常
        }
        RCLCPP_WARN(this->get_logger(), "串口连接异常 (tcgetattr failed): %s", strerror(errno));
        close_serial(); // 关闭无效的描述符
    }

    double current_time = this->now().seconds();
    if ((current_time - last_reconnect_time_) < reconnect_delay_) {
        return false; // 未到重连时间
    }

    if (max_reconnect_attempts_ > 0 && reconnect_attempts_ >= max_reconnect_attempts_) {
        RCLCPP_ERROR(this->get_logger(), "已达到最大重连尝试次数 (%d)", max_reconnect_attempts_);
        return false;
    }

    reconnect_attempts_++;
    last_reconnect_time_ = current_time;
    RCLCPP_INFO(this->get_logger(), "尝试重新连接串口 (第 %d 次)...", reconnect_attempts_);

    if (init_serial()) {
        RCLCPP_INFO(this->get_logger(), "串口重连成功!");
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "串口重连失败，将在 %.1f 秒后重试", reconnect_delay_);
        return false;
    }
}

bool SerialDriver::send_frame(const std::vector<uint8_t>& frame)
{
    if (serial_fd_ < 0 || frame.empty()) {
        return false;
    }

    ssize_t total_written = 0;
    const ssize_t frame_size = static_cast<ssize_t>(frame.size());

    while (total_written < frame_size) {
        ssize_t written = write(serial_fd_, frame.data() + total_written, frame_size - total_written);
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TX: 串口写入缓冲区满 (EAGAIN/EWOULDBLOCK).");
                // 可以选择在这里等待一小段时间或在下一次timer_callback重试
                // 为避免阻塞timer_callback，这里返回false，让上层决定是否重试
                return false;
            }
            RCLCPP_ERROR(this->get_logger(), "TX: 写入数据失败: %s", strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                close_serial(); // 严重错误，关闭串口
            }
            return false;
        } else if (written == 0) {
            // 理论上 O_NONBLOCK 下 write 不应返回0除非请求写入0字节
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TX: 串口写入返回0字节.");
            return false; // 视为未成功发送
        }
        total_written += written;
    }
    // RCLCPP_DEBUG(this->get_logger(), "TX: 成功发送 %zu 字节.", frame.size());
    return true;
}

void SerialDriver::prepare_and_send_robot_cmd()
{
    if (serial_fd_ < 0) return;

    rm_serial::SendRobotCmdData packet_to_send = latest_send_cmd_; // 复制一份以修改时间戳和CRC
    packet_to_send.time_stamp = static_cast<uint32_t>(this->now().seconds() * 1000.0); // 毫秒级时间戳

    // 1. 填充帧头 (sof, id 已在 init_state_variables 或回调中设置)
    packet_to_send.frame_header.len = sizeof(packet_to_send.time_stamp) + sizeof(packet_to_send.data);

    // 2. 计算帧头 CRC8
    uint8_t header_crc_payload[3] = {
        packet_to_send.frame_header.sof,
        packet_to_send.frame_header.len,
        packet_to_send.frame_header.id
    };
    packet_to_send.frame_header.crc = crc8::get_CRC8_check_sum(header_crc_payload, 3, 0);

    // 3. 计算整个数据包的 CRC16 (帧头 + 时间戳 + 数据段)
    std::vector<uint8_t> crc16_payload_buffer;
    crc16_payload_buffer.resize(sizeof(rm_serial::HeaderFrame) + packet_to_send.frame_header.len);
    uint8_t* ptr = crc16_payload_buffer.data();
    memcpy(ptr, &packet_to_send.frame_header, sizeof(rm_serial::HeaderFrame));
    ptr += sizeof(rm_serial::HeaderFrame);
    memcpy(ptr, &packet_to_send.time_stamp, sizeof(packet_to_send.time_stamp));
    ptr += sizeof(packet_to_send.time_stamp);
    memcpy(ptr, &packet_to_send.data, sizeof(packet_to_send.data));

    packet_to_send.crc = crc16::get_CRC16_check_sum(crc16_payload_buffer.data(), crc16_payload_buffer.size(), 0);

    // 4. 序列化并发送
    std::vector<uint8_t> frame_bytes = rm_serial::toVector(packet_to_send);
    if (!send_frame(frame_bytes)) {
        RCLCPP_WARN(this->get_logger(), "TX: SendRobotCmdData 发送未完成.");
    } else {
        RCLCPP_DEBUG(this->get_logger(), "TX Sent: mode=%d, vx=%.2f, vy=%.2f, vw=%.2f | pitch=%.2f, yaw=%.2f, dist=%.2f, fire=%d",
            packet_to_send.data.move_mode_vector.move_mode, packet_to_send.data.speed_vector.vx, packet_to_send.data.speed_vector.vy, packet_to_send.data.speed_vector.vw,
            packet_to_send.data.gimbal_vector.pitch_diff, packet_to_send.data.gimbal_vector.yaw_diff, packet_to_send.data.move_vector.distance, packet_to_send.data.fire_vector.fire_advice);
    }
}

void SerialDriver::process_received_packet(const rm_serial::HeaderFrame& header, const std::vector<uint8_t>& data_segment)
{
    if (data_segment.size() < sizeof(uint32_t)) { // 数据段至少应包含时间戳
        RCLCPP_WARN(this->get_logger(), "RX: 数据段过短 (ID: 0x%02X, Len: %zu), 无法解析时间戳.", header.id, data_segment.size());
        return;
    }

    uint32_t time_stamp;
    memcpy(&time_stamp, data_segment.data(), sizeof(uint32_t));
    const uint8_t* actual_payload_ptr = data_segment.data() + sizeof(uint32_t);
    size_t actual_payload_len = data_segment.size() - sizeof(uint32_t);

    // RCLCPP_INFO(this->get_logger(), "RX: Processing ID: 0x%02X, Expected Payload Len: %zu, Actual Payload Len: %zu", header.id, header.len - sizeof(uint32_t), actual_payload_len);


    switch (header.id) {
        case rm_serial::ID_GAME_STATUS:
            if (actual_payload_len == sizeof(latest_game_status_.data)) {
                memcpy(&latest_game_status_.data, actual_payload_ptr, actual_payload_len);
                latest_game_status_.frame_header = header;
                latest_game_status_.time_stamp = time_stamp;
                game_status_valid_ = true;
                RCLCPP_DEBUG(this->get_logger(), "RX GameStatus: Progress %d, Time %d", latest_game_status_.data.game_progress, latest_game_status_.data.stage_remain_time);
            } else {
                RCLCPP_WARN(this->get_logger(), "RX: ID_GAME_STATUS 长度不匹配. Expected %zu, Got %zu", sizeof(latest_game_status_.data), actual_payload_len);
            }
            break;

        case rm_serial::ID_ALL_ROBOT_HP:
            if (actual_payload_len == sizeof(latest_all_robot_hp_.data)) {
                memcpy(&latest_all_robot_hp_.data, actual_payload_ptr, actual_payload_len);
                latest_all_robot_hp_.frame_header = header;
                latest_all_robot_hp_.time_stamp = time_stamp;
                all_robot_hp_valid_ = true;
                RCLCPP_DEBUG(this->get_logger(), "RX AllRobotHP: R7_HP %d, B7_HP %d", latest_all_robot_hp_.data.red_7_robot_hp, latest_all_robot_hp_.data.blue_7_robot_hp);
            } else {
                RCLCPP_WARN(this->get_logger(), "RX: ID_ALL_ROBOT_HP 长度不匹配. Expected %zu, Got %zu", sizeof(latest_all_robot_hp_.data), actual_payload_len);
            }
            break;

        case rm_serial::ID_ROBOT_GIMBAL_POSE:
            if (actual_payload_len == sizeof(latest_robot_gimbal_pose_.data)) {
                memcpy(&latest_robot_gimbal_pose_.data, actual_payload_ptr, actual_payload_len);
                latest_robot_gimbal_pose_.frame_header = header;
                latest_robot_gimbal_pose_.time_stamp = time_stamp;
                robot_gimbal_pose_valid_ = true;
                RCLCPP_DEBUG(this->get_logger(), "RX GimbalPose (ID %d): R%.2f P%.2f Y%.2f", latest_robot_gimbal_pose_.data.robot_id, latest_robot_gimbal_pose_.data.gimbal_roll, latest_robot_gimbal_pose_.data.gimbal_pitch, latest_robot_gimbal_pose_.data.gimbal_yaw);
            } else {
                RCLCPP_WARN(this->get_logger(), "RX: ID_ROBOT_GIMBAL_POSE 长度不匹配. Expected %zu, Got %zu", sizeof(latest_robot_gimbal_pose_.data), actual_payload_len);
            }
            break;

        case rm_serial::ID_ROBOT_CHASSIS_POSE:
            if (actual_payload_len == sizeof(latest_robot_chassis_pose_.data)) {
                memcpy(&latest_robot_chassis_pose_.data, actual_payload_ptr, actual_payload_len);
                latest_robot_chassis_pose_.frame_header = header;
                latest_robot_chassis_pose_.time_stamp = time_stamp;
                robot_chassis_pose_valid_ = true;
                RCLCPP_DEBUG(this->get_logger(), "RX ChassisPose (ID %d): R%.2f P%.2f Y%.2f", latest_robot_chassis_pose_.data.robot_id, latest_robot_chassis_pose_.data.chassis_roll, latest_robot_chassis_pose_.data.chassis_pitch, latest_robot_chassis_pose_.data.chassis_yaw);
            } else {
                RCLCPP_WARN(this->get_logger(), "RX: ID_ROBOT_CHASSIS_POSE 长度不匹配. Expected %zu, Got %zu", sizeof(latest_robot_chassis_pose_.data), actual_payload_len);
            }
            break;
        
        // case rm_serial::ID_EVENT_DATA: // 假设 ReceiveIfRebornData 使用了 EVENT_DATA ID
        //     // 注意：packet_typedef.hpp 中 ReceiveIfRebornData 没有指定 ID，这里假设它使用 ID_EVENT_DATA
        //     // 如果 ReceiveIfRebornData 有自己的独立 ID (例如 0x0D)，则需要修改 packet_typedef.hpp 中的常量定义
        //     // 并在这里添加一个新的 case。当前 packet_typedef.hpp 中没有 0x0D 的 ID 常量。
        //     // 为了演示，我们假设 ReceiveIfRebornData 使用 ID_EVENT_DATA
        //     if (header.id == rm_serial::ID_EVENT_DATA) { // 显式检查，因为 case 标签是 EVENT_DATA
        //         if (actual_payload_len == sizeof(latest_if_reborn_data_.data)) {
        //             memcpy(&latest_if_reborn_data_.data, actual_payload_ptr, actual_payload_len);
        //             latest_if_reborn_data_.frame_header = header;
        //             latest_if_reborn_data_.time_stamp = time_stamp;
        //             if_reborn_data_valid_ = true;
        //             RCLCPP_DEBUG(this->get_logger(), "RX IfRebornData: FreeReborn %d", latest_if_reborn_data_.data.is_free_reborn);
        //         } else {
        //             RCLCPP_WARN(this->get_logger(), "RX: ID_EVENT_DATA (as IfReborn) 长度不匹配. Expected %zu, Got %zu", sizeof(latest_if_reborn_data_.data), actual_payload_len);
        //         }
        //     }
        //     // 如果 ID_EVENT_DATA 是用于其他事件，你需要定义 ReceiveEventData 结构体并处理
        //     // else { RCLCPP_WARN(this->get_logger(), "RX: Unhandled specific event for ID_EVENT_DATA"); }
        //     break;
        // 修正：根据 packet_typedef.hpp，ReceiveIfRebornData 应该有自己的ID，但目前未定义。
        // 我们需要一个ID给 ReceiveIfRebornData，例如 ID_IF_REBORN = 0x0D (如注释中提到的)
        // 假设 packet_typedef.hpp 中添加了: const uint8_t ID_IF_REBORN = 0x0D;
        // case 0x0D: // 假设这是 ReceiveIfRebornData 的 ID
        //     if (actual_payload_len == sizeof(latest_if_reborn_data_.data)) {
        //         memcpy(&latest_if_reborn_data_.data, actual_payload_ptr, actual_payload_len);
        //         latest_if_reborn_data_.frame_header = header;
        //         latest_if_reborn_data_.time_stamp = time_stamp;
        //         if_reborn_data_valid_ = true;
        //         RCLCPP_DEBUG(this->get_logger(), "RX IfRebornData: FreeReborn %d", latest_if_reborn_data_.data.is_free_reborn);
        //     } else {
        //         RCLCPP_WARN(this->get_logger(), "RX: ID_IF_REBORN 长度不匹配. Expected %zu, Got %zu", sizeof(latest_if_reborn_data_.data), actual_payload_len);
        //     }
        //     break;
        // 再次修正：用户提供的 packet_typedef.hpp 中没有为 ReceiveIfRebornData 定义 ID。
        // 它在注释中提到了 0x020D，这看起来像裁判系统协议的 CmdID，而不是我们自定义的短 ID。
        // 为了能运行，我将暂时不处理 ReceiveIfRebornData，除非用户澄清其 ID。
        // 如果用户确实想用 ID_EVENT_DATA (0x03) 来传输是否复活，那么上面的 case ID_EVENT_DATA 需要调整。

        default:
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "RX: 未知或未处理的 Cmd ID: 0x%02X", header.id);
            break;
    }
}


void SerialDriver::timer_callback()
{
    static int heartbeat_counter = 0;
    if (++heartbeat_counter % 100 == 0) { // 大约每秒一次心跳日志
        RCLCPP_INFO(this->get_logger(), "串口驱动心跳 - 发送模式: %d", latest_send_cmd_.data.move_mode_vector.move_mode);
    }

    if (!ensure_serial_connection()) {
        // 如果连接失败，不执行后续的读写和发布，等待下一次重连尝试
        return;
    }
    check_timeouts(); // 检查命令超时

    auto current_time = this->now();

    // --- 1. 从串口读取数据并追加到 receive_buffer_ ---
    if (serial_fd_ >= 0) {
        uint8_t temp_buffer[512]; // 增加读取缓冲区大小
        ssize_t bytes_read = read(serial_fd_, temp_buffer, sizeof(temp_buffer));
        if (bytes_read > 0) {
            receive_buffer_.insert(receive_buffer_.end(), temp_buffer, temp_buffer + bytes_read);
        } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "RX: 读取数据失败: %s", strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) close_serial();
        }
    }

    // --- 2. 处理 receive_buffer_ 中的数据帧 ---
    bool processed_a_packet_this_cycle = true;
    while(processed_a_packet_this_cycle && !receive_buffer_.empty()) {
        processed_a_packet_this_cycle = false; // 假设本轮循环未处理

        auto sof_it = std::find(receive_buffer_.begin(), receive_buffer_.end(), rm_serial::SOF_RECEIVE);
        if (sof_it == receive_buffer_.end()) {
            if (receive_buffer_.size() > 2048) { // 防止缓冲区在全是无效数据时无限增长
                RCLCPP_WARN(this->get_logger(), "RX: 缓冲区过大 (%zu bytes) 且未找到SOF，清空缓冲区.", receive_buffer_.size());
                receive_buffer_.clear();
            }
            break; // 未找到SOF，等待更多数据
        }

        if (sof_it != receive_buffer_.begin()) {
            // RCLCPP_DEBUG(this->get_logger(), "RX: 丢弃SOF前的 %ld 字节.", std::distance(receive_buffer_.begin(), sof_it));
            receive_buffer_.erase(receive_buffer_.begin(), sof_it);
        }

        if (receive_buffer_.size() < sizeof(rm_serial::HeaderFrame)) {
            break; // 数据不足以构成帧头
        }

        rm_serial::HeaderFrame current_header;
        memcpy(&current_header, receive_buffer_.data(), sizeof(rm_serial::HeaderFrame));

        uint8_t header_crc_payload[3] = {current_header.sof, current_header.len, current_header.id};
        uint8_t calculated_header_crc8 = crc8::get_CRC8_check_sum(header_crc_payload, 3, 0);

        if (calculated_header_crc8 != current_header.crc) {
            RCLCPP_WARN(this->get_logger(), "RX: 帧头CRC8校验失败! SOF:0x%X, Len:%d, ID:0x%X, RecvCRC8:0x%X, CalcCRC8:0x%X. 丢弃SOF.",
                        current_header.sof, current_header.len, current_header.id, current_header.crc, calculated_header_crc8);
            receive_buffer_.erase(receive_buffer_.begin()); // 丢弃错误的SOF
            processed_a_packet_this_cycle = true; // 尝试寻找下一个SOF
            continue;
        }
        
        // 帧头CRC正确，检查数据包总长度
        // 数据段长度 (header.len) + 帧头大小 + CRC16大小 (2字节)
        size_t expected_total_packet_len = sizeof(rm_serial::HeaderFrame) + current_header.len + sizeof(uint16_t);
        if (receive_buffer_.size() < expected_total_packet_len) {
            break; // 数据不足以构成完整数据包
        }

        // 提取完整数据包字节 (帧头 + 数据段 + 接收到的CRC16)
        std::vector<uint8_t> full_packet_bytes(receive_buffer_.begin(), receive_buffer_.begin() + expected_total_packet_len);
        
        // 校验整个数据包的CRC16 (校验范围：帧头 + 数据段)
        uint16_t calculated_packet_crc16 = crc16::get_CRC16_check_sum(
            full_packet_bytes.data(),
            sizeof(rm_serial::HeaderFrame) + current_header.len, // CRC16校验的数据长度
            0);

        uint16_t received_packet_crc16;
        memcpy(&received_packet_crc16, full_packet_bytes.data() + sizeof(rm_serial::HeaderFrame) + current_header.len, sizeof(uint16_t));

        if (calculated_packet_crc16 != received_packet_crc16) {
            RCLCPP_WARN(this->get_logger(), "RX: 数据包CRC16校验失败! ID:0x%X, Len:%d. RecvCRC16:0x%04X, CalcCRC16:0x%04X. 丢弃数据包.",
                        current_header.id, current_header.len, received_packet_crc16, calculated_packet_crc16);
            receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + expected_total_packet_len);
            processed_a_packet_this_cycle = true;
            continue;
        }

        // CRC16校验通过! 处理数据包
        std::vector<uint8_t> data_segment(
            full_packet_bytes.begin() + sizeof(rm_serial::HeaderFrame),
            full_packet_bytes.begin() + sizeof(rm_serial::HeaderFrame) + current_header.len
        );
        process_received_packet(current_header, data_segment);

        receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + expected_total_packet_len);
        processed_a_packet_this_cycle = true; // 成功处理一个包，继续尝试处理缓冲区中的其他包
    }

    // --- 3. 根据接收到的数据更新状态并执行决策逻辑 ---
    bool mode_changed_by_logic = false;
    uint32_t previous_move_mode = latest_send_cmd_.data.move_mode_vector.move_mode;
    uint16_t current_blood = 0; // TODO: Parameterize or dynamically determine self robot ID for HP

    if (all_robot_hp_valid_) {
        // 根据 self_robot_id_for_hp_ 参数选择对应的血量
        if (self_robot_id_for_hp_ == "red_7") current_blood = latest_all_robot_hp_.data.red_7_robot_hp;
        else if (self_robot_id_for_hp_ == "blue_7") current_blood = latest_all_robot_hp_.data.blue_7_robot_hp;
        // Add more IDs if needed
        else {
            RCLCPP_WARN_ONCE(this->get_logger(), "未知的 self_robot_id_for_hp: %s. 无法获取自身血量.", self_robot_id_for_hp_.c_str());
        }
    }


    if (game_status_valid_ && all_robot_hp_valid_) {
        uint8_t game_progress = latest_game_status_.data.game_progress;

        bool free_rebirth_active = false;
        if (if_reborn_data_valid_) { // 检查是否收到过复活数据包
             free_rebirth_active = latest_if_reborn_data_.data.is_free_reborn == 1;
        }

        if (is_navigating_to_center_ && free_rebirth_active) {
            latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 自瞄模式
            is_navigating_to_center_ = false;
            mode_changed_by_logic = true;
            RCLCPP_INFO(this->get_logger(), "检测到免费复活，切换到自瞄模式(2)");
        }
        else if (current_blood <= 150 && current_blood > 0) { // 大于0避免刚复活就回家
            if (latest_send_cmd_.data.move_mode_vector.move_mode != 4) {
                latest_send_cmd_.data.move_mode_vector.move_mode = 4; // 导航模式
                mode_changed_by_logic = true;
            }
            if (!is_navigating_to_start_ || mode_changed_by_logic) { // 如果模式刚改变或未在导航到起点
                start_point_.header.stamp = current_time;
                send_nav_goal(start_point_);
                is_navigating_to_start_ = true;
                is_navigating_to_center_ = false;
                RCLCPP_INFO(this->get_logger(), "血量过低 (%d)，切换到导航模式(4)返回起点", current_blood);
            }
        }
        else if (game_progress == 4 && current_blood > 150) { // 比赛进行中且血量充足
            if (latest_send_cmd_.data.move_mode_vector.move_mode != 4 && latest_send_cmd_.data.move_mode_vector.move_mode != 2) {
                 latest_send_cmd_.data.move_mode_vector.move_mode = 4; // 导航模式
                 mode_changed_by_logic = true;
            }
             if (!is_navigating_to_center_ || mode_changed_by_logic) { // 如果模式刚改变或未在导航到中心
                center_point_.header.stamp = current_time;
                send_nav_goal(center_point_);
                is_navigating_to_center_ = true;
                is_navigating_to_start_ = false;
                RCLCPP_INFO(this->get_logger(), "状态良好(血量：%d)，切换到导航模式(4)前往中心点", current_blood);
            }
        }
        else { // 其他情况（例如比赛未开始、暂停，或血量非常高但不在前往中心点）
            if (latest_send_cmd_.data.move_mode_vector.move_mode != 2) { // 默认自瞄
                latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 自瞄模式
                mode_changed_by_logic = true;
            }
            // 如果不在导航，则清除导航状态标志
            if (latest_send_cmd_.data.move_mode_vector.move_mode == 2) {
                is_navigating_to_center_ = false;
                is_navigating_to_start_ = false;
            }
             RCLCPP_DEBUG(this->get_logger(), "其他情况，模式设置为 %d (状态=%d,血量=%d)", latest_send_cmd_.data.move_mode_vector.move_mode, game_progress, current_blood);
        }

        if (mode_changed_by_logic) {
            RCLCPP_INFO(this->get_logger(), "模式变更: %d -> %d (状态=%d, 血量=%d, 免费复活=%d)",
                        previous_move_mode, latest_send_cmd_.data.move_mode_vector.move_mode, game_progress, current_blood, free_rebirth_active);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "游戏状态或HP数据无效，无法执行决策逻辑。当前发送模式: %d", latest_send_cmd_.data.move_mode_vector.move_mode);
        // 保持当前模式或设置为安全模式
        if (latest_send_cmd_.data.move_mode_vector.move_mode != 2 && latest_send_cmd_.data.move_mode_vector.move_mode != 4) {
            latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 默认为自瞄
        }
    }

    // --- 4. 准备并发送命令数据 ---
    prepare_and_send_robot_cmd();

    // --- 5. 发布ROS消息和TF变换 ---
    rm_interfaces::msg::GimbalState gimbal_state_msg;
    if (robot_chassis_pose_valid_) { // 使用底盘姿态作为云台关节状态的基础（如果云台是相对底盘的）
        gimbal_state_msg.pitch = latest_robot_chassis_pose_.data.chassis_pitch; // 或者使用云台的pitch
        gimbal_state_msg.yaw = latest_robot_chassis_pose_.data.chassis_yaw;     // 或者使用云台的yaw
    } else {
        gimbal_state_msg.pitch = NAN; // 无有效数据
        gimbal_state_msg.yaw = NAN;
    }
    gimbal_state_pub_->publish(gimbal_state_msg);

    rm_interfaces::msg::SerialReceiveData serial_receive_msg;
    serial_receive_msg.header.stamp = current_time;
    serial_receive_msg.header.frame_id = gimbal_frame_id_; // 或 base_frame_id_
    // 从各个接收到的数据包中填充 serial_receive_msg
    if (robot_gimbal_pose_valid_) {
        serial_receive_msg.roll = latest_robot_gimbal_pose_.data.gimbal_roll;
        serial_receive_msg.pitch = latest_robot_gimbal_pose_.data.gimbal_pitch;
        serial_receive_msg.yaw = latest_robot_gimbal_pose_.data.gimbal_yaw;
        // serial_receive_msg.mode = latest_robot_gimbal_pose_.data.robot_id; // 示例：如果模式与机器人ID相关
    } else {
        serial_receive_msg.roll = NAN; serial_receive_msg.pitch = NAN; serial_receive_msg.yaw = NAN;
    }
    if (game_status_valid_) {
        serial_receive_msg.judge_system_data.game_status = latest_game_status_.data.game_progress;
        serial_receive_msg.judge_system_data.remaining_time = latest_game_status_.data.stage_remain_time;
    }
    if (all_robot_hp_valid_) {
        serial_receive_msg.judge_system_data.blood = current_blood; // 使用前面确定的自身血量
    }
    if (if_reborn_data_valid_) {
        // serial_receive_msg.judge_system_data.is_rfid_rebirth = latest_if_reborn_data_.data.is_free_reborn; // 假设有这个字段
    }
    gimbal_pub_->publish(serial_receive_msg);

    // TF变换
    geometry_msgs::msg::TransformStamped t_gimbal;
    t_gimbal.header.stamp = current_time;
    t_gimbal.header.frame_id = base_frame_id_; // 例如 "odom" 或 "base_link"
    t_gimbal.child_frame_id = gimbal_frame_id_; // 例如 "gimbal_link"
    tf2::Quaternion q_gimbal;
    if (robot_gimbal_pose_valid_) {
        // 假设云台姿态是相对于 base_frame_id_ 的绝对姿态
        // 或者如果它是相对于底盘的，你需要先获取底盘姿态，再组合
        q_gimbal.setRPY(latest_robot_gimbal_pose_.data.gimbal_roll,
                        -latest_robot_gimbal_pose_.data.gimbal_pitch, // 注意正负号约定
                        latest_robot_gimbal_pose_.data.gimbal_yaw);
    } else {
        q_gimbal.setRPY(0,0,0); // 默认姿态
    }
    t_gimbal.transform.rotation = tf2::toMsg(q_gimbal);
    t_gimbal.transform.translation.x = 0.0; // 根据实际情况设置云台相对于base_frame的平移
    t_gimbal.transform.translation.y = 0.0;
    t_gimbal.transform.translation.z = 0.1; // 示例：云台高0.1米
    tf_broadcaster_->sendTransform(t_gimbal);

    // 可选：底盘TF (如果 odom -> base_link 由其他节点发布，则不需要这个)
    // geometry_msgs::msg::TransformStamped t_chassis;
    // ... 填充 t_chassis ...
    // tf_broadcaster_->sendTransform(t_chassis);


    // 固定目标点发布 (调试用)
    if (this->get_parameter("use_fixed_goal").as_bool()) {
        center_point_.header.stamp = current_time;
        nav_goal_pub_->publish(center_point_);
    }

    // 在每个周期结束时，可以选择性地将某些数据的有效性标志重置为false，
    // 以确保如果下一周期没有收到更新，则不会使用旧数据。
    // 或者，在 check_timeouts 中为收到的数据也设置超时。
    // game_status_valid_ = false; // 比较激进的做法
}


void SerialDriver::gimbal_cmd_callback(std::shared_ptr<const rm_interfaces::msg::GimbalCmd> msg)
{
    latest_send_cmd_.data.gimbal_vector.pitch_diff = msg->pitch_diff;
    latest_send_cmd_.data.gimbal_vector.yaw_diff = msg->yaw_diff;
    latest_send_cmd_.data.move_vector.distance = msg->distance; // 假设这是云台控制相关的距离
    latest_send_cmd_.data.fire_vector.fire_advice = msg->fire_advice ? 1 : 0;

    last_gimbal_cmd_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "GimbalCmd RX: pitch_diff=%.2f, yaw_diff=%.2f, dist=%.2f, fire=%d",
               latest_send_cmd_.data.gimbal_vector.pitch_diff, latest_send_cmd_.data.gimbal_vector.yaw_diff,
               latest_send_cmd_.data.move_vector.distance, latest_send_cmd_.data.fire_vector.fire_advice);
}

void SerialDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_send_cmd_.data.speed_vector.vx = msg->linear.x;
    latest_send_cmd_.data.speed_vector.vy = msg->linear.y;
    latest_send_cmd_.data.speed_vector.vw = msg->angular.z;

    last_nav_cmd_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "CmdVel RX: vx=%.2f, vy=%.2f, vw=%.2f",
              msg->linear.x, msg->linear.y, msg->angular.z);
}

void SerialDriver::check_timeouts()
{
    auto current_time = this->now();
    // 导航命令超时
    if ((current_time - last_nav_cmd_time_).seconds() > cmd_timeout_) {
        if (latest_send_cmd_.data.speed_vector.vx != 0.0f ||
            latest_send_cmd_.data.speed_vector.vy != 0.0f ||
            latest_send_cmd_.data.speed_vector.vw != 0.0f) {
            RCLCPP_WARN(this->get_logger(), "导航命令超时，速度已重置为0");
            latest_send_cmd_.data.speed_vector.vx = 0.0f;
            latest_send_cmd_.data.speed_vector.vy = 0.0f;
            latest_send_cmd_.data.speed_vector.vw = 0.0f;
        }
    }

    // 自瞄命令超时
    if ((current_time - last_gimbal_cmd_time_).seconds() > cmd_timeout_) {
         if (latest_send_cmd_.data.gimbal_vector.pitch_diff != 0.0f ||
             latest_send_cmd_.data.gimbal_vector.yaw_diff != 0.0f ||
             latest_send_cmd_.data.fire_vector.fire_advice != 0) {
            RCLCPP_WARN(this->get_logger(), "自瞄命令超时，保留最后指令或重置 (当前为保留).");
            // 可选：重置自瞄指令
            // latest_send_cmd_.data.gimbal_vector.pitch_diff = 0.0f;
            // latest_send_cmd_.data.gimbal_vector.yaw_diff = 0.0f;
            // latest_send_cmd_.data.fire_vector.fire_advice = 0;
        }
    }

    // TODO: 为接收到的数据添加超时逻辑，例如如果一段时间未收到 GameStatus，则 game_status_valid_ = false;
}

void SerialDriver::send_nav_goal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
    if (!nav_client_ || !nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "导航Action服务不可用或等待超时!");
        // 导航失败时，可能需要重置 is_navigating_to_... 标志
        is_navigating_to_center_ = false;
        is_navigating_to_start_ = false;
        // 切换到安全模式，例如自瞄
        latest_send_cmd_.data.move_mode_vector.move_mode = 2;
        RCLCPP_INFO(this->get_logger(), "导航服务不可用，切换到自瞄模式(2)");
        return;
    }

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
        [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr /*goal_handle*/,
               const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
    {
        RCLCPP_DEBUG(this->get_logger(), "导航反馈：剩余距离 %.2f m", feedback->distance_remaining);
    };

    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        // 重置导航状态标志，无论成功与否
        bool was_navigating_to_center = is_navigating_to_center_;
        is_navigating_to_center_ = false;
        is_navigating_to_start_ = false;

        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "导航目标达成！");
                if (was_navigating_to_center) { // 如果之前是去中心点
                    latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 到达中心点后切换到自瞄模式
                    RCLCPP_INFO(this->get_logger(), "导航到中心点完成，切换到自瞄模式(2)");
                } else { // 如果是去起点或其他点
                    // latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 也可切换到自瞄
                    RCLCPP_INFO(this->get_logger(), "导航到目标点完成，当前模式: %d", latest_send_cmd_.data.move_mode_vector.move_mode);
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "导航目标被中止！");
                latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 导航失败，切换到自瞄
                RCLCPP_INFO(this->get_logger(), "导航中止，切换到自瞄模式(2)");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "导航目标被取消！");
                latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 导航失败，切换到自瞄
                RCLCPP_INFO(this->get_logger(), "导航取消，切换到自瞄模式(2)");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "导航目标失败，未知结果代码！");
                latest_send_cmd_.data.move_mode_vector.move_mode = 2; // 导航失败，切换到自瞄
                RCLCPP_INFO(this->get_logger(), "导航失败，切换到自瞄模式(2)");
                break;
        }
    };

    RCLCPP_INFO(this->get_logger(), "发送导航目标: (%.2f, %.2f)", goal_pose.pose.position.x, goal_pose.pose.position.y);
    nav_client_->async_send_goal(goal_msg, send_goal_options);
}

// main 函数通常在另一个文件中，或者如果这是节点主文件，则保持不变
// int main(int argc, char * argv[]) { ... }