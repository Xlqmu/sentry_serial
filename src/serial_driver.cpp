#include "rm_serial/serial_driver.hpp"
#include <chrono>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// 数据包格式定义 - 直接使用结构体
struct PacketFormat {
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
    
    static const size_t RX_PACKET_SIZE = sizeof(RxPacket);
    static const size_t TX_PACKET_SIZE = sizeof(TxPacket);
};

SerialDriver::SerialDriver(const rclcpp::NodeOptions & options)
: Node("serial_driver", options), 
  serial_fd_(-1), 
  move_mode_(4), 
  nav_data_updated_(true),
  gimbal_data_updated_(false),
  reconnect_attempts_(0),
  last_reconnect_time_(0)
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
    gimbal_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/armor_solver/cmd_gimbal", qos,
        std::bind(&SerialDriver::gimbal_cmd_callback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/red_standard_robot1/cmd_vel", qos,
        std::bind(&SerialDriver::cmd_vel_callback, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "/decision/mode_control", 10,
        std::bind(&SerialDriver::mode_command_callback, this, std::placeholders::_1));

    // 创建定时器
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&SerialDriver::timer_callback, this));

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
    this->declare_parameter("serial_port", "/dev/ttyUART"); 
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("cmd_timeout", 10.0);  // 增大超时时间，避免速度被清零
    this->declare_parameter("timer_period", 0.01);
    this->declare_parameter("reconnect_delay", 5.0);
    this->declare_parameter("max_reconnect_attempts", 10);

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

bool SerialDriver::write_data(const std::vector<uint8_t>& data)
{
    if (serial_fd_ < 0) return false;
    
    ssize_t total_written = 0;
    while (total_written < static_cast<ssize_t>(data.size())) {
        ssize_t written = write(serial_fd_, 
                              data.data() + total_written, 
                              data.size() - total_written);
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
    
    if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "读取数据失败: %s", strerror(errno));
            
            if (errno == EIO || errno == ENODEV || errno == ENXIO) {
                close_serial();
            }
        }
        data.clear();
        return false;
    }
    
    if (bytes_read == 0) {
        data.clear();
        return false;
    }
    
    data.resize(bytes_read);
    return true;
}

void SerialDriver::mode_command_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    move_mode_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "收到模式控制命令: %d", move_mode_);
}

void SerialDriver::check_timeouts()
{
    auto current_time = this->now();
    
    // 输出超时检测的详细日志
    double time_since_last_cmd = (current_time - last_nav_cmd_time_).seconds();
    RCLCPP_DEBUG(this->get_logger(), "超时检查: 自上次命令已过%.2f秒(超时阈值%.2f秒), nav_updated=%d",
               time_since_last_cmd, cmd_timeout_, nav_data_updated_);
    
    // 检查导航指令超时 - 增加条件判断
    if (time_since_last_cmd > cmd_timeout_) {
        if (nav_data_updated_) {
            latest_twist_ = geometry_msgs::msg::Twist();
            nav_data_updated_ = false;
            RCLCPP_WARN(this->get_logger(), "导航命令超时，速度已重置为0");
        }
    }

    // 检查云台指令超时
    if ((current_time - last_gimbal_cmd_time_).seconds() > cmd_timeout_) {
        if (gimbal_data_updated_) {
            gimbal_data_.fill(0.0);
            gimbal_data_updated_ = false;
        }
    }
}

void SerialDriver::timer_callback()
{
    // 确保串口连接
    ensure_serial_connection();
    check_timeouts();
    
    auto current_time = this->now();
    
    // 准备发送数据包
    PacketFormat::TxPacket tx_packet{};
    
    // 填充发送数据
    tx_packet.pitch_diff = gimbal_data_[0];
    tx_packet.yaw_diff = gimbal_data_[1];
    tx_packet.distance = gimbal_data_[2];
    tx_packet.fire_advice = static_cast<uint32_t>(gimbal_data_[3]);
    tx_packet.vx = latest_twist_.linear.x;
    tx_packet.vy = latest_twist_.linear.y;
    tx_packet.vw = latest_twist_.angular.z;
    tx_packet.move_mode = move_mode_;
    
    // 输出调试信息
    RCLCPP_INFO(this->get_logger(), "发送速度: vx=%.2f, vy=%.2f, vw=%.2f, mode=%d", 
              tx_packet.vx, tx_packet.vy, tx_packet.vw, tx_packet.move_mode);

    // 发送数据
    if (serial_fd_ >= 0) {
        std::vector<uint8_t> tx_data(PacketFormat::TX_PACKET_SIZE);
        memcpy(tx_data.data(), &tx_packet, PacketFormat::TX_PACKET_SIZE);
        
        if (!write_data(tx_data)) {
            RCLCPP_ERROR(this->get_logger(), "TX: 发送失败");
        }
    }

    // 初始化发布消息
    rm_interfaces::msg::GimbalState gimbal_state;
    gimbal_state.pitch = 0.0;
    gimbal_state.yaw = 0.0;
    
    rm_interfaces::msg::SerialReceiveData serial_data;
    serial_data.header.stamp = current_time;
    serial_data.header.frame_id = "gimbal_link";
    serial_data.mode = 0;
    serial_data.bullet_speed = 15.0;
    serial_data.roll = 0.0;
    serial_data.yaw = 0.0;
    serial_data.pitch = 0.0;
    
    auto judge_data = rm_interfaces::msg::JudgeSystemData();
    judge_data.game_status = 0;
    judge_data.remaining_time = 0;
    judge_data.blood = 400;
    judge_data.is_free_rebirth = 0;
    
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

    // 接收数据
    std::vector<uint8_t> rx_data;
    bool rx_success = false;
    PacketFormat::RxPacket rx_packet{};

    if (serial_fd_ >= 0 && read_data(rx_data, PacketFormat::RX_PACKET_SIZE)) {
        if (rx_data.size() >= PacketFormat::RX_PACKET_SIZE) {
            memcpy(&rx_packet, rx_data.data(), PacketFormat::RX_PACKET_SIZE);
            rx_success = true;

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
            judge_data.is_free_rebirth = rx_packet.is_free_rebirth;
            
            serial_data.judge_system_data = judge_data;

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
            
            // 记录成功接收日志
            RCLCPP_INFO(this->get_logger(), 
                "RX: mode=%d, roll=%.2f, pitch_g=%.2f, yaw_g=%.2f, hp=%d",
                rx_packet.decator_mode, rx_packet.roll, 
                rx_packet.pitch_gimbal, rx_packet.yaw_gimbal, rx_packet.blood);
        }
    }

    // 发布消息
    gimbal_state_pub_->publish(gimbal_state);
    gimbal_pub_->publish(serial_data);
    tf_broadcaster_->sendTransform(t);
    tf_broadcaster_->sendTransform(rectify_t);
}

void SerialDriver::gimbal_cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= 4) {
        std::copy(msg->data.begin(), msg->data.begin() + 4, gimbal_data_.begin());
        last_gimbal_cmd_time_ = this->now();
        gimbal_data_updated_ = true;
    }
}

void SerialDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    latest_twist_ = *msg;
    last_nav_cmd_time_ = this->now();
    nav_data_updated_ = true;
    
    // 从速度命令中提取模式信息
    if (msg->linear.z > 0.5) {  // 确保有效值
        uint32_t new_mode = static_cast<uint32_t>(msg->linear.z + 0.5);  // 四舍五入转换为整数
        if (new_mode != move_mode_) {
            RCLCPP_INFO(this->get_logger(), "从速度命令中提取新模式: %d (原模式: %d)", new_mode, move_mode_);
            move_mode_ = new_mode;
        }
    }
    
    // 添加日志确认收到速度命令
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