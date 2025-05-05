#ifndef rm_interfaces__SERIAL_DRIVER_HPP_
#define rm_interfaces__SERIAL_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <array>
#include <vector>
#include <memory>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rm_interfaces/msg/gimbal_state.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/judge_system_data.hpp"
#include "rm_interfaces/msg/operator_command.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SerialDriver();

  uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
  }

private:
  // 初始化函数
  void init_params();
  void init_state();
  bool init_serial();
  void close_serial();

  // 通信函数
  bool ensure_serial_connection();
  bool write_data(const std::vector<uint8_t>& data);
  bool read_data(std::vector<uint8_t>& data, size_t length);
  void handle_receive();
  void check_timeouts();
  
  // 回调函数
  void timer_callback();
// 在 serial_driver.hpp 中更新这一行
  void gimbal_cmd_callback(const std::shared_ptr<const rm_interfaces::msg::GimbalCmd> msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void mode_command_callback(const std_msgs::msg::Int32::SharedPtr msg);  // 修改为与实现一致
  void send_nav_goal(const geometry_msgs::msg::PoseStamped & goal_pose);

  // 串口相关变量
  int serial_fd_;
  std::string serial_port_;
  int baud_rate_;
  double cmd_timeout_;
  double timer_period_;
  int reconnect_attempts_;
  int max_reconnect_attempts_;
  double reconnect_delay_;
  double last_reconnect_time_;
  
  // 数据缓存
  std::array<double, 4> gimbal_data_;  // pitch_diff, yaw_diff, distance, fire_advice
  geometry_msgs::msg::Twist latest_twist_;
  rclcpp::Time last_gimbal_cmd_time_;
  rclcpp::Time last_nav_cmd_time_;
  bool nav_data_updated_;
  bool gimbal_data_updated_;
  uint32_t move_mode_;  // 导航模式控制
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_;
  geometry_msgs::msg::PoseStamped center_point_;
  geometry_msgs::msg::PoseStamped start_point_;
  bool is_navigating_to_center_;
  bool is_navigating_to_start_;
  
  // 发布器和订阅器
  rclcpp::Publisher<rm_interfaces::msg::GimbalState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr gimbal_pub_;
  
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  
  // TF变换
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string base_frame_id_;
  std::string gimbal_frame_id_;
};

#endif  // rm_interfaces__SERIAL_DRIVER_HPP_