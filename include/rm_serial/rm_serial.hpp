#ifndef RM_SERIAL_SERIAL_DRIVER_HPP_
#define RM_SERIAL_SERIAL_DRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>
#include <memory> // For std::shared_ptr
#include <array>  // For std::array (if still needed, though current cpp doesn't show direct use for gimbal_data_)

// TF2 and geometry messages
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Standard messages
#include <std_msgs/msg/int32.hpp> // For mode_command_callback if used

// Custom messages and types
#include "rm_serial/packet_typedef.hpp" // Your packet definitions
#include "rm_serial/crc8_crc16.hpp"     // Your CRC utilities (assuming it's just functions, not a class to include)
#include "rm_interfaces/msg/gimbal_state.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
// #include "rm_interfaces/msg/judge_system_data.hpp" // This seems to be part of SerialReceiveData now
// #include "rm_interfaces/msg/operator_command.hpp" // Not seen in cpp, remove if not used

// Navigation action
#include "nav2_msgs/action/navigate_to_pose.hpp"

class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SerialDriver();

private:
  // Initialization functions
  void init_params();
  void init_state_variables(); // Changed from init_state() to match cpp
  bool init_serial();
  void close_serial();

  // Communication and processing functions
  bool ensure_serial_connection();
  // ssize_t write_data(const uint8_t *data, size_t len); // Replaced by send_frame in cpp
  // ssize_t read_data(uint8_t *buffer, size_t len); // Logic moved to timer_callback with receive_buffer_
  void prepare_and_send_robot_cmd();
  void process_received_packet(const rm_serial::HeaderFrame& header, const std::vector<uint8_t>& data_segment);
  bool send_frame(const std::vector<uint8_t>& frame); // Added to match cpp

  // Callback functions
  void timer_callback();
  void gimbal_cmd_callback(const std::shared_ptr<const rm_interfaces::msg::GimbalCmd> msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  // void mode_command_callback(const std_msgs::msg::Int32::SharedPtr msg); // Keep if you plan to implement
  void send_nav_goal(const geometry_msgs::msg::PoseStamped & goal_pose);
  void check_timeouts();


  // Serial port related variables
  int serial_fd_;
  std::string serial_port_;
  // int baud_rate_; // Set directly in init_serial, not stored as member in cpp
  double cmd_timeout_;
  double timer_period_;
  int reconnect_attempts_;
  int max_reconnect_attempts_;
  double reconnect_delay_;
  double last_reconnect_time_; // double in cpp

  // State and data variables
  rm_serial::SendRobotCmdData latest_send_cmd_; // Stores the complete command to be sent
  
  // Received data storage
  rm_serial::ReceiveGameStatusData latest_game_status_;
  rm_serial::ReceiveAllRobotHpData latest_all_robot_hp_;
  rm_serial::ReceiveRobotGimbalPoseData latest_robot_gimbal_pose_;
  rm_serial::ReceiveRobotChassisPoseData latest_robot_chassis_pose_;
  rm_serial::ReceiveIfRebornData latest_if_reborn_data_;
  // rm_serial::ReceiveEventData latest_event_data_; // If you define and use this

  std::vector<uint8_t> receive_buffer_;

  // Data validity flags
  bool game_status_valid_;
  bool all_robot_hp_valid_;
  bool robot_gimbal_pose_valid_;
  bool robot_chassis_pose_valid_;
  bool if_reborn_data_valid_;
  // bool event_data_valid_;

  // Time tracking for commands
  rclcpp::Time last_gimbal_cmd_time_;
  rclcpp::Time last_nav_cmd_time_;
  
  // Navigation state
  bool is_navigating_to_center_;
  bool is_navigating_to_start_;
  geometry_msgs::msg::PoseStamped center_point_;
  geometry_msgs::msg::PoseStamped start_point_;
  std::string self_robot_id_for_hp_; // Parameter for identifying self HP

  // ROS Publishers and Subscribers
  rclcpp::Publisher<rm_interfaces::msg::GimbalState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_; // For publishing fixed goals
  
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_; // Keep if you plan to implement

  // ROS Action Client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  
  // ROS Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // TF Broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string base_frame_id_;
  std::string gimbal_frame_id_;
};

#endif  // RM_SERIAL_SERIAL_DRIVER_HPP_