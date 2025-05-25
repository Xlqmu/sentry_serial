#ifndef RM_SERIAL__SERIAL_DRIVER_HPP_ // Changed include guard
#define RM_SERIAL__SERIAL_DRIVER_HPP_ // Changed include guard

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
#include "rm_interfaces/srv/set_mode.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

// Forward declarations for composed classes
namespace rm_serial {
class SerialCommunicator;
class DecisionMaker;
class RosInterface;
}

class SerialDriver : public rclcpp::Node
{
public:
  explicit SerialDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~SerialDriver();

private:
  // 初始化函数
  void init_params();
  void init_state();
  // bool init_serial(); // Managed by SerialCommunicator
  // void close_serial(); // Managed by SerialCommunicator

  // 通信函数
  // bool ensure_serial_connection(); // Managed by SerialCommunicator
  // bool write_data(const std::vector<uint8_t>& data); // Managed by SerialCommunicator
  // bool read_data(std::vector<uint8_t>& data, size_t length); // Managed by SerialCommunicator
  // void handle_receive(); // Logic is within timer_callback
  void check_timeouts();
  
  // 回调函数
  void timer_callback();
  void gimbal_cmd_callback(const std::shared_ptr<const rm_interfaces::msg::GimbalCmd> msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void mode_command_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void send_nav_goal(const geometry_msgs::msg::PoseStamped & goal_pose);

  // Parameters (retrieved from YAML or defaults)
  std::string serial_port_;
  int baud_rate_;
  double cmd_timeout_;
  double timer_period_;
  double reconnect_delay_;
  int max_reconnect_attempts_;
  bool use_fixed_goal_;
  std::string base_frame_id_;
  std::string gimbal_frame_id_;
  double default_bullet_speed_; // Added for RosInterface

  // Decision Maker Parameters
  int low_blood_threshold_;
  int low_ammo_threshold_;
  std::string nav_points_map_frame_id_;


  // Internal State
  std::array<double, 4> gimbal_data_;  // pitch_diff, yaw_diff, distance, fire_advice
  geometry_msgs::msg::Twist latest_twist_;
  rclcpp::Time last_gimbal_cmd_time_;
  rclcpp::Time last_nav_cmd_time_;
  bool nav_data_updated_;
  bool gimbal_data_updated_;
  uint32_t move_mode_;
  
  geometry_msgs::msg::PoseStamped center_point_;
  geometry_msgs::msg::PoseStamped start_point_;
  geometry_msgs::msg::PoseStamped helper_point_;  // 添加辅助点
  bool is_navigating_to_center_;
  bool is_navigating_to_start_;
  bool is_navigating_to_healing_;  // 添加这个变量声明
  
  // ROS Communications (Subscribers, Action Client, specific Publishers not in RosInterface)
  rclcpp::Subscription<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_; // Assuming you might add this
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_pub_; // For fixed goal debugging


  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Composed Objects
  std::unique_ptr<rm_serial::SerialCommunicator> communicator_;
  std::unique_ptr<rm_serial::DecisionMaker> decision_maker_;
  std::unique_ptr<rm_serial::RosInterface> ros_interface_;



  // 添加模式服务客户端相关的成员变量
  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> mode_clients_;
  uint8_t last_aim_color_;
  
  // 添加模式服务客户端相关的方法
  void init_mode_service_clients();
  void check_aim_color_change(uint8_t current_aim_color);
  void call_set_mode_service(uint8_t mode);
  void handle_service_response(
      rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture future,
      const std::string& service_name);
};

#endif  // RM_SERIAL__SERIAL_DRIVER_HPP_ // Changed include guard