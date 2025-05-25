#ifndef RM_SERIAL__ROS_INTERFACE_HPP_
#define RM_SERIAL__ROS_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rm_serial/packet_format.hpp"
#include "rm_interfaces/msg/gimbal_state.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/msg/judge_system_data.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <optional>

namespace rm_serial
{

class RosInterface
{
public:
    RosInterface(
        rclcpp::Node* node,
        const std::string& base_frame_id,
        const std::string& gimbal_frame_id,
        double default_bullet_speed);

    void publish_feedback_data(
        const rclcpp::Time& stamp,
        const PacketFormat::RxPacket* rx_packet_data); // Pointer, can be nullptr

private:
    // Publishers
    rclcpp::Publisher<rm_interfaces::msg::GimbalState>::SharedPtr gimbal_state_pub_;
    rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_data_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Member variables
    std::string base_frame_id_;
    std::string gimbal_frame_id_;
    double default_bullet_speed_;
    rclcpp::Logger logger_;
    rclcpp::Node* node_;  // Add node pointer for clock access

    // Private methods
    bool validate_packet_data(const PacketFormat::RxPacket* packet) const;
    
    void initialize_default_messages(
        const rclcpp::Time& stamp,
        rm_interfaces::msg::GimbalState& gimbal_msg,
        rm_interfaces::msg::SerialReceiveData& serial_msg,
        rm_interfaces::msg::JudgeSystemData& judge_msg) const;
    
    geometry_msgs::msg::TransformStamped create_safe_transform(
        const rclcpp::Time& stamp,
        const std::string& parent_frame,
        const std::string& child_frame,
        double roll, double pitch, double yaw) const;
};

} // namespace rm_serial

#endif // RM_SERIAL__ROS_INTERFACE_HPP_