#include "rm_serial/ros_interface.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <bitset>

namespace rm_serial
{

RosInterface::RosInterface(
    rclcpp::Node* node,
    const std::string& base_frame_id,
    const std::string& gimbal_frame_id,
    double default_bullet_speed)
: base_frame_id_(base_frame_id),
  gimbal_frame_id_(gimbal_frame_id),
  default_bullet_speed_(default_bullet_speed),
  logger_(node->get_logger().get_child("ros_interface"))
{
    // Create publishers
    gimbal_state_pub_ = node->create_publisher<rm_interfaces::msg::GimbalState>(
        "/gimbal/state", rclcpp::QoS(10));
    serial_data_pub_ = node->create_publisher<rm_interfaces::msg::SerialReceiveData>(
        "/serial/receive", rclcpp::QoS(10));
    
    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    RCLCPP_INFO(logger_, "RosInterface initialized");
}

void RosInterface::publish_feedback_data(
    const rclcpp::Time& stamp,
    const PacketFormat::RxPacket* rx_packet_data)
{
    // Prepare gimbal state message with defaults
    rm_interfaces::msg::GimbalState gimbal_state_msg;
    // gimbal_state_msg.header.stamp = stamp;
    // gimbal_state_msg.header.frame_id = gimbal_frame_id_;
    gimbal_state_msg.pitch = 0.0; // Default
    gimbal_state_msg.yaw = 0.0; // Default
    // gimbal_state_msg.bullet_speed = default_bullet_speed_;

    // Prepare serial receive data message with defaults
    rm_interfaces::msg::SerialReceiveData serial_data_msg;
    serial_data_msg.header.stamp = stamp;
    serial_data_msg.mode = 0; // Default

    // Judge system data with defaults
    rm_interfaces::msg::JudgeSystemData judge_data_msg;
    judge_data_msg.game_status = 0;
    judge_data_msg.remaining_time = 0;
    judge_data_msg.blood = 400; // Default
    judge_data_msg.outpost_hp = 2000; // Default
    serial_data_msg.judge_system_data = judge_data_msg;

    // Prepare TF transforms with defaults
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = base_frame_id_;
    t.child_frame_id = gimbal_frame_id_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.w = 1.0; // Identity rotation

    geometry_msgs::msg::TransformStamped rectify_t;
    rectify_t.header.stamp = stamp;
    rectify_t.header.frame_id = base_frame_id_;
    rectify_t.child_frame_id = base_frame_id_ + "_rectify";
    rectify_t.transform.translation.x = 0.0;
    rectify_t.transform.translation.y = 0.0;
    rectify_t.transform.translation.z = 0.0;
    rectify_t.transform.rotation.w = 1.0; // Identity rotation

    // Fill with actual data if rx_packet_data is provided
    if (rx_packet_data) {
        // Gimbal state
        gimbal_state_msg.pitch = -rx_packet_data->pitch_gimbal; // Negate for correct direction
        gimbal_state_msg.yaw = rx_packet_data->yaw_gimbal;

        // Serial receive data
        serial_data_msg.mode = rx_packet_data->aim_color;

        // Judge system data
        judge_data_msg.game_status = rx_packet_data->game_status;
        judge_data_msg.blood = rx_packet_data->blood;
        judge_data_msg.outpost_hp = rx_packet_data->outpost_hp;
        judge_data_msg.remaining_time = rx_packet_data->remaining_time;
        serial_data_msg.judge_system_data = judge_data_msg;

        // TF transforms
        tf2::Quaternion q_gimbal;
        q_gimbal.setRPY(rx_packet_data->roll, -rx_packet_data->pitch_gimbal, rx_packet_data->yaw_gimbal);
        t.transform.rotation = tf2::toMsg(q_gimbal);

        tf2::Quaternion q_rectify;
        q_rectify.setRPY(rx_packet_data->roll, 0.0, 0.0); // Rectify only roll
        rectify_t.transform.rotation = tf2::toMsg(q_rectify);
    }

    // Publish
    gimbal_state_pub_->publish(gimbal_state_msg);
    serial_data_pub_->publish(serial_data_msg);
    tf_broadcaster_->sendTransform(t);
    tf_broadcaster_->sendTransform(rectify_t);
}

} // namespace rm_serial