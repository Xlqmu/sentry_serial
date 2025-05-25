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
        "/serial/gimbal_joint_state", rclcpp::QoS(10));
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
    // Prepare gimbal state message
    rm_interfaces::msg::GimbalState gimbal_state_msg;
    
    // Prepare serial receive data message
    rm_interfaces::msg::SerialReceiveData serial_data_msg;
    serial_data_msg.header.stamp = stamp;
    
    // Prepare judge system data
    rm_interfaces::msg::JudgeSystemData judge_data_msg;
    
    // Prepare TF transforms
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = base_frame_id_;
    t.child_frame_id = gimbal_frame_id_;
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    geometry_msgs::msg::TransformStamped rectify_t;
    rectify_t.header.stamp = stamp;
    rectify_t.header.frame_id = base_frame_id_;
    rectify_t.child_frame_id = base_frame_id_ + "_rectify";
    rectify_t.transform.translation.x = 0.0;
    rectify_t.transform.translation.y = 0.0;
    rectify_t.transform.translation.z = 0.0;

    // 优先使用实际数据，如果没有则使用默认值
    if (rx_packet_data) {
        RCLCPP_INFO(logger_,
            "Using actual serial data - pitch_gimbal: %.6f, yaw_gimbal: %.6f, roll: %.6f, "
            "game_status: %d, blood: %d, aim_color: %d",
            rx_packet_data->pitch_gimbal, rx_packet_data->yaw_gimbal, rx_packet_data->roll,
            rx_packet_data->game_status, rx_packet_data->blood, rx_packet_data->aim_color);

        // 使用实际的云台数据
        gimbal_state_msg.pitch = -rx_packet_data->pitch_gimbal; // Negate for correct direction
        gimbal_state_msg.yaw = rx_packet_data->yaw_gimbal;

        // 使用实际的串口数据
        serial_data_msg.mode = rx_packet_data->aim_color;
        serial_data_msg.roll = rx_packet_data->roll;
        serial_data_msg.pitch = rx_packet_data->pitch_gimbal;
        serial_data_msg.yaw = rx_packet_data->yaw_gimbal;


        // 使用实际的裁判系统数据
        judge_data_msg.game_status = rx_packet_data->game_status;
        judge_data_msg.remaining_time = rx_packet_data->remaining_time;
        judge_data_msg.blood = rx_packet_data->blood;
        judge_data_msg.outpost_hp = rx_packet_data->outpost_hp;


        // 使用实际数据计算TF变换
        tf2::Quaternion q_gimbal;
        q_gimbal.setRPY(rx_packet_data->roll, -rx_packet_data->pitch_gimbal, rx_packet_data->yaw_gimbal);
        t.transform.rotation = tf2::toMsg(q_gimbal);

        tf2::Quaternion q_rectify;
        q_rectify.setRPY(rx_packet_data->roll, 0.0, 0.0); // Rectify only roll
        rectify_t.transform.rotation = tf2::toMsg(q_rectify);
        
        RCLCPP_INFO(logger_, 
            "Processed gimbal values - pitch: %.6f, yaw: %.6f", 
            gimbal_state_msg.pitch, gimbal_state_msg.yaw);
    } 
    else {
        // 只有在没有实际数据时才使用默认值
        // 修复时钟API调用
        static auto clock = rclcpp::Clock();
        RCLCPP_WARN_THROTTLE(logger_, clock, 5000, 
            "No serial data available, using default values");
            
        // 设置默认的云台数据
        gimbal_state_msg.pitch = 0.0;
        gimbal_state_msg.yaw = 0.0;

        // 设置默认的串口数据
        serial_data_msg.mode = 0;
        serial_data_msg.roll = 0.0;
        serial_data_msg.pitch = 0.0; // 云台俯仰角
        serial_data_msg.yaw = 0.0;   // 云台偏航角

        // 设置默认的裁判系统数据
        judge_data_msg.game_status = 0;        // 游戏未开始
        judge_data_msg.remaining_time = 0; // 无剩余时间
        judge_data_msg.blood = 400;            // 满血
        judge_data_msg.outpost_hp = 1000;      // 前哨站满血
        judge_data_msg.remaining_time = 0;     // 无剩余时间

        // 设置默认的TF变换（单位四元数）
        t.transform.rotation.w = 1.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        
        rectify_t.transform.rotation.w = 1.0;
        rectify_t.transform.rotation.x = 0.0;
        rectify_t.transform.rotation.y = 0.0;
        rectify_t.transform.rotation.z = 0.0;
    }

    // 将裁判系统数据赋值给串口数据消息
    serial_data_msg.judge_system_data = judge_data_msg;

    RCLCPP_DEBUG(logger_, 
        "Publishing gimbal state - pitch: %.6f, yaw: %.6f", 
        gimbal_state_msg.pitch, gimbal_state_msg.yaw);

    // 发布所有消息
    gimbal_state_pub_->publish(gimbal_state_msg);
    serial_data_pub_->publish(serial_data_msg);
    tf_broadcaster_->sendTransform(t);
    tf_broadcaster_->sendTransform(rectify_t);
}

} // namespace rm_serial