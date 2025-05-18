#ifndef RM_SERIAL__DECISION_MAKER_HPP_
#define RM_SERIAL__DECISION_MAKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rm_serial/packet_format.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace rm_serial
{

struct DecisionResult
{
    uint32_t new_move_mode;
    bool send_goal;
    geometry_msgs::msg::PoseStamped goal_to_send; // Valid only if send_goal is true
    bool updated_is_nav_to_center;
    bool updated_is_nav_to_start;
    bool mode_was_changed;
};

class DecisionMaker
{
public:
    DecisionMaker(
        rclcpp::Node* node,
        const geometry_msgs::msg::PoseStamped& center_point_template,
        const geometry_msgs::msg::PoseStamped& start_point_template,
        int low_blood_threshold,    // New parameter
        int low_ammo_threshold);    // New parameter

    DecisionResult make_decision(
        const PacketFormat::RxPacket& rx_packet,
        uint32_t current_move_mode,
        bool current_is_nav_to_center,
        bool current_is_nav_to_start,
        const rclcpp::Time& current_ros_time);

private:
    rclcpp::Logger logger_;
    geometry_msgs::msg::PoseStamped center_point_template_;
    geometry_msgs::msg::PoseStamped start_point_template_;
    int low_blood_threshold_;   // New member
    int low_ammo_threshold_;    // New member
};

} // namespace rm_serial

#endif // RM_SERIAL__DECISION_MAKER_HPP_
