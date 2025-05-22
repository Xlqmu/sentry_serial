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
    bool updated_is_nav_to_healing; // 添加回血点标识
    bool mode_was_changed;
};

class DecisionMaker
{
public:
    DecisionMaker(
        rclcpp::Node* node,
        const geometry_msgs::msg::PoseStamped& center_point_template,
        const geometry_msgs::msg::PoseStamped& start_point_template,
        const geometry_msgs::msg::PoseStamped& helper_point_template, // 添加辅助点参数
        int low_blood_threshold,
        int low_ammo_threshold);

    DecisionResult make_decision(
        const PacketFormat::RxPacket& rx_packet,
        uint32_t current_move_mode,
        bool current_is_nav_to_center,
        bool current_is_nav_to_start,
        bool current_is_nav_to_healing,
        const rclcpp::Time& current_ros_time);

private:
    rclcpp::Logger logger_;
    geometry_msgs::msg::PoseStamped center_point_template_;
    geometry_msgs::msg::PoseStamped start_point_template_;
    geometry_msgs::msg::PoseStamped helper_point_template_; // 添加辅助点模板
    int low_blood_threshold_;
    int low_ammo_threshold_;
};

} // namespace rm_serial

#endif // RM_SERIAL__DECISION_MAKER_HPP_