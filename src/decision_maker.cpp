#include "rm_serial/decision_maker.hpp"

namespace rm_serial
{

DecisionMaker::DecisionMaker(
    rclcpp::Node* node,
    const geometry_msgs::msg::PoseStamped& center_point_template,
    const geometry_msgs::msg::PoseStamped& start_point_template,
    int low_blood_threshold,
    int low_ammo_threshold)
: logger_(node->get_logger().get_child("decision_maker")),
  center_point_template_(center_point_template),
  start_point_template_(start_point_template),
  low_blood_threshold_(low_blood_threshold),
  low_ammo_threshold_(low_ammo_threshold)
{
}

DecisionResult DecisionMaker::make_decision(
    const PacketFormat::RxPacket& rx_packet,
    uint32_t current_move_mode,
    bool current_is_nav_to_center,
    bool current_is_nav_to_start,
    const rclcpp::Time& current_ros_time)
{
    DecisionResult result;
    result.new_move_mode = current_move_mode;
    result.send_goal = false;
    result.updated_is_nav_to_center = current_is_nav_to_center;
    result.updated_is_nav_to_start = current_is_nav_to_start;
    result.mode_was_changed = false;

    uint8_t game_status = rx_packet.game_status & 0x0F;
    uint16_t blood = rx_packet.blood;
    // uint16_t outpost_hp = rx_packet.outpost_hp; // Not used in current decision logic but available
    uint16_t projectile_allowance_17mm = rx_packet.projectile_allowance_17mm;

    RCLCPP_DEBUG(logger_, "Making decision: game_status=%d, blood=%d, ammo=%d, rfid=%d, current_mode=%u, nav_center=%d, nav_start=%d. Thresholds: blood=%d, ammo=%d",
                game_status, blood, projectile_allowance_17mm, rx_packet.is_rfid, current_move_mode, current_is_nav_to_center, current_is_nav_to_start,
                low_blood_threshold_, low_ammo_threshold_);

    if (current_is_nav_to_center && rx_packet.is_rfid) {
        result.new_move_mode = 2; //自瞄模式
        result.updated_is_nav_to_center = false;
        result.mode_was_changed = (current_move_mode != result.new_move_mode);
        RCLCPP_INFO(logger_, "RFID detected while navigating to center. Switching to aim mode (2).");
    }
    // 优先级1: 血量低或没子弹 => 立即返回起点(无论当前位置或状态)
    else if (blood <= low_blood_threshold_ || projectile_allowance_17mm <= low_ammo_threshold_) {
        if (current_move_mode != 4) { // 导航模式
            result.new_move_mode = 4;
            result.mode_was_changed = true;
            RCLCPP_INFO(logger_, "Low health/ammo (blood: %d <= %d, ammo: %d <= %d). Switching to nav mode (4) to return to start.",
                        blood, low_blood_threshold_, projectile_allowance_17mm, low_ammo_threshold_);
        }
        result.goal_to_send = start_point_template_;
        result.goal_to_send.header.stamp = current_ros_time;
        result.send_goal = true;
        result.updated_is_nav_to_start = true;
        result.updated_is_nav_to_center = false;
        RCLCPP_INFO(logger_, "Sending nav goal: Return to start point (Low health/ammo).");
    }
    // 优先级2: 比赛进行中且血量充足 => 前往中心点 (如果不在自瞄模式)
    else if (game_status == 4 && blood > low_blood_threshold_) { // Assuming "good health" means above low_blood_threshold
        if (current_move_mode != 4 && current_move_mode != 2) { // 如果当前不是导航模式也不是自瞄模式
            result.new_move_mode = 4; // 切换到导航模式
            result.mode_was_changed = true;
            RCLCPP_INFO(logger_, "Game active, good health (blood: %d > %d). Switching to nav mode (4) to go to center.", blood, low_blood_threshold_);
        }
         // If already in mode 2 (aiming) but not navigating to center, and game is on, consider going to center.
        // However, the original logic implies if it's not mode 4 or 2, it becomes 4.
        // If it's mode 2, it stays mode 2 unless RFID is hit or health is low.
        // This part of the logic might need refinement if mode 2 should actively decide to navigate.
        // For now, if it's not mode 2, it will try to go to center.
        if (result.new_move_mode == 4 && !result.updated_is_nav_to_center) { // Only send goal if switching to nav or not already going to center
            result.goal_to_send = center_point_template_;
            result.goal_to_send.header.stamp = current_ros_time;
            result.send_goal = true;
            result.updated_is_nav_to_center = true;
            result.updated_is_nav_to_start = false;
            RCLCPP_INFO(logger_, "Sending nav goal: Go to center point.");
        } else if (current_move_mode == 2) { // Already in aiming mode, stay unless other conditions met
             result.updated_is_nav_to_center = false; // Not actively navigating if in aim mode
             result.updated_is_nav_to_start = false;
        }
    }
    // 优先级3: 其他情况 (例如比赛未开始，或状态异常但血量尚可) => 切换至自瞄模式 (或保持导航模式如果已是)
    else {
        if (current_move_mode != 2) { // 如果不是自瞄模式
            // The original logic was: move_mode_ = 4; then changed to 2 in some cases.
            // This implies a preference for navigation if not explicitly aiming.
            // Let's assume it should go to aiming mode if conditions for nav to start/center are not met.
            result.new_move_mode = 2; // 自瞄模式
            result.mode_was_changed = true;
            RCLCPP_INFO(logger_, "Defaulting to aim mode (2). Game status: %d, blood: %d", game_status, blood);
        }
        result.updated_is_nav_to_center = false;
        result.updated_is_nav_to_start = false;
    }
    
    if (result.mode_was_changed) {
         RCLCPP_INFO(logger_, "Mode changed: %u -> %u. Game status: %d, blood: %d",
                    current_move_mode, result.new_move_mode, game_status, blood);
    }

    return result;
}

} // namespace rm_serial
