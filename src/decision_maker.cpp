#include "rm_serial/decision_maker.hpp"

namespace rm_serial
{

DecisionMaker::DecisionMaker(
    rclcpp::Node* node,
    const geometry_msgs::msg::PoseStamped& center_point_template,
    const geometry_msgs::msg::PoseStamped& start_point_template,
    const geometry_msgs::msg::PoseStamped& helper_point_template, // 添加辅助点参数
    int low_blood_threshold,
    int low_ammo_threshold)
: logger_(node->get_logger().get_child("decision_maker")),
  center_point_template_(center_point_template),
  start_point_template_(start_point_template),
  helper_point_template_(helper_point_template), // 初始化辅助点模板
  low_blood_threshold_(low_blood_threshold),
  low_ammo_threshold_(low_ammo_threshold)
{
}

DecisionResult DecisionMaker::make_decision(
    const PacketFormat::RxPacket& rx_packet,
    uint32_t current_move_mode,
    bool current_is_nav_to_center,
    bool current_is_nav_to_start,
    bool current_is_nav_to_healing,
    const rclcpp::Time& current_ros_time)
{
    DecisionResult result;
    result.new_move_mode = current_move_mode;
    result.send_goal = false;
    result.updated_is_nav_to_center = current_is_nav_to_center;
    result.updated_is_nav_to_start = current_is_nav_to_start;
    result.updated_is_nav_to_healing = current_is_nav_to_healing;
    result.mode_was_changed = false;

    uint8_t game_status = rx_packet.game_status & 0x0F;
    uint16_t blood = rx_packet.blood;
    uint16_t projectile_allowance_17mm = rx_packet.projectile_allowance_17mm;

    RCLCPP_DEBUG(logger_, "Making decision: game_status=%d, blood=%d, ammo=%d, rfid=%d, current_mode=%u, nav_center=%d, nav_start=%d, nav_healing=%d. Thresholds: blood=%d, ammo=%d",
                game_status, blood, projectile_allowance_17mm, rx_packet.is_rfid, current_move_mode, 
                current_is_nav_to_center, current_is_nav_to_start, current_is_nav_to_healing,
                low_blood_threshold_, low_ammo_threshold_);

    // 首先检查比赛状态 - 只有在比赛进行中(game_status == 4)才执行导航
    if (game_status == 4) {
        // 比赛进行中的逻辑
        
        // 修复问题一：在回血点回满血后应导航到中心点
        // 检测到RFID（位于回血点）且血量和弹药都充足，导航到中心点
        if (rx_packet.is_rfid && blood > low_blood_threshold_ && projectile_allowance_17mm > low_ammo_threshold_) {
            result.new_move_mode = 5; // 导航模式
            result.mode_was_changed = (current_move_mode != result.new_move_mode);
            
            // 设置导航目标为中心点
            result.goal_to_send = center_point_template_;
            result.goal_to_send.header.stamp = current_ros_time;
            result.send_goal = true;
            result.updated_is_nav_to_center = true;
            result.updated_is_nav_to_start = false;
            result.updated_is_nav_to_healing = false;
            
            RCLCPP_INFO(logger_, "At healing point (RFID=1) with good health/ammo. Navigating to center point.");
        }
        // 检测到RFID但血量或弹药不足，保持在原地自瞄模式
        else if (rx_packet.is_rfid) {
            result.new_move_mode = 2; // 自瞄模式
            result.updated_is_nav_to_healing = false; // 停止任何导航
            result.updated_is_nav_to_center = false;
            result.updated_is_nav_to_start = false;
            result.mode_was_changed = (current_move_mode != result.new_move_mode);
            RCLCPP_INFO(logger_, "At healing point (RFID=1) but health/ammo still low. Staying in aim mode (2).");
        }
        // 优先级1: 血量低或没子弹 => 立即前往回血点(无论当前位置或状态)
        else if (blood <= low_blood_threshold_ || projectile_allowance_17mm <= low_ammo_threshold_) {
            // 切换到导航模式
            result.new_move_mode = 5;
            result.mode_was_changed = (current_move_mode != result.new_move_mode);
            
            // 设置导航目标为回血点 (使用辅助点作为回血点)
            result.goal_to_send = helper_point_template_;
            result.goal_to_send.header.stamp = current_ros_time;
            result.send_goal = true;
            result.updated_is_nav_to_healing = true;
            result.updated_is_nav_to_center = false;
            result.updated_is_nav_to_start = false;
            
            RCLCPP_INFO(logger_, "Low health/ammo (blood: %d <= %d, ammo: %d <= %d). Switching to nav mode (5) to go to healing point (helper_point).",
                        blood, low_blood_threshold_, projectile_allowance_17mm, low_ammo_threshold_);
        }
        // 修复问题二：比赛阶段为4且血量弹量充足时应该执行去中心点的导航任务
        // 优先级2: 血量、子弹充足 => 前往中心点
        else if (blood > low_blood_threshold_ && projectile_allowance_17mm > low_ammo_threshold_) {
            // 如果当前不是导航模式或已在中心点但模式不是自瞄模式，切换到适当模式
            if (current_is_nav_to_center) {
                // 修复问题三：到达中心点后应该切换到自瞄模式
                result.new_move_mode = 2; // 自瞄模式
                result.mode_was_changed = (current_move_mode != result.new_move_mode);
                RCLCPP_INFO(logger_, "Already navigating to center point. Switching to aim mode (2).");
            } 
            else {
                // 不在中心点，启动导航
                result.new_move_mode = 5; // 导航模式
                result.mode_was_changed = (current_move_mode != result.new_move_mode);
                
                // 设置导航目标为中心点
                result.goal_to_send = center_point_template_;
                result.goal_to_send.header.stamp = current_ros_time;
                result.send_goal = true;
                result.updated_is_nav_to_center = true;
                result.updated_is_nav_to_start = false;
                result.updated_is_nav_to_healing = false;
                
                RCLCPP_INFO(logger_, "Game active, good health (blood: %d > %d, ammo: %d > %d). Starting navigation to center point.", 
                            blood, low_blood_threshold_, projectile_allowance_17mm, low_ammo_threshold_);
            }
        }
        // 其他比赛中状态：使用自瞄模式
        else {
            if (current_move_mode != 2) {
                result.new_move_mode = 2; // 自瞄模式
                result.mode_was_changed = true;
                RCLCPP_INFO(logger_, "Game active but conditions not met. Switching to aim mode (2).");
            }
            result.updated_is_nav_to_center = false;
            result.updated_is_nav_to_start = false;
            result.updated_is_nav_to_healing = false;
        }
    } 
    // 比赛未开始或已结束：不允许移动，使用自瞄模式但不执行导航
    else {
        // 非比赛状态下不能移动，切换到自瞄模式并停止任何导航任务
        if (current_move_mode != 2) {
            result.new_move_mode = 2; // 自瞄模式
            result.mode_was_changed = true;
            RCLCPP_INFO(logger_, "Game not active (status: %d). Switching to aim mode (2) and stopping navigation.", game_status);
        }
        
        // 确保不再执行导航任务
        if (current_is_nav_to_center || current_is_nav_to_start || current_is_nav_to_healing) {
            result.send_goal = false; // 不发送任何导航目标
            result.updated_is_nav_to_center = false;
            result.updated_is_nav_to_start = false;
            result.updated_is_nav_to_healing = false;
            RCLCPP_INFO(logger_, "Game not active. Cancelling any active navigation.");
        }
    }
    
    if (result.mode_was_changed) {
        RCLCPP_INFO(logger_, "Mode changed: %u -> %u. Game status: %d, blood: %d",
                    current_move_mode, result.new_move_mode, game_status, blood);
    }

    return result;
}

} // namespace rm_serial