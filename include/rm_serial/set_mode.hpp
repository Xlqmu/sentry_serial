#ifndef RM_SERIAL__MODE_SERVICE_CLIENT_HPP_
#define RM_SERIAL__MODE_SERVICE_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"

namespace rm_serial
{

class ModeServiceClient : public rclcpp::Node
{
public:
    ModeServiceClient();

private:
    void serial_data_callback(const rm_interfaces::msg::SerialReceiveData::SharedPtr msg);
    void call_set_mode_service(uint8_t mode);
    void handle_service_response(
        rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture future,
        const std::string& service_name);

    rclcpp::Subscription<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_data_sub_;
    std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> mode_clients_;
    uint8_t last_mode_;
};

} // namespace rm_serial

#endif // RM_SERIAL__MODE_SERVICE_CLIENT_HPP_