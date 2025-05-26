#ifndef RM_SERIAL__SERIAL_COMMUNICATOR_HPP_
#define RM_SERIAL__SERIAL_COMMUNICATOR_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"
#include "rm_serial/packet_format.hpp"
#include "rm_serial/crc8_crc16.hpp" // For checksum calculations

namespace rm_serial
{

class SerialCommunicator
{
public:
    SerialCommunicator(
        rclcpp::Node* node, // Pass node for clock and logger
        const std::string & port,
        int baud_rate,
        double reconnect_delay_sec,
        int max_reconnect_attempts);
    ~SerialCommunicator();

    bool open_port();
    void close_port();
    bool is_connected() const;
    bool ensure_connection();

    bool write_payload(const std::vector<uint8_t>& payload);
    bool read_payload(std::vector<uint8_t>& payload, size_t expected_payload_size);

    static uint8_t calculate_checksum(const uint8_t* data, size_t length);

private:
    bool configure_port();
    bool read_raw_data_into_buffer();


    rclcpp::Node* node_; // For logger and clock
    rclcpp::Logger logger_;
    int serial_fd_;
    std::string port_;
    int baud_rate_value_; // e.g., B115200
    
    double reconnect_delay_sec_;
    int max_reconnect_attempts_;
    int current_reconnect_attempts_;
    rclcpp::Time last_reconnect_attempt_time_;

    std::vector<uint8_t> rx_buffer_;
    static const size_t MAX_BUFFER_SIZE = PacketFormat::RX_FRAME_SIZE * 10; // Buffer for multiple frames
};

} // namespace rm_serial

#endif // RM_SERIAL__SERIAL_COMMUNICATOR_HPP_
