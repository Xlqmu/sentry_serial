#include "rm_serial/serial_communicator.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <cstring> // For strerror, memset, memcpy
#include <termios.h> // For termios struct and baud rate constants
#include <algorithm> // For std::min

namespace rm_serial
{

SerialCommunicator::SerialCommunicator(
    rclcpp::Node* node,
    const std::string & port,
    int baud_rate, // Actual baud rate integer, e.g., 115200
    double reconnect_delay_sec,
    int max_reconnect_attempts)
: node_(node),
  logger_(node->get_logger().get_child("serial_communicator")),
  serial_fd_(-1),
  port_(port),
  reconnect_delay_sec_(reconnect_delay_sec),
  max_reconnect_attempts_(max_reconnect_attempts),
  current_reconnect_attempts_(0),
  last_reconnect_attempt_time_(node->now())
{
    if (baud_rate == 115200) {
        baud_rate_value_ = B115200;
    } else if (baud_rate == 460800) {
        baud_rate_value_ = B460800;
    } else if (baud_rate == 921600) {
        baud_rate_value_ = B921600;
    }
    // Add more baud rates if needed
    else {
        RCLCPP_ERROR(logger_, "Unsupported baud rate: %d. Defaulting to 115200.", baud_rate);
        baud_rate_value_ = B115200;
    }
}

SerialCommunicator::~SerialCommunicator()
{
    close_port();
}

bool SerialCommunicator::open_port()
{
    close_port(); // Ensure any existing port is closed

    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Failed to open serial port %s: %s", port_.c_str(), strerror(errno));
        return false;
    }

    if (!configure_port()) {
        close_port();
        return false;
    }

    RCLCPP_INFO(logger_, "Serial port %s opened successfully.", port_.c_str());
    current_reconnect_attempts_ = 0; // Reset on successful open
    rx_buffer_.clear(); // Clear buffer on new connection
    return true;
}

bool SerialCommunicator::configure_port()
{
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd_, &tty) != 0) {
        RCLCPP_ERROR(logger_, "Failed to get serial port attributes: %s", strerror(errno));
        return false;
    }

    cfsetospeed(&tty, baud_rate_value_);
    cfsetispeed(&tty, baud_rate_value_);

    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, ignore modem control lines
    tty.c_cflag &= ~CSIZE;              // Clear data size bits
    tty.c_cflag |= CS8;                 // 8 data bits
    tty.c_cflag &= ~PARENB;             // No parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // No hardware flow control

    // Input modes: no break processing, no CR to NL, no parity check, no strip char, no XON/XOFF
    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | ISTRIP | IXON);
    // Output modes: raw output
    tty.c_oflag &= ~OPOST;
    // Local modes: no canonical, no echo, no signals
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // Control characters: read returns immediately with available data
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0; // Non-blocking read

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(logger_, "Failed to set serial port attributes: %s", strerror(errno));
        return false;
    }
    return true;
}

void SerialCommunicator::close_port()
{
    if (serial_fd_ >= 0) {
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(logger_, "Serial port %s closed.", port_.c_str());
    }
}

bool SerialCommunicator::is_connected() const
{
    if (serial_fd_ < 0) return false;
    struct termios tty;
    return tcgetattr(serial_fd_, &tty) == 0;
}

bool SerialCommunicator::ensure_connection()
{
    if (is_connected()) {
        return true;
    }

    RCLCPP_WARN(logger_, "Serial port disconnected. Attempting to reconnect...");
    close_port(); // Clean up before trying to reopen

    if (max_reconnect_attempts_ > 0 && current_reconnect_attempts_ >= max_reconnect_attempts_) {
        RCLCPP_ERROR(logger_, "Maximum reconnect attempts (%d) reached for %s.", max_reconnect_attempts_, port_.c_str());
        return false;
    }

    if ((node_->now() - last_reconnect_attempt_time_).seconds() < reconnect_delay_sec_) {
        return false; // Wait for reconnect delay
    }
    
    current_reconnect_attempts_++;
    last_reconnect_attempt_time_ = node_->now();
    RCLCPP_INFO(logger_, "Attempting to reconnect to %s (attempt %d/%s)...",
               port_.c_str(), current_reconnect_attempts_,
               max_reconnect_attempts_ > 0 ? std::to_string(max_reconnect_attempts_).c_str() : "unlimited");

    if (open_port()) {
        RCLCPP_INFO(logger_, "Successfully reconnected to %s.", port_.c_str());
        return true;
    } else {
        RCLCPP_WARN(logger_, "Reconnect attempt failed for %s. Will retry in %.1f seconds.", port_.c_str(), reconnect_delay_sec_);
        return false;
    }
}

uint8_t SerialCommunicator::calculate_checksum(const uint8_t* data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i]; // XOR checksum
    }
    return checksum;
}

bool SerialCommunicator::write_payload(const std::vector<uint8_t>& payload)
{
    if (serial_fd_ < 0 || payload.empty()) {
        return false;
    }

    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 3); // Header + Payload + Checksum + Tail
    frame.push_back(PacketFormat::FRAME_HEADER);
    frame.insert(frame.end(), payload.begin(), payload.end());
    uint8_t checksum = calculate_checksum(payload.data(), payload.size());
    frame.push_back(checksum);
    frame.push_back(PacketFormat::FRAME_TAIL);

    ssize_t total_written = 0;
    const size_t frame_size = frame.size();
    
    while (total_written < static_cast<ssize_t>(frame_size)) {
        ssize_t written = write(serial_fd_, frame.data() + total_written, frame_size - total_written);
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Non-blocking, try again later or use select/poll
                RCLCPP_DEBUG(logger_, "Write would block, trying again.");
                rclcpp::sleep_for(std::chrono::microseconds(100)); // Small delay
                continue;
            }
            RCLCPP_ERROR(logger_, "Failed to write to serial port %s: %s", port_.c_str(), strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) { // Critical errors
                close_port();
            }
            return false;
        }
        if (written == 0 && frame_size > 0) { // Should not happen with O_NONBLOCK if no error
             RCLCPP_WARN(logger_, "Write returned 0, port may be problematic.");
             rclcpp::sleep_for(std::chrono::microseconds(100));
             continue;
        }
        total_written += written;
    }
    return true;
}


bool SerialCommunicator::read_raw_data_into_buffer() {
    if (serial_fd_ < 0) return false;

    uint8_t temp_buffer[1024]; // Temporary buffer for read
    ssize_t bytes_read = read(serial_fd_, temp_buffer, sizeof(temp_buffer));

    if (bytes_read > 0) {
        if (rx_buffer_.size() + bytes_read > MAX_BUFFER_SIZE) {
            RCLCPP_WARN(logger_, "RX buffer overflow, discarding %zu old bytes.", rx_buffer_.size() + bytes_read - MAX_BUFFER_SIZE);
            // Simple strategy: discard oldest data to make space
            // A more robust solution might involve a circular buffer or discarding oldest frames
            size_t excess = rx_buffer_.size() + bytes_read - MAX_BUFFER_SIZE;
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + excess);
        }
        rx_buffer_.insert(rx_buffer_.end(), temp_buffer, temp_buffer + bytes_read);
        return true;
    } else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(logger_, "Failed to read from serial port %s: %s", port_.c_str(), strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) { // Critical errors
                close_port();
            }
        }
    }
    // bytes_read == 0 means no data available (for non-blocking)
    return false; // No new data read or error
}


bool SerialCommunicator::read_payload(std::vector<uint8_t>& payload, size_t expected_payload_size)
{
    if (serial_fd_ < 0) return false;

    read_raw_data_into_buffer(); // Attempt to read new data

    const size_t expected_frame_size = 1 + expected_payload_size + 1 + 1; // Header + Payload + Checksum + Tail

    for (size_t i = 0; (i + expected_frame_size) <= rx_buffer_.size(); ) {
        if (rx_buffer_[i] == PacketFormat::FRAME_HEADER) {
            if (rx_buffer_[i + expected_frame_size - 1] == PacketFormat::FRAME_TAIL) {
                // Potential frame found
                const uint8_t* data_part_ptr = rx_buffer_.data() + i + 1;
                uint8_t received_checksum = rx_buffer_[i + 1 + expected_payload_size];
                uint8_t calculated_checksum = calculate_checksum(data_part_ptr, expected_payload_size);

                if (received_checksum == calculated_checksum) {
                    // Valid frame
                    payload.assign(data_part_ptr, data_part_ptr + expected_payload_size);
                    // Remove the processed frame from the buffer
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + expected_frame_size);
                    return true;
                } else {
                    RCLCPP_WARN(logger_, "Checksum mismatch. Expected 0x%02X, Got 0x%02X. Discarding byte 0x%02X.",
                                calculated_checksum, received_checksum, rx_buffer_[i]);
                    // Checksum failed, discard the header and continue search
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + 1);
                    i = 0; // Restart search from beginning of modified buffer
                    continue;
                }
            } else {
                // Header found, but tail doesn't match. This could be a corrupted frame or
                // the start of a new frame if expected_frame_size is wrong.
                // For now, assume it's a bad header and discard it.
                RCLCPP_DEBUG(logger_, "Header found at %zu, but tail mismatch. Discarding byte 0x%02X.", i, rx_buffer_[i]);
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + 1);
                i = 0; // Restart search
                continue;
            }
        }
        // If not a header, advance search by one byte in the original buffer logic
        // but since we erase, we just increment i if no header was found at current i
        i++;
    }
    return false; // No complete, valid frame found
}

} // namespace rm_serial
