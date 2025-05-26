#include "rm_serial/serial_communicator.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring> // For strerror, memset, memcpy
#include <termios.h> // For termios struct and baud rate constants
#include <algorithm> // For std::min
#include <chrono> // For time measurements

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
        checksum ^= data[i]; // XOR checksum calculation
    }
    return checksum;
}

bool SerialCommunicator::write_payload(const std::vector<uint8_t>& payload)
{
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger_, "Write_payload: Serial port not open.");
        return false;
    }
    if (payload.empty()) { // Corrected variable name from data_payload to payload
        RCLCPP_WARN(logger_, "Write_payload: Payload is empty, nothing to send.");
        return true; // Or false, depending on desired behavior for empty payload
    }

    std::vector<uint8_t> frame;
    // Reserve space for: Header (1) + Payload (payload.size()) + CRC (2) + Tail (1)
    frame.reserve(1 + payload.size() + 2 + 1); 

    // 1. Add Header
    frame.push_back(PacketFormat::FRAME_HEADER);
    // 2. Add Payload
    frame.insert(frame.end(), payload.begin(), payload.end());
    
    // 3. Calculate CRC on (Header + Payload)
    // At this point, frame contains Header + Payload.
    uint16_t crc_value = crc8::get_CRC8_check_sum(
        frame.data(), frame.size(), PacketFormat::CRC8_INIT);
        
    // 4. Add CRC (LSB, then MSB)
    frame.push_back(static_cast<uint8_t>(crc_value & 0xFF)); // LSB
    frame.push_back(static_cast<uint8_t>((crc_value >> 8) & 0xFF)); // MSB
    
    // 5. Add Tail
    frame.push_back(PacketFormat::FRAME_TAIL);

    ssize_t total_written = 0;
    const size_t frame_size = frame.size();
    int eagain_retries = 0;
    auto write_start_time = std::chrono::steady_clock::now();
    
    while (total_written < static_cast<ssize_t>(frame_size)) {
        ssize_t written = write(serial_fd_, frame.data() + total_written, frame_size - total_written);
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                eagain_retries++;
                // Non-blocking, try again later or use select/poll
                // RCLCPP_DEBUG(logger_, "Write would block, trying again."); // This might be too verbose if it happens often
                rclcpp::sleep_for(std::chrono::microseconds(100)); // Small delay
                
                // Safety break if it retries too many times, indicating a persistent issue
                if (eagain_retries > 1000) { // Threshold for too many retries, adjust as needed
                    RCLCPP_ERROR(logger_, "Write_payload: Excessive EAGAIN retries (%d) for a single frame. Aborting write.", eagain_retries);
                    auto write_end_time_err = std::chrono::steady_clock::now();
                    auto duration_err = std::chrono::duration_cast<std::chrono::milliseconds>(write_end_time_err - write_start_time);
                    RCLCPP_WARN(logger_, "Write_payload: Time spent before aborting due to EAGAIN: %ld ms", duration_err.count()); // Changed %lld to %ld
                    return false; 
                }
                continue;
            }
            RCLCPP_ERROR(logger_, "Failed to write to serial port %s: %s", port_.c_str(), strerror(errno));
            if (errno == EIO || errno == ENODEV || errno == ENXIO) { // Critical errors
                close_port();
            }
            return false;
        }
        if (written == 0 && frame_size > 0) { // Should not happen with O_NONBLOCK if no error
             RCLCPP_WARN(logger_, "Write returned 0, port may be problematic. Retrying.");
             rclcpp::sleep_for(std::chrono::microseconds(100));
             eagain_retries++; // Count this as a retry scenario as well
             if (eagain_retries > 1000) {
                 RCLCPP_ERROR(logger_, "Write_payload: Excessive retries (write returned 0) for a single frame. Aborting write.");
                 return false;
             }
             continue;
        }
        total_written += written;
    }

    auto write_end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(write_end_time - write_start_time);

    if (eagain_retries > 0) {
        RCLCPP_WARN(logger_, "Write_payload: Frame (size %zu) sent with %d EAGAIN retries. Total write time: %ld µs.", frame_size, eagain_retries, duration.count()); // Changed %lld to %ld
    } else {
        RCLCPP_DEBUG(logger_, "Write_payload: Frame (size %zu) sent successfully. Write time: %ld µs.", frame_size, duration.count()); // Changed %lld to %ld
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

    const size_t crc_field_size = 1; // CRC is 1 bytes
    // Expected frame: Header (1) + Payload (expected_payload_size) + CRC (1) + Tail (1)
    const size_t expected_frame_size = 1 + expected_payload_size + crc_field_size + 1;

    for (size_t i = 0; (i + expected_frame_size) <= rx_buffer_.size(); ) {
        if (rx_buffer_[i] == PacketFormat::FRAME_HEADER) {
            // Check if the tail byte matches at the expected position
            if (rx_buffer_[i + expected_frame_size - 1] == PacketFormat::FRAME_TAIL) {
                // Potential frame found. Verify CRC.
                // Data for CRC check: Header + Payload
                // Starts at rx_buffer_[i], length is 1 (Header) + expected_payload_size
                uint8_t* data_for_crc_ptr = rx_buffer_.data() + i; // Removed const
                size_t length_for_crc = 1 + expected_payload_size;
                
                uint8_t calculated_crc = crc8::get_CRC8_check_sum(
                    data_for_crc_ptr, length_for_crc, PacketFormat::CRC8_INIT);
                
                // Extract received CRC from frame (LSB then MSB)
                // CRC LSB is after Header and Payload: index i + 1 (header) + expected_payload_size
                uint8_t received_crc_lsb = rx_buffer_[i + 1 + expected_payload_size];
                uint8_t received_crc_msb = rx_buffer_[i + 1 + expected_payload_size + 1];
                uint16_t received_crc = (static_cast<uint16_t>(received_crc_msb) << 8) | received_crc_lsb;

                if (calculated_crc == received_crc) {
                    // CRC match: Valid frame
                    // Extract payload (it's after the header at rx_buffer_[i+1])
                    payload.assign(rx_buffer_.data() + i + 1, 
                                   rx_buffer_.data() + i + 1 + expected_payload_size);
                    
                    // Remove the processed frame from the buffer
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + expected_frame_size);
                    return true;
                } else {
                    // CRC mismatch
                    RCLCPP_WARN(logger_, "CRC mismatch. Expected 0x%04X, Got 0x%04X. Discarding header byte 0x%02X at index %zu.",
                                calculated_crc, received_crc, rx_buffer_[i], i);
                    // Discard the header and continue search from the next byte
                    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + 1);
                    i = 0; // Restart search from beginning of modified buffer
                    continue; 
                }
            } else {
                // Header found, but tail doesn't match at expected position.
                RCLCPP_DEBUG(logger_, "Header found at index %zu, but tail mismatch (expected 0x%02X at index %zu, got 0x%02X). Discarding header byte 0x%02X.",
                             i, PacketFormat::FRAME_TAIL, i + expected_frame_size - 1, rx_buffer_[i + expected_frame_size - 1], rx_buffer_[i]);
                // Discard the header and continue search from the next byte
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + i + 1);
                i = 0; // Restart search from beginning of modified buffer
                continue; 
            }
        }
        // If not a header at rx_buffer_[i], advance search by one byte.
        // If we erased, set i=0, and continued, this i++ is skipped for that iteration.
        i++;
    }
    return false; // No complete, valid frame found in the current buffer
}

} // namespace rm_serial
