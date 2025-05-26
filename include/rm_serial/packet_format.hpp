#ifndef RM_SERIAL__PACKET_FORMAT_HPP_
#define RM_SERIAL__PACKET_FORMAT_HPP_

#include <cstdint>
#include <cstddef> // For size_t

// 数据包格式定义
struct PacketFormat {
    // 协议常量定义
    enum : uint8_t {
        FRAME_HEADER = 0xFF,  // 帧头
        FRAME_TAIL = 0x0D     // 帧尾
    };
    // CRC16校验和初始值
    static const uint16_t CRC16_INIT = 0xFFFF;
    // CRC8校验和初始值 (假设一个常用的CRC8初始值，您可能需要根据您的CRC8算法调整)
    static const uint8_t CRC8_INIT = 0x00;


    struct __attribute__((packed)) RxPacket {
        uint8_t aim_color; // 自瞄颜色
        float roll; // 车体roll角
        float pitch_gimbal; // 云台(小yaw)pitch角
        float yaw_gimbal; // 云台(小yaw)yaw角
        float pitch_chassis; // 大yaw pitch角
        float yaw_chassis; // 大yaw yaw角
        uint8_t game_status; // 比赛状态
        uint16_t remaining_time; // 剩余时间
        uint16_t blood;
        uint16_t outpost_hp; // 己方前哨站血量
        uint16_t projectile_allowance_17mm; // 17mm弹丸允许发弹量
        // uint16_t 
        bool is_rfid; // 是否为补给区域RFID
    };
    
    struct __attribute__((packed)) TxPacket {
        float pitch_diff;
        float yaw_diff;
        float distance;
        uint32_t fire_advice;
        float vx;
        float vy;
        float vw;
        uint32_t move_mode;
    };
    
    // 数据包本身大小
    static const size_t RX_PACKET_SIZE = sizeof(RxPacket);
    static const size_t TX_PACKET_SIZE = sizeof(TxPacket);
    
    // 帧总大小 = 帧头(1字节) + 数据包 + CRC8校验和(1字节) + 帧尾(1字节)
    static const size_t RX_FRAME_SIZE = RX_PACKET_SIZE + 1 + 1 + 1; // Header + Payload + CRC8 + Tail
    static const size_t TX_FRAME_SIZE = TX_PACKET_SIZE + 1 + 1 + 1; // Header + Payload + CRC8 + Tail
};

#endif // RM_SERIAL__PACKET_FORMAT_HPP_
