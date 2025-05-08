# ifndef RM_SERIAL_PACKET_TYPEDEF_HPP
#define RM_SERIAL_PACKET_TYPEDEF_HPP
#include<algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial
{
const uint8_t SOF_RECEIVE = 0x5A; // receive数据帧起始字节，固定值为 0x5A
const uint8_t SOF_SEND = 0x5A; // send数据帧起始字节，固定值为 0x5A

// Receive Data
const uint8_t ID_DEBUG = 0x01;
const uint8_t ID_GAME_STATUS = 0x02; // 游戏状态数据段id
const uint8_t ID_EVENT_DATA = 0x03;  // 事件数据段id
const uint8_t ID_ALL_ROBOT_HP = 0x04; // 所有机器人血量数据段id
const uint8_t ID_ROBOT_GIMBAL_POSE = 0x05; // 云台位姿数据段id
const uint8_t ID_ROBOT_CHASSIS_POSE = 0x06; // 底盘位姿数据段id
                                            
// Send Data
const uint8_t ID_ROBOT_CMD_DATA = 0x01; // 机器人控制数据段id



struct HeaderFrame
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));
 // namespace rm_serial

/***********************************************/
/* Receive Data                                */
/***********************************************/

// 串口调试数据包
struct DebugData
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
  uint8_t data[0]; // 数据段内容
} __attribute__((packed));

// 比赛状态数据包
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct {
    uint8_t game_type : 4;
    uint8_t game_progress : 4; // 具体数据包字节数还有待考证
    uint16_t stage_remain_time;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

/*
@brief 事件数据包
@details 事件数据包包含了非重叠补给区和重叠补给区的状态，暂时不考虑其他事件
@note 事件数据包的长度为 2 字节
@note 事件数据包的 CRC 校验为 2 字节
@note 事件数据包的帧头和帧尾均为 1 字节
@note 事件数据包的帧头为 0x5A
@note 事件数据包的帧尾为 0x5A
*/
// 0x020D “是否可以免费复活” 数据包
struct ReceiveIfRebornData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct __attribute__((packed))
  {
    uint32_t is_free_reborn     : 1;  // 是否可以确认免费复活
  } data;

  uint16_t crc;
} __attribute__((packed));

/*
@brief 所有机器人血量数据包
@details 所有机器人血量数据包包含了所有机器人的血量信息


*/

struct ReceiveAllRobotHpData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    // 现在有个问题是如果从当前的数据包读取哨兵的血量数据的话，该怎么去确定红蓝方的哨兵呢
    uint16_t red_7_robot_hp;
    uint16_t red_outpost_hp;
    uint16_t red_base_hp;

    uint16_t blue_7_robot_hp;
    uint16_t blue_outpost_hp;
    uint16_t blue_base_hp;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

struct ReceiveRobotGimbalPoseData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t robot_id;
    float gimbal_roll;
    float gimbal_yaw;
    float gimbal_pitch;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

struct ReceiveRobotChassisPoseData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    uint8_t robot_id;
    float chassis_roll;
    float chassis_pitch;
    float chassis_yaw;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

/********************************************/
/* Send Data                                */
/********************************************/

struct SendRobotCmdData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float vw;
    } __attribute__((packed)) speed_vector;

    struct
    {
      uint8_t move_mode;
    } __attribute__((packed)) move_mode_vector; // 适配哨兵电控端的控制逻辑是模式控制
    
    struct
    {
      float pitch_diff;
      float yaw_diff;
    } __attribute__((packed)) gimbal_vector;

    struct
    {
      float distance;
    } __attribute__((packed)) move_vector;
    struct
    {
      uint32_t fire_advice;
    } __attribute__((packed)) fire_vector;


  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}// namespace rm_serial

#endif // RM_SERIAL_PACKET_TYPEDEF_HPP
