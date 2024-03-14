/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 16:03:01
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 16:06:21
 */

#ifndef USB_HPP
#define USB_HPP

#include "../sdk/include/usbcdc_transporter.hpp"

#define DESICION_REFEREE_RECEIVE_ID 0x1
#define RMOS_REFEREE_RECEIVE_ID 0x2
#define NAV_IMU_RECEIVE_ID 0x3
#define NAV_VELOCITY_SEND_ID 0x4
#define RMOS_IMU_0_RECEIVE_ID 0x5
#define RMOS_IMU_1_RECEIVE_ID 0x6
#define RMOS_SEND_ID 0x7

namespace transporter
{


typedef signed char     int8_t;
typedef unsigned char   uint8_t;
typedef signed short    int16_t;
typedef unsigned short  uint16_t;
typedef signed int      int32_t;
typedef unsigned int    uint32_t;


#pragma pack(push, 1)

// 49字节
typedef struct
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // game state
  uint8_t game_type_progress;
  uint16_t game_stage_remain_time;
  // enemy information and outpose base HP
  uint16_t hero_remain_HP;
  uint16_t engineer_remain_HP;
  uint16_t infantry3_remain_HP;
  uint16_t infantry4_remain_HP;
  uint16_t infantry5_remain_HP;
  uint16_t sentry_remain_HP;
  uint16_t red_outpose_HP;
  uint16_t blue_outpose_HP;
  uint16_t red_base_HP;
  uint16_t blue_base_HP;
  // 基地护甲
  uint8_t base_state;
  // 机器人自身信息
  uint8_t robot_id;
  uint16_t remain_HP;
  uint16_t max_HP;
  // 剩余弹量与金币
  uint16_t projectile_allowance_17mm;
  uint16_t remaining_gold_coin;
  // rfid
  uint32_t rfid_status;
  // 小地图
  float x;
	float y;
	uint8_t key;
  // 包尾
  uint8_t _EOF;
} DesicionRefereeReceivePackage;

typedef struct
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // 机器人自身信息
  uint8_t robot_id;
  uint16_t remain_HP;
  uint16_t max_HP;
  // 包尾
  uint8_t _EOF;
} RMOSRefereeReceivePackage;

// 23字节
typedef struct
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
	// 电机数据
	float RightMotorAngle;
	float LeftMotorAngle;
	// imu数据
	uint32_t TimeStamp;
  int16_t q0;
  int16_t q1;
  int16_t q2;
  int16_t q3;
  // 包尾
  uint8_t _EOF;
} NavIMUReceivePackage;

// 15字节
typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// nuc控制
    float vx;
    float vy;
    float yaw_imu;
    // 包尾
    uint8_t _EOF;
} NavVelocitySendPackage;

// 15字节
typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// imu数据
		uint32_t TimeStamp;
    int16_t q0;
    int16_t q1;
    int16_t q2;
    int16_t q3;
    // 包尾
    uint8_t _EOF;
} RMOSIMUReceivePackage;

// 15字节
typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// 自瞄状态
		uint8_t AimbotState;
		// 自瞄数据
		int16_t PitchRelativeAngle;
    int16_t YawRelativeAngle;
    uint32_t SystemTimer;
    // 包尾
    uint8_t _EOF;
} RMOSSendPackage;

#pragma pack(pop)

} // namespace transporter


#endif



