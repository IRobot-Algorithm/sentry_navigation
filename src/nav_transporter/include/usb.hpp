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
#define MAP_DATA_SEND_ID 0x8
#define UWB_RECEIVE_ID 0x9

namespace transporter
{


typedef signed char     int8_t;
typedef unsigned char   uint8_t;
typedef signed short    int16_t;
typedef unsigned short  uint16_t;
typedef signed int      int32_t;
typedef unsigned int    uint32_t;


#pragma pack(push, 1)

typedef struct
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
  // game state
  uint8_t game_type_progress;
  uint16_t game_stage_remain_time;
  // enemy information and outpose base HP
  uint16_t red_hero_remain_HP;
  uint16_t red_engineer_remain_HP;
  uint16_t red_infantry3_remain_HP;
  uint16_t red_infantry4_remain_HP;
  uint16_t red_infantry5_remain_HP;
  uint16_t red_sentry_remain_HP;
  uint16_t red_outpose_HP;
  uint16_t red_base_HP;
  uint16_t blue_hero_remain_HP;
  uint16_t blue_engineer_remain_HP;
  uint16_t blue_infantry3_remain_HP;
  uint16_t blue_infantry4_remain_HP;
  uint16_t blue_infantry5_remain_HP;
  uint16_t blue_sentry_remain_HP;
  uint16_t blue_outpose_HP;
  uint16_t blue_base_HP;
  // 基地护甲
  uint8_t base_state;
  // 机器人自身信息
  uint8_t robot_id;
  uint16_t max_HP;
  // 剩余弹量与金币
  uint16_t projectile_allowance_17mm;
  uint16_t remaining_gold_coin;
  uint16_t bought_bullets;
  // rfid
  uint32_t rfid_status;
  // 遥控器模式
  uint8_t mode;
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
  float q0;
  float q1;
  float q2;
  float q3;
  // 包尾
  uint8_t _EOF;
} NavIMUReceivePackage;

typedef struct
{
  // 包头
  uint8_t _SOF;
  uint8_t ID;
	// 坐标
	float x;
	float y;
  // 包尾
  uint8_t _EOF;
} UwbReceivePackage;

typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// nuc控制
		uint8_t chassis_mode;
    float vx;
    float vy;
    float yaw_imu;
    uint8_t direction; // 0 正常 / 1 逆时针 / 2 顺时针
    uint8_t capacitance; // 0 关闭 // 1 开启
    uint32_t sentry_cmd;
    // 包尾
    uint8_t _EOF;
} NavVelocitySendPackage;

typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
    // 标识
    uint8_t intention;
    // 起点坐标 单位dm
    uint16_t start_position_x;
    uint16_t start_position_y;
    // 增量数组 单位dm
    int8_t delta_x[49];
    int8_t delta_y[49];
    // 发送者ID
    uint16_t sender_id;
    // 包尾
    uint8_t _EOF;
} MapDataSendPackage;

typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// imu数据
		uint32_t TimeStamp;
    float q0;
    float q1;
    float q2;
    float q3;
    // 包尾
    uint8_t _EOF;
} RMOSIMUReceivePackage;

typedef struct
{
    // 包头
    uint8_t _SOF;
    uint8_t ID;
		// 自瞄状态
		uint8_t AimbotState;
		// 自瞄数据
		float PitchRelativeAngle;
    float YawRelativeAngle;
    uint32_t SystemTimer;
    // 包尾
    uint8_t _EOF;
} RMOSSendPackage;

#pragma pack(pop)

} // namespace transporter


#endif



