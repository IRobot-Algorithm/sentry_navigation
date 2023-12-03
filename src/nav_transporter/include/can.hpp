#ifndef _CAN_HPP_
#define _CAN_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <chrono>
#include <iostream>

#define CAN_NAME "can0"

#define STATE_SEND_ID 0x106
#define DATA_SEND_ID 0x108

#define IMU_TIME_RECEIVE_ID 0x100    // 时间ID
#define IMU_RECEIVE_ID 0x0FF         // 四元数ID
#define MODE_RECEIVE_ID 0x110        // 模式ID
#define BS_RECEIVE_ID 0x125          // 弹速等级ID
#define ROBOT_RECEIVE_ID 0x129       // 机器人状态信息ID
#define COMPETITION_RECEVIE_ID 0x12B // 比赛信息ID

#define VELOCITY_SEND_ID 0x12C            // 哨兵目标速度ID
#define TASK_RESULT_ID 0x12D              //任务完成情况ID
#define SPINTIME_SEND_ID 0x12E            // 旋转时间ID

#define GIMBAL_MODE_ID 0x130            //云台模式ID
#define CHASSIS_MODE_ID 0x131           //底盘模式ID

#define ARMOR_ATTACK_ID 0x132         //裁判系统传来装甲板受攻击ID
#define GAME_MODE_ID 0x133           //云台模式ID
#define CLIENT_RECEIVE_ID 0x134     // 由云台手通过客户端发送的信息
#define ROBOT_HP_ID 0x135
#define CLIENT_RECEIVE_ID2 0x136
#define SPIN_AT_HOME 0x137
#define VEHICLE_YAW 0x138
#define NAV_STATE 0x139


namespace transporter
{
    class Can
    {
    private:
        int socket_fd;
        struct sockaddr_can addr;
        struct ifreq interface_request;

    public:
        /**
         *  @brief  接收数据，根据id区分数据包，需要大于1000Hz频率接收
         *  @return error_code
         */
        int receive(uint &id, u_char *buf, u_char &dlc);

        /**
         * @brief   发送数据
         * @param   id  数据对应的ID
         * @param   buf 数据，长度小于等于8
         * @param   dlc 数据长度
         * @return  error_code
         */
        int send(uint id, u_char *buf, u_char dlc);

        Can();
        ~Can();

        enum CanError
        {
            SUCCESS,
            DLC_ERROR,
            WRITE_ERROR,
            READ_ERROR,
            TIME_ERROR
        };
    };

} // namespace transporter

#endif