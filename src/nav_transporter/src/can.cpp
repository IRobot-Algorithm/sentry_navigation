#include "can.hpp"

namespace transporter
{
    int Can::send(uint id, u_char *buf, u_char dlc)
    {
        if (dlc > 8)
            return DLC_ERROR;

        struct can_frame send_frame;

        send_frame.can_id = id;
        send_frame.can_dlc = dlc;

        for (int i = 0; i < (int)dlc; i++)
            send_frame.data[i] = buf[i];

        int t = write(socket_fd, &send_frame, sizeof(send_frame));
        if (t > 0)
            return SUCCESS;
        return WRITE_ERROR;
    }

    int Can::receive(uint &id, u_char *buf, u_char &dlc)
    {
        struct can_frame frame;
        int t = read(socket_fd, &frame, sizeof(frame));
        if (t <= 0)
            return READ_ERROR;

        id = frame.can_id;
        dlc = frame.can_dlc;

        memcpy(buf, frame.data, dlc);
        return SUCCESS;
    }

    Can::Can()
    {
        socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        // 配置 Socket CAN 为非阻塞IO
        int flags = fcntl(socket_fd, F_GETFL, 0);
        flags |= O_NONBLOCK;
        fcntl(socket_fd, F_SETFL, flags);

        // 指定can设备
        strcpy(interface_request.ifr_name, CAN_NAME);
        ioctl(socket_fd, SIOCGIFINDEX, &interface_request);
        addr.can_family = AF_CAN;
        addr.can_ifindex = interface_request.ifr_ifindex;

        // 将套接字与can设备绑定
        bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr));
    }

    Can::~Can()
    {
        close(this->socket_fd);
    }
} // namespace transporter