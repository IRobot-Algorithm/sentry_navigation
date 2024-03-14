/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 13:45:09
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 14:57:50
 */
#pragma once

#include "transporter_interface.hpp"
#include <libusb-1.0/libusb.h>

namespace transporter_sdk
{

class UsbcdcTransporter : public TransporterInterface
{
public:
    UsbcdcTransporter(uint vid, uint pid, 
                    uint read_endpoint, uint write_endpoint, 
                    uint read_timeout, uint write_timeout
                    )
                    : vid_(vid), pid_(pid), 
                    read_endpont_(read_endpoint), write_endpoint_(write_endpoint), 
                    read_timeout_(read_timeout), write_timeout_(write_timeout),
                    libusb_context_(NULL)
                    {}

    bool open() override;
    bool close() override;
    int read(unsigned char * buffer, size_t len) override;
    int write(unsigned char * buffer, size_t len) override;
    
private:
    uint vid_;
    uint pid_;
    uint read_endpont_;
    uint write_endpoint_;
    uint read_timeout_;
    uint write_timeout_;
    libusb_context* libusb_context_;
    libusb_device_handle* device_handle;
};

}

