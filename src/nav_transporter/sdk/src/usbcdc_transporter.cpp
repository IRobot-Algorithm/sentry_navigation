/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 13:56:36
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 14:56:21
 */
#include "usbcdc_transporter.hpp"

namespace transporter_sdk
{

bool UsbcdcTransporter::open()
{
    // 初使化libusb
    int ret = libusb_init(&libusb_context_);
    if (ret != libusb_error::LIBUSB_SUCCESS) {
        libusb_exit(libusb_context_);
        return false;
    }
    // 打开指定厂商的某类产品
    device_handle = libusb_open_device_with_vid_pid(libusb_context_, vid_, pid_);
    if (device_handle == NULL) {
        libusb_exit(libusb_context_);
        return false;
    }
    

    // 卸载使用的USBCDC接口内核驱动
    while (libusb_kernel_driver_active(device_handle, 0) != libusb_error::LIBUSB_SUCCESS) {
        libusb_detach_kernel_driver(device_handle, 0);
    }

    // 打开指定接口
    if ((libusb_claim_interface(device_handle, 0)) != libusb_error::LIBUSB_SUCCESS) {
        libusb_close(device_handle);
        libusb_exit(libusb_context_);
        return false;
    }
    
    return true;
}

bool UsbcdcTransporter::close()
{
    libusb_close(device_handle);
    libusb_exit(libusb_context_);
    return true;
}

int UsbcdcTransporter::read(unsigned char * buffer, size_t len)
{
    int ReceiveSize = 0;
    if (libusb_bulk_transfer(device_handle, read_endpont_, buffer, len, &ReceiveSize, read_timeout_) < 0) {
        return -1;
    }
    return ReceiveSize;
}

int UsbcdcTransporter::write(unsigned char * buffer, size_t len)
{
    int ActualTransmitSize = 0;
    return libusb_bulk_transfer(device_handle, write_endpoint_, buffer, len, &ActualTransmitSize, write_timeout_);
}

}
