/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-05 13:38:37
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 14:56:51
 */
#pragma once

#include <memory>
#include <string>

namespace transporter_sdk
{

class TransporterInterface
{
public:
    virtual bool open() = 0;
    virtual bool close() = 0;
    virtual int read(unsigned char * buffer, size_t len) = 0;
    virtual int write(unsigned char * buffer, size_t len) = 0;
    
};



}

