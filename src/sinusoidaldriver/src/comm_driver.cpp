#include "sinusoidaldriver/crc32.h"
#include "sinusoidaldriver/comm_driver.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>

bool CommDriver::readDriverData(void)
{
    uint8_t read_buf[64];
    bool receive_ready = true;
    
    uint8_t numberofbytesread = serial_conn_.read(read_buf,64);
    if (numberofbytesread < 0) {
        printf("No bytes received from driver\n");
        return false;
    }
    if(read_buf[0]!=STARTDATA)
    {
        receive_ready = false;
    }
    if(read_buf[1]!=SYNCDATA)
    {
        receive_ready = false;
    }
    if(read_buf[60]!=STOPDATA1)
    {
        receive_ready = false;
    }
    if(read_buf[61]!=STOPDATA2)
    {
        receive_ready = false;
    }
    if(read_buf[62]!=STOPDATA3)
    {
        receive_ready = false;
    }
    if(read_buf[63]!=STOPDATA4)
    {
        receive_ready = false;
    }
    if(receive_ready)
    {
        memcpy(&receive_data_,read_buf,64);
        uint32_t calculated_crc32 = GenerateCrc(receive_data_.data, 52);
        if(calculated_crc32 != receive_data_.checksum)
        {
            receive_ready = false;
            std::cout << std::hex << calculated_crc32;
            printf("\nChecksum Error\n");
            std::cout << std::hex << receive_data_.checksum;
        }
        else
        {
            setMotorValues();
            sendMsg();
        }
    }
    if(receive_ready==false)
    {
        printf("Pack Sync Starrted\n");
        uint8_t lost_package_lock = 0;
        uint8_t lost_count = 0;
        while((lost_package_lock!=4)&&(lost_count<65))
        {
            lost_count++;
            serial_conn_.read(read_buf,1);
            switch(read_buf[0])
            {
                case STOPDATA1:
                    lost_package_lock = 1;
                    break;
                case STOPDATA2:
                    if(lost_package_lock == 1)
                    {
                        lost_package_lock = 2;
                    }
                    else
                    {
                        lost_package_lock = 0;
                    }
                    break;
                case STOPDATA3:
                    if(lost_package_lock == 2)
                    {
                        lost_package_lock = 3;
                    }
                    else
                    {
                        lost_package_lock = 0;
                    }
                    break;
                case STOPDATA4:
                    if(lost_package_lock == 3)
                    {
                        lost_package_lock = 4;
                    }
                    else
                    {
                        lost_package_lock = 0;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return receive_ready;
}
void CommDriver::setMotorValues(void)
{
    send_data_.command = 1;
    memset(send_data_.data,'\0',52);
    memcpy(send_data_.data,&reference_data_,8);
    send_data_.checksum = GenerateCrc(send_data_.data, 52);
}
void CommDriver::communicationTimeoutEnable(void)
{
    send_data_.command = 5;
    memset(send_data_.data,'\0',52);
    send_data_.checksum = GenerateCrc(send_data_.data, 52);
}
void CommDriver::communicationTimeoutDisable(void)
{
    send_data_.command = 6;
    memset(send_data_.data,'\0',52);
    send_data_.checksum = GenerateCrc(send_data_.data, 52);
}

void CommDriver::sendMsg(void)
{
    uint8_t sentbuffer[64];
    memcpy(sentbuffer,&send_data_,64);
    serial_conn_.write(sentbuffer,64);
}