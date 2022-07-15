#ifndef COMM_DRIVER_H
#define COMM_DRIVER_H

#include <serial/serial.h>
#include <cstring>
#include "sinusoidaldriver/comm_driver.h"

#define STARTDATA 'S'
#define SYNCDATA  'Y'
#define STOPDATA1 'A'
#define STOPDATA2 'B'
#define STOPDATA3 'C'
#define STOPDATA4 'D'

class CommDriver
{
  public:
    CommDriver(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
        : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
    {  }
    bool readDriverData(void);
    void setMotorValues(void);
    void communicationTimeoutEnable(void);
    void communicationTimeoutDisable(void);
    bool connected() const { return serial_conn_.isOpen(); }
    void sendMsg(void);
    struct reference_data
    {
      int32_t reference_right=0;
      int32_t reference_left=0;
    };
    struct control_data
    {
      int32_t  counter_right=0;
      int32_t  counter_left=0;
      uint16_t  battery=0;
      uint16_t  globalerror=0;
      uint32_t  dc_curr_right=0;
      uint32_t  dc_curr_left=0;
      uint32_t  temperature=0;
    };
    struct usb_data
    {
      uint8_t start=STARTDATA;
      uint8_t sync=SYNCDATA;
      uint8_t packageno=0;
      uint8_t command=0;
      uint8_t data[52];
      uint32_t checksum=0;
      uint8_t stop1=STOPDATA1;
      uint8_t stop2=STOPDATA2;
      uint8_t stop3=STOPDATA3;
      uint8_t stop4=STOPDATA4;
    };


  private:
    serial::Serial serial_conn_;

  public:
      usb_data receive_data_;
      usb_data send_data_;
      control_data control_data_;
      reference_data reference_data_;
};
#endif 