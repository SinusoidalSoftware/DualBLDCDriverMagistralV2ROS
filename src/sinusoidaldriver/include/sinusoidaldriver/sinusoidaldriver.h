#ifndef SINUSOIDAL_DRIVER_H
#define SINUSOIDAL_DRIVER_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>
#include <cstring>
#include "sinusoidaldriver/comm_driver.h"

class DualDriver : public hardware_interface::RobotHW 
{
    public:
        struct Config
        {
            std::string left_wheel_name = "front_left_wheel_joint";
            std::string right_wheel_name = "front_right_wheel_joint";
            float loop_rate = 10;
            std::string device = "/dev/ttyUSB0";
            int baud_rate = 115200;
            int timeout = 1000;
            int hall_counts_per_rev = 90;
            bool left_wheel_reversed = false;
            bool right_wheel_reversed = true;
        };

    private:
        struct Wheel
        {
            std::string name = "";
            int enc = 0;
            double cmd = 0;
            double pos = 0;
            double vel = 0;
            double eff = 0;
            double velSetPt = 0;
            double rads_per_count = 0;
            bool wheel_reversed = false;

            Wheel(const std::string &wheel_name, int counts_per_rev, bool reversed);
            double calcEncAngle();
        };
        struct DriverData
        {
            uint16_t  battery=0;
            uint16_t  globalerror=0;
            uint32_t  dc_curr_right=0;
            uint32_t  dc_curr_left=0;
            uint32_t  temperature=0;
        };

    public:
        DualDriver(const Config &cfg);
        void read();
        void write();
        void setCommunicationTimeoutEnable();
        void setCommunicationTimeoutDisable();
        const ros::Time &get_time() { return time_; }
        const ros::Duration &get_period() { return period_; }
        DriverData driver_;

    private:
        hardware_interface::JointStateInterface jnt_state_interface_;
        hardware_interface::VelocityJointInterface jnt_vel_interface_;
        CommDriver commdriver_;
        Wheel l_wheel_;
        Wheel r_wheel_;

        ros::Time time_;
        ros::Duration period_;
        float loop_rate_;
  
};
#endif 