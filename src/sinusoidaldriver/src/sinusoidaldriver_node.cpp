#include <ros/ros.h>
#include <sinusoidaldriver/sinusoidaldriver.h>
#include <controller_manager/controller_manager.h>
#include "sinusoidaldriver/comm_driver.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_driver");
    ros::NodeHandle n("~");

    DualDriver::Config dualdriver_cfg;

    n.getParam("left_wheel_name", dualdriver_cfg.left_wheel_name);
    n.getParam("right_wheel_name", dualdriver_cfg.right_wheel_name);
    n.getParam("baud_rate", dualdriver_cfg.baud_rate);
    n.getParam("device", dualdriver_cfg.device);
    n.getParam("hall_counts_per_rev", dualdriver_cfg.hall_counts_per_rev);
    n.getParam("driver_loop_rate", dualdriver_cfg.loop_rate);
    n.getParam("left_wheel_reversed", dualdriver_cfg.left_wheel_reversed);
    n.getParam("right_wheel_reversed", dualdriver_cfg.right_wheel_reversed);

    ros::Publisher voltage_pub = n.advertise<std_msgs::UInt16>("battery", 1000);
    ros::Publisher error_pub = n.advertise<std_msgs::UInt16>("error", 1000);
    ros::Publisher current_r_pub = n.advertise<std_msgs::UInt32>("currentleft", 1000);
    ros::Publisher current_l_pub = n.advertise<std_msgs::UInt32>("currentright", 1000);
    ros::Publisher temperature_pub = n.advertise<std_msgs::UInt32>("temperature", 1000);

    DualDriver driver(dualdriver_cfg);
    controller_manager::ControllerManager cm(&driver);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Time prevTime = ros::Time::now();
    ros::Rate loop_rate(10);
    std_msgs::UInt16 volt;
    std_msgs::UInt16 error;
    std_msgs::UInt32 currr;
    std_msgs::UInt32 currl;
    std_msgs::UInt32 temper;
    driver.setCommunicationTimeoutEnable();

    while (ros::ok())
    {
        driver.read();
        cm.update(driver.get_time(), driver.get_period());
        driver.write();
        volt.data = driver.driver_.battery;
        error.data = driver.driver_.globalerror;
        currr.data = driver.driver_.dc_curr_right;
        currl.data = driver.driver_.dc_curr_left;
        temper.data = driver.driver_.temperature;
        voltage_pub.publish(volt);
        error_pub.publish(error);
        current_r_pub.publish(currr);
        current_l_pub.publish(currl);
        temperature_pub.publish(temper);
        loop_rate.sleep();
    }
}