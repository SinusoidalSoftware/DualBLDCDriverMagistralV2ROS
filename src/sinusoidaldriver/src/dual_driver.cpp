#include <sinusoidaldriver/sinusoidaldriver.h>
#include "sinusoidaldriver/comm_driver.h"

DualDriver::Wheel::Wheel(const std::string &wheel_name, int counts_per_rev, bool reversed)
{
  name = wheel_name;
  rads_per_count = (2*M_PI)/counts_per_rev;
  wheel_reversed = reversed;
}


double DualDriver::Wheel::calcEncAngle()
{
  return enc * rads_per_count;
}


DualDriver::DualDriver(const Config &cfg)
    : commdriver_(cfg.device, cfg.baud_rate, cfg.timeout),
    l_wheel_(cfg.left_wheel_name, cfg.hall_counts_per_rev,cfg.left_wheel_reversed),
    r_wheel_(cfg.right_wheel_name, cfg.hall_counts_per_rev,cfg.right_wheel_reversed)
{
  loop_rate_ = cfg.loop_rate;

  hardware_interface::JointStateHandle state_handle_a(l_wheel_.name, &l_wheel_.pos, &l_wheel_.vel, &l_wheel_.eff);
  jnt_state_interface_.registerHandle(state_handle_a);

  hardware_interface::JointStateHandle state_handle_b(r_wheel_.name, &r_wheel_.pos, &r_wheel_.vel, &r_wheel_.eff);
  jnt_state_interface_.registerHandle(state_handle_b);

  registerInterface(&jnt_state_interface_);

  hardware_interface::JointHandle vel_handle_a(jnt_state_interface_.getHandle(l_wheel_.name), &l_wheel_.cmd);
  jnt_vel_interface_.registerHandle(vel_handle_a);

  hardware_interface::JointHandle vel_handle_b(jnt_state_interface_.getHandle(r_wheel_.name), &r_wheel_.cmd);
  jnt_vel_interface_.registerHandle(vel_handle_b);

  registerInterface(&jnt_vel_interface_);
}
void DualDriver::setCommunicationTimeoutEnable()
{
  commdriver_.communicationTimeoutEnable();
  commdriver_.sendMsg();
}
void DualDriver::setCommunicationTimeoutDisable()
{
  commdriver_.communicationTimeoutDisable();
  commdriver_.sendMsg();
}
void DualDriver::read()
{
    ros::Time new_time = ros::Time::now();
    period_ = new_time - time_;

    bool data_ready = commdriver_.readDriverData();
    if(data_ready)
    {
        switch(commdriver_.receive_data_.command)
        {
            case 0:
                memcpy(&commdriver_.control_data_,&commdriver_.receive_data_.data,24);
                r_wheel_.enc = commdriver_.control_data_.counter_right;
                l_wheel_.enc = commdriver_.control_data_.counter_left;
                driver_.battery = commdriver_.control_data_.battery;
                driver_.globalerror = commdriver_.control_data_.globalerror;
                driver_.dc_curr_right = commdriver_.control_data_.dc_curr_right;
                driver_.dc_curr_left = commdriver_.control_data_.dc_curr_left;
                driver_.temperature = commdriver_.control_data_.temperature;

                if(l_wheel_.wheel_reversed)
                {
                  l_wheel_.vel =  -l_wheel_.calcEncAngle() * loop_rate_;
                  l_wheel_.pos = (l_wheel_.pos + (l_wheel_.vel/loop_rate_));
                }
                else
                {
                  l_wheel_.vel =  l_wheel_.calcEncAngle() * loop_rate_;
                  l_wheel_.pos = (l_wheel_.pos + (l_wheel_.vel/loop_rate_));
                }
                if(r_wheel_.wheel_reversed)
                {
                  r_wheel_.vel =  -r_wheel_.calcEncAngle() * loop_rate_;
                  r_wheel_.pos = (r_wheel_.pos + (r_wheel_.vel/loop_rate_));
                }
                else
                {
                  r_wheel_.vel =  r_wheel_.calcEncAngle() * loop_rate_;
                  r_wheel_.pos = (r_wheel_.pos + (r_wheel_.vel/loop_rate_));
                }
                time_ = new_time;            
            break;
            case 1:
            break;
            case 2:
            break;
            case 3:
            break;
            default:
            break;
        }
    }
    else
    {
        printf("Package Lost\n");
    }

}

void DualDriver::write()
{
  if(l_wheel_.wheel_reversed)
  {
    commdriver_.reference_data_.reference_left = -l_wheel_.cmd * loop_rate_;
  }
  else
  {
    commdriver_.reference_data_.reference_left = l_wheel_.cmd * loop_rate_;
  }
  if(r_wheel_.wheel_reversed)
  {
    commdriver_.reference_data_.reference_right = -r_wheel_.cmd * loop_rate_;
  }
  else
  {
    commdriver_.reference_data_.reference_right = r_wheel_.cmd * loop_rate_;
  }
}