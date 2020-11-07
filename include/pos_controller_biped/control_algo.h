#pragma once

#include <math.h>
#include <hardware_interface/joint_command_interface.h>

struct IMUData{
  double acc[3];
  double omega[3];
};

void update_control(int loop_count_, std::vector<double>& commands, const std::vector<hardware_interface::JointHandle>& joints_, ros::Time time);

IMUData get_imu_data();
