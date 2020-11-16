#pragma once

#include <math.h>
#include <algorithm>
#include "pos_controller_biped/pos_controller_biped.h"

struct IMUData{
  double acc[3];
  double omega[3];
};

void update_control(std::vector<double>& commands, const std::vector<hardware_interface::JointHandle>& joints_, const std::vector<double>& rpyImu, const ros::Time& time);

IMUData get_imu_data();
