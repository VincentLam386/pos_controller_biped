#pragma once

#include <math.h>
#include <algorithm>
#include "pos_controller_biped/pos_controller_biped.h"

struct IMUData{
  double acc[3];
  double omega[3];
};

void update_control(std::vector<double>& commands, const double niu, const std::vector<hardware_interface::JointHandle>& joints_, const std::vector<double>& linearVelFromJoint, const std::vector<double>& rpyImu, const ros::Time& time);

void getLinearVelFromJoint(std::vector<double>& linearVelFromJoint, bool& rightStand, const std::vector<double>& jointVel, const std::vector<double>& jointPos, const double duration);

void motorAngleAndVel(std::vector<double>& motorAngWithBase, std::vector<double>& motorVel, const std::vector<double>& jointPos, const std::vector<double>& jointVel);

void legTipForce(std::vector<double>& tipForce, const std::vector<double>& motorAngWithBase, const std::vector<double>& jointPos, const std::vector<double>& springCoef);

void rightStandForLinearVel();

void rightStandForControl();

IMUData get_imu_data();
