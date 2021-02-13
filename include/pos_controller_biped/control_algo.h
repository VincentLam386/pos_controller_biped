#pragma once

#include <math.h>
#include <algorithm>
#include <deque>
#include "pos_controller_biped/pos_controller_biped.h"

struct IMUData{
  double acc[3];
  double omega[3];
};

void xyTipPlacement(std::vector<double>& xyTipPos, const std::vector<double>& linearVelFromJoint, const std::vector<double>& prevLinearVelFromJoint);

void update_control(std::vector<double>& commands, const double niu, const std::vector<hardware_interface::JointHandle>& joints_, const std::vector<double>& linearVelFromJoint, const std::vector<double>& rpyImu, const ros::Time& time);

void getJointVel(std::vector<double>& jointVel, std::deque< std::vector<double> >& jointPosCummulative, std::deque<uint64_t>& time_ms, const uint64_t curTime_msec, const std::vector<double>& jointPos);

void linksAngleAndVel(std::vector<double>& linksAngWithBase, std::vector<double>& linksVel, const std::vector<double>& jointPos, const std::vector<double>& jointVel);

void legTipForce(std::vector<double>& tipForce, const std::vector<double>& linksAngWithBase, const std::vector<double>& jointPos, const std::vector<double>& springCoef);

void rightStandForLinearVel(bool& rightStand, const std::vector<double>& tipForce);

void getLinearVelFromJoint(std::vector<double>& linearVelFromJoint, const bool& rightStand, const std::vector<double>& jointVel, const std::vector<double>& jointPos);

void rightStandForControl();

IMUData get_imu_data();
