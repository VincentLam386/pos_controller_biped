#pragma once

#include <math.h>
#include <algorithm>
#include <deque>
#include <queue>
#include "pos_controller_biped/pos_controller_biped.h"

struct IMUData{
  double acc[3];
  double omega[3];
};

void rightStandForControl(bool& rightStandControl, bool& dropping, bool& startTouch, const std::vector<double>& tipForce);

void targetXYTipPlacement(std::vector<double>& xyTipPosTarget,
                          std::queue< std::vector<double> >& aveLinearVel,
                          std::deque< std::vector<double> >& linearVelFromJoint,
                          const double* desired_vel,
                          const double* tipPlacementK);

void xyTipPlacementInControl_main(std::vector<double>& xyTipPos,
                                  const std::vector<double>& xyTipPosTarget);

void xyTipPlacementInControl_switch(std::vector<double>& xyTipPos,
                                    std::vector<double>& xyTipPosTarget,
                                    std::queue< std::vector<double> >& aveLinearVel,
                                    std::deque< std::vector<double> >& linearVelFromJoint,
                                    const double* desired_vel,
                                    const double* tipPlacementK);
 
double targetLegExtension(double thisAveLinearVel,
                          const double* desired_vel,
                          const double* extK);

void legExtensionInControl(std::vector<double>& currentExt, 
                           bool rightStandControl, 
                           double targetExt, 
                           const double* desired_vel,
                           const std::vector<double>& linksAngWithBase);

void update_control(bool& prevRightStandControl,
                    double& prevVel,
                    double& targetExt,
                    std::vector<double>& currentExt,
                    std::vector<double>& commands, 
                    std::vector<double>& xyTipPos, 
                    std::vector<double>& xyTipPosTarget,
                    std::queue< std::vector<double> >& aveLinearVel, 
                    std::deque< std::vector<double> >& linearVelFromJoint,
                    const bool rightStandControl, 
                    const std::vector<hardware_interface::JointHandle>& joints_, 
                    const std::vector<double>& rpyImu, 
                    const std::vector<double>& linksAngWithBase,
                    const ros::Time& time);

void getJointVel(std::vector<double>& jointVel, std::deque< std::vector<double> >& jointPosCummulative, std::deque<uint64_t>& time_ms, const uint64_t curTime_msec, const std::vector<double>& jointPos);

void linksAngleAndVel(std::vector<double>& linksAngWithBase, std::vector<double>& linksVel, const std::vector<double>& jointPos, const std::vector<double>& jointVel);

void legTipForce(std::vector<double>& tipForce, const std::vector<double>& linksAngWithBase, const std::vector<double>& jointPos, const std::vector<double>& springCoef);

void rightStandForLinearVel(bool& rightStand, const std::vector<double>& tipForce);

void getLinearVelFromJoint(std::deque< std::vector<double> >& linearVelFromJoint, const bool& rightStand, const std::vector<double>& jointVel, const std::vector<double>& jointPos);

void rightStandForControl();

IMUData get_imu_data();
