#pragma once

#include <math.h>
#include <algorithm>
#include <deque>
#include <queue>
#include "pos_controller_biped/pos_controller_biped.h"

void targetXYTipPlacement(std::vector<double>& xyTipPosTarget,
                          std::queue< std::vector<double> >& aveLinearVel,
                          std::deque< std::vector<double> >& linearVelFromLink,
                          const double* desired_vel,
                          const double* tipPlacementK,
                          bool fixPitch);

void xyTipPlacementInControl_main(std::vector<double>& xyTipPos,
                                  const std::vector<double>& xyTipPosTarget);

void xyTipPlacementInControl_switch(std::vector<double>& xyTipPos,
                                    std::vector<double>& xyTipPosTarget,
                                    std::queue< std::vector<double> >& aveLinearVel,
                                    std::deque< std::vector<double> >& linearVelFromLink,
                                    const double* desired_vel,
                                    const double* tipPlacementK);

void update_control(bool& prevRightStandControl,
                    std::vector<double>& commands, 
                    std::vector<double>& xyTipPos, 
                    std::vector<double>& xyTipPosTarget,
                    std::queue< std::vector<double> >& aveLinearVel, 
                    std::deque< std::vector<double> >& linearVelFromLink,
                    const bool stop,
                    const std::vector<double>& rpyImu, 
                    const ros::Time& time,
                    const double startControlTime);

bool cummulativeTimeUpdate(std::deque<uint64_t>& time_ms, 
                           const unsigned int storedNum,
                           const uint64_t curTime_msec);

void getVel(std::vector<double>& cumuVel, 
                 std::deque< std::vector<double> >& cummulative, 
                 const bool timeUpdated,
                 const unsigned int storedNum,
                 const std::vector<double>& newMeasure,
                 const std::deque<uint64_t>& time_ms);


void linksAngleAndVel(std::vector<double>& linksAngWithVert, 
                      std::vector<double>& linksAngVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel,
                      const std::vector<double>& rpyImu,
                      const std::vector<double>& rpyVel);

void legTipForce(std::vector<double>& tipForce, 
                 const std::vector<double>& linksAngWithBase, 
                 const std::vector<double>& jointPos, 
                 const std::vector<double>& springCoef);

void singleSupportSwitch(bool& swang,
                         unsigned int& walkingState, 
                         double phaseSwitchConst);

void doubleSupportSwitch(unsigned int& walkingState, 
                         bool rightToSwing, 
                         double phaseSwitchConst);

void rightStandForLinearVel(bool& rightStand, const std::vector<double>& tipForce);

void getLinearVelFromLink(std::deque< std::vector<double> >& linearVelFromLink, 
                           const bool& rightStand,
                           const std::vector<double>& linksAngWithVert,
                           const std::vector<double>& linksAngVel);

void update_control_free(bool& prevRightStandControl,
                    bool& swang,
                    unsigned int& walkingState,
                    double* targetPitch,
                    double* controlPitch,
                    std::vector<double>& commands, 
                    std::vector<double>& xyTipPos, 
                    std::vector<double>& xyTipPosTarget,
                    std::queue< std::vector<double> >& aveLinearVel, 
                    std::deque< std::vector<double> >& linearVelFromJoint,
                    const bool stop, 
                    const int loop_count_,
                    const std::vector<double>& rpyImu, 
                    const std::vector<double>& rpyVel,
                    const std::vector<double>& linksAngWithVert,
                    const ros::Time& time,
                    const double startControlTime);


