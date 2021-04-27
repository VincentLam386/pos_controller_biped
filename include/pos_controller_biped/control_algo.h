#pragma once

#include <math.h>
#include <algorithm>
#include <deque>
#include <queue>
#include "pos_controller_biped/pos_controller_biped.h"

int boolToSgn(bool in);

void getAverageLinearVel(std::queue< std::array<double,2> >& aveLinearVel,
                         const std::deque< std::array<double,3> >& linearVelFromLink);

void targetXYTipPlacement(std::array<double,2>& xyTipPosTarget,
                          std::queue< std::array<double,2> >& aveLinearVel,
                          const std::deque< std::array<double,3> >& linearVelFromLink,
                          const std::array<double,2>& desired_vel,
                          const std::array<double,3>& tipPlacementK,
                          bool fixPitch);

void xyTipPlacementInControl_main(std::array<double,2>& xyTipPos,
                                  const std::array<double,2>& xyTipPosTarget,
                                  double maxStep );

double velStepCal(long period, const std::array<double,2> & xyTipPosTarget);

void update_control(bool& prevRightStandControl,
                    double& velStep,
                    std::vector<double>& commands, 
                    std::array<double,2>& xyTipPos, 
                    std::array<double,2>& xyTipPosTarget,
                    std::queue< std::array<double,2> >& aveLinearVel, 
                    const std::deque< std::array<double,3> >& linearVelFromLink,
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


void linksAngleAndVel(std::array< std::array<double,3> ,2>& linksAngWithVert, 
                      std::array< std::array<double,3> ,2>& linksAngVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel,
                      const std::vector<double>& rpyImu,
                      const std::vector<double>& rpyVel);

void legTipForce(std::array< std::array<double,2> ,2>& tipForce, 
                 const std::array< std::array<double,3> ,2>& linksAngWithVert, 
                 const std::vector<double>& jointPos, 
                 const std::array<double,2>& springCoef);

void singleSupportSwitch(bool& swang,
                         unsigned int& walkingState, 
                         double phaseSwitchConst);

void doubleSupportSwitch(unsigned int& walkingState, 
                         bool rightToSwing, 
                         double phaseSwitchConst);

void rightStandForLinearVel(bool& rightStand, 
                            const std::array< std::array<double,2> ,2>& tipForce);

void getLinearVelFromLink(std::deque< std::array<double,3> >& linearVelFromLink, 
                           const bool& rightStand,
                           const std::array< std::array<double,3> ,2>& linksAngWithVert,
                           const std::array< std::array<double,3> ,2>& linksAngVel);

void update_control_free(bool& prevRightStandControl,
                    bool& swang,
                    unsigned int& walkingState,
                    double& phaseSwitchConst,
                    double* targetPitch,
                    double* controlPitch,
                    std::array<double,4>& controlStandAngle,
                    std::vector<double>& commands, 
                    std::array<double,2>& xyTipPos, 
                    std::array<double,2>& xyTipPosTarget,
                    std::queue< std::array<double,2> >& aveLinearVel, 
                    const std::deque< std::array<double,3> >& linearVelFromLink,
                    const bool stop, 
                    const int loop_count_,
                    const std::vector<double>& rpyImu, 
                    const std::vector<double>& rpyVel,
                    const std::array< std::array<double,3> ,2>& linksAngWithVert,
                    const ros::Time& time,
                    const double startControlTime);

void writeToFile(std::string msg);


