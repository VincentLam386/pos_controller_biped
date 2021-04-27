#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include <ros/console.h>

int boolToSgn(bool in){
  return -1+in*2;
}

void getAverageLinearVel(std::queue< std::array<double,2> >& aveLinearVel,
                         const std::deque< std::array<double,3> >& linearVelFromLink){
  std::array<double,2> curVel {0.0, 0.0};
  for(unsigned int i=0; i<2; ++i){
    for(unsigned int j=0; j<linearVelFromLink.size(); ++j){
      curVel[i] += linearVelFromLink[j][i];
    } // for
    curVel[i] /= linearVelFromLink.size();
  } // for

  if(aveLinearVel.size() > 1){
    aveLinearVel.pop();
  } // if
  aveLinearVel.push(curVel);

  return;
}

/*----------------------------------------------------------------------------------------------*/

void targetXYTipPlacement(std::array<double,2>& xyTipPosTarget,
                          std::queue< std::array<double,2> >& aveLinearVel,
                          const std::deque< std::array<double,3> >& linearVelFromLink,
                          const std::array<double,2>& desired_vel,
                          const std::array<double,3>& tipPlacementK,
                          bool fixPitch){
  /*
  Find the average velocity during a step -> fill in the aveLinearVel queue of size 2

  Obtain average velocity of previous step and a step before previous, and

  Calculate the target tip placement along x and y direction and make sure it is within the preset range (x direction only)
  
  (test was done in x direction only)
  */
  const double maxX = 0.16;

  // Get average linear velocity
  getAverageLinearVel(aveLinearVel, linearVelFromLink);

  // Update target leg tip position
  std::array<double,2> curVel = aveLinearVel.back();  
  for(unsigned int i=0; i<2; ++i){
    xyTipPosTarget[i] = boolToSgn(fixPitch)*tipPlacementK[0]*(desired_vel[i]-curVel[i]) + 
                        boolToSgn(fixPitch)*tipPlacementK[1]*(curVel[i]-aveLinearVel.front()[i]) +
                        tipPlacementK[2]*curVel[i];
  } // for

  // Apply limit to target leg tip position
  xyTipPosTarget[0] = std::max(-maxX, std::min(maxX, xyTipPosTarget[0]));

  return;
} // targetXYTipPlacement

/*----------------------------------------------------------------------------------------------*/

void xyTipPlacementInControl_main(std::array<double,2>& xyTipPos,
                                  const std::array<double,2>& xyTipPosTarget,
                                  double maxStep = 0.00064){
  /*
  Move the leg tip progressively until it reach the target leg tip position when not switching leg
  */

  //double maxStep = 0.0005;  // fix pitch
  //double maxStep = 0.0007;
  //double maxStep = 0.001;  // free pitch
  //std::cout << maxStep << " ";
  // 0.5: 0.0064
  // 0.25: 0.0032

  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    double err = xyTipPos[i] - xyTipPosTarget[i];

    //writeToFile( std::to_string(err) + " ");

    if(err > 0){  
      xyTipPos[i] -= std::min(err,maxStep);
    } // if true
    else if(err < 0){
      xyTipPos[i] += std::min(-err,maxStep);
    } // else if true
  } // for
  
  return;
} // xyTipPlacementInControl_main

/*----------------------------------------------------------------------------------------------*/

double velStepCal(long period, const std::array<double,2> & xyTipPosTarget){
  return (xyTipPosTarget[0] * 2 * 1.15 / period);
}

/*----------------------------------------------------------------------------------------------*/

// Implement the control algorithm here
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
                    const double startControlTime)
{
  const double stepFrequency = 3;
  const double PI = 3.14159;
  const double maxAngle = 45.0 *(PI/180.0);
  const double leg_0 = 0.5;     //neutral length of leg
  const double leg_maxRet = 0.1; //max retraction length of the leg
  const double angle_0 = acos((leg_0/2)/(0.26)) - PI/6;

  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)*(time.toSec() - startControlTime));

  std::array<double,2> targetLegLength = {0.0, 0.0}; // left, right
  std::array<double,2> targetLegAngle =  {0.0,0.0}; // left, right
  std::array<double,4> controlAngle = {0.0,0.0, 0.0,0.0}; // left(miu,niu), right(miu,niu)

  std::array<double,2> desired_vel = { 0.5,0.0 };  // x direction, y direction
  //std::array<double,3> tipPlacementK = { 0.25, 0.0, 0.225 }; // kp, kd, kv
  std::array<double,3> tipPlacementK = { 0.3, 0.05, 0.225 }; // kp, kd, kv

  // Get the current actual hip pitch angles from IMU
  float pitchToFront = rpyImu[1];

  if(!stop){
    std::cout << time.toSec() << " ";
    if((retractionLength < 0) != prevRightStandControl){
      // Switch phase control
      targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromLink, desired_vel, tipPlacementK,true);

      for(unsigned int i=0; i<xyTipPos.size(); ++i){
        xyTipPos[i] *= -1;
      } // for
      velStep = velStepCal(1000.0 / stepFrequency, xyTipPosTarget);

    } // if true
    else {
      // Normal control
     // xyTipPlacementInControl_main(xyTipPos, xyTipPosTarget, velStep);
      xyTipPlacementInControl_main(xyTipPos, xyTipPosTarget);

    } // else
    std::cout << xyTipPosTarget[0] << " " << xyTipPos[0] << " ";

    targetLegLength[0] = leg_0 + std::min(0.0,retractionLength);
    targetLegLength[1] = leg_0 - std::max(0.0,retractionLength);

    if (retractionLength < 0) {
      prevRightStandControl = true; // temporary 

      controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]);
      controlAngle[1] = -controlAngle[0];

      controlAngle[2] = asin(xyTipPos[0]/targetLegLength[1]);
      controlAngle[3] = -controlAngle[2]*1.0;
    } // if true
    else{   
      prevRightStandControl = false; // temporary 

      controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]);
      controlAngle[3] = -controlAngle[2];

      controlAngle[0] = asin(xyTipPos[0]/targetLegLength[0]);
      controlAngle[1] = -controlAngle[0]*1.0;
    } // else

    controlAngle[0] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[1] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[2] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
    controlAngle[3] += acos((targetLegLength[1]/2)/(0.26)) - PI/6;

    // Apply limit to the joint angle
    for(unsigned int i=0;i<4;++i){
      controlAngle[i] = std::max(-maxAngle,std::min(controlAngle[i], maxAngle));
    }
 
  }

  /*----------Update the joint angles below------------*/
  // left_abad_joint
  commands[0] = 0;

  // right_abad_joint
  commands[1] = 0;

  // left spring rear joint
  commands[2] = (stop)? angle_0 : controlAngle[0];

  // left spring front joint
  commands[3] = (stop)? angle_0 : controlAngle[1];

  // left rear joint
  commands[4] = 0;

  // left front joint
  commands[5] = 0;

  // right spring rear joint
  commands[6] = (stop)? angle_0 : controlAngle[2];

  // right spring front joint
  commands[7] = (stop)? angle_0 : controlAngle[3];

  // right rear joint
  commands[8] = 0;

  // right front joint
  commands[9] = 0;
  
  return;
} // update_control

bool cummulativeTimeUpdate(std::deque<uint64_t>& time_ms, 
                           const unsigned int storedNum,
                           const uint64_t curTime_msec){
  bool timeUpdated = false;

  if (time_ms.size() < storedNum) {
    // Fill up the vectors of size (storedNum)
    if (time_ms.empty()){
      // Start filling up vectors (very first time only)
      time_ms.push_back(curTime_msec);
      timeUpdated = true;
    } else {
      if(time_ms.back() != curTime_msec) {
        // Push back vector if timestamp is different
        time_ms.push_back(curTime_msec);
        timeUpdated = true;
      } 
    } 
  } else {
    // Start estimating velocity and Allowing pop front when vectors have size (storedNum)
    if(time_ms.back() != curTime_msec) {
      // Pop front and push back when timestamp is different
      time_ms.pop_front();
      time_ms.push_back(curTime_msec);
      timeUpdated = true;
    } 
  }
  return timeUpdated;
}

void getVel(std::vector<double>& cumuVel, 
                 std::deque< std::vector<double> >& cummulative, 
                 const bool timeUpdated,
                 const unsigned int storedNum,
                 const std::vector<double>& newMeasure,
                 const std::deque<uint64_t>& time_ms){
  if (cummulative.size() < storedNum) {
    // Fill up the vectors of size (storedNum)
    // Replace last element with the new joint position when timestamp is the same as previous one
    if(!timeUpdated) {    
      cummulative.pop_back();
    } 
    cummulative.push_back(newMeasure);
  } 
  else {
    // Start estimating velocity and Allowing pop front when vectors have size (storedNum)
    // Pop front and push back when timestamp is different
    // Or replace last element with the new joint position when timestamp is the same as previous one
    if(timeUpdated) {
      cummulative.pop_front();
    } else { 
      cummulative.pop_back();
    }
    cummulative.push_back(newMeasure);

    // Estimate velocity
    for(unsigned int i=0; i<cumuVel.size(); ++i){
      // Simple numerical differentiation with (storedNum) time interval
      cumuVel[i] = (cummulative.back()[i]-cummulative.front()[i])/(time_ms.back()-time_ms[time_ms.size()-storedNum])*1000;
    }
  }
  return;
}

void linksAngleAndVel(std::array< std::array<double,3> ,2>& linksAngWithVert, 
                      std::array< std::array<double,3> ,2>& linksAngVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel,
                      const std::vector<double>& rpyImu,
                      const std::vector<double>& rpyVel)
{
  for(unsigned int i=0; i<2; ++i){
    linksAngWithVert[i][0] = ( jointPos[4+i*4]+jointPos[2+i*4]+rpyImu[1]+3.14159/6.0 ); // PI/6 comes from xacro definition of the robot // left miu
    linksAngWithVert[i][1] = ( jointPos[5+i*4]+jointPos[3+i*4]-rpyImu[1]+3.14159/6.0 ); // left niu
    linksAngWithVert[i][2] = ( jointPos[i] ); // left abad
    linksAngVel[i][0] = ( jointVel[4+i*4]+jointVel[2+i*4]+rpyVel[1] ); // right miu
    linksAngVel[i][1] = ( jointVel[5+i*4]+jointVel[3+i*4]-rpyVel[1] ); // right niu
    linksAngVel[i][2] = ( jointVel[i] ); // right abad
  }

  return;
}

void legTipForce(std::array< std::array<double,2> ,2>& tipForce, 
                 const std::array< std::array<double,3> ,2>& linksAngWithVert, 
                 const std::vector<double>& jointPos, 
                 const std::array<double,2>& springCoef)
{
  const double PI = 3.14159;

  // force on bottom links by top link (k*(link angle-spring angle)*link length)
  std::array<double,2> force_rear{ springCoef[0]*jointPos[4]/0.26, springCoef[0]*jointPos[8]/0.26 }; // left, right
  std::array<double,2> force_front{ springCoef[1]*jointPos[5]/0.2, springCoef[1]*jointPos[9]/0.2 }; // left, right

  // Angle theta on link AB in leg tip force calculation
  std::array<double,2> theta { PI/2-linksAngWithVert[0][0]-linksAngWithVert[0][1], PI/2-linksAngWithVert[1][0]-linksAngWithVert[1][1] }; // left, right
  
  // Calculate leg tip force in link AB frame
  std::array< std::array<double,2> ,2> linkForce; // left (x,y), right (x,y)
  for(unsigned int i=0; i<force_rear.size(); ++i){
    linkForce[i] = { force_rear[i]*cos(theta[i]),
                    -force_rear[i]*sin(theta[i])+force_front[i]};
  }

  // Calculate leg tip force in world frame
  for(unsigned int i=0; i<theta.size(); ++i){
    double force = sqrt(linkForce[i][0]*linkForce[i][0] + linkForce[i][1]*linkForce[i][1]);
    double alpha = linksAngWithVert[i][1] - atan(linkForce[i][1]/linkForce[i][0]);
    tipForce[i][0] = -force*sin(alpha);
    tipForce[i][1] = force*cos(alpha);
  }

  return;
}

void singleSupportSwitch(bool& swang,
                         unsigned int& walkingState, 
                         double phaseSwitchConst){
  if(swang){ 
    if(phaseSwitchConst > 0 && phaseSwitchConst < 1){
      walkingState = (walkingState+1)%4;
      swang = false;
    }
  } else {
    swang = (phaseSwitchConst == 0.0 || phaseSwitchConst == 1.0);
  }

  return;
}

void doubleSupportSwitch(unsigned int& walkingState, 
                         bool rightToSwing, 
                         double phaseSwitchConst){
  if(rightToSwing){
    if(phaseSwitchConst > 0.6){
      walkingState = (walkingState+1)%4;  
    }
  } else {
    if(phaseSwitchConst < 0.4){
      walkingState = (walkingState+1)%4;  
    }
  }

  return;
}


void rightStandForLinearVel(bool& rightStand, 
                            const std::array< std::array<double,2> ,2>& tipForce){
  double startSwingForceThreshold = 0.0;
  if(rightStand){
    rightStand = ((tipForce[1][1] < startSwingForceThreshold) + 1)%2;
  } else {
    rightStand = tipForce[0][1] < startSwingForceThreshold;
  }
  writeToFile( std::to_string(tipForce[0][1]) + " " + std::to_string(tipForce[1][1]) + " ");

  return;
}

void getLinearVelFromLink(std::deque< std::array<double,3> >& linearVelFromLink, 
                           const bool& rightStand,
                           const std::array< std::array<double,3> ,2>& linksAngWithVert,
                           const std::array< std::array<double,3> ,2>& linksAngVel)
{
  std::array<double,3> curLinearVelFromLink;
  
  const float r0 = 0.26;
  const float rb = 0.08;

  // Calculate robot body linear velocity from joint position and velocity
  // (refer to the calculation in "velocity kinematic calculation.pptx")
  const double miu = linksAngWithVert[rightStand][0];
  const double niu = linksAngWithVert[rightStand][1];
  const double lambda = linksAngWithVert[rightStand][2];
  const double miu_dot = linksAngVel[rightStand][0];
  const double niu_dot = linksAngVel[rightStand][1];
  const double lambda_dot = linksAngVel[rightStand][2];

  // x-direction velocity
  curLinearVelFromLink[0] = (r0*cos(miu) *miu_dot -
                             r0*cos(niu) *niu_dot); 
  // y-direction velocity
  curLinearVelFromLink[1] = ( -boolToSgn(rightStand)*
                             (-r0*sin(miu)*sin(lambda) *miu_dot -
                              r0*sin(niu)*sin(lambda) *niu_dot +
                             (r0*(cos(miu)+cos(niu))*cos(lambda)-rb*sin(lambda)) *lambda_dot));
  // z-direction velocity   
  curLinearVelFromLink[2] = ( -r0*sin(miu)*cos(lambda) *miu_dot -
                                   r0*sin(niu)*cos(lambda) *niu_dot +
                                   (-r0*(cos(miu)+cos(niu))*sin(lambda)-rb*cos(lambda)) *lambda_dot);

  linearVelFromLink.push_back(curLinearVelFromLink);

  std::cout << linearVelFromLink.back()[0] << " " << linearVelFromLink.back()[1] << " " << linearVelFromLink.back()[2] << "" << std::endl;
  //std::cout << rightStand << std::endl;
  writeToFile( std::to_string(linearVelFromLink.back()[0]) + " ");
  
  
  if(linearVelFromLink.size() > 10){
    linearVelFromLink.pop_front();
  }

}

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
                    const double startControlTime)
{
  const double stepFrequency = 3;
  const double PI = 3.14159;
  const double maxAngle = 35.0 *(PI/180.0);
  const double leg_0 = 0.47;     //neutral length of leg
  const double leg_maxRet = 0.1; //max retraction length of the leg
  const double angle_0 = acos((leg_0/2)/(0.26)) - PI/6;
 
  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)* (time.toSec() - startControlTime));
  const double phaseSwitchParam = 2;
  phaseSwitchConst = std::min( 0.5, std::max(retractionLength/leg_maxRet* phaseSwitchParam, -0.5)) +0.5;

  std::array<double,2> targetLegLength = {0.0, 0.0}; // left, right
  std::array<double,2> targetLegAngle =  {0.0,0.0}; // left, right
  std::array<double,4> controlAngle = {0.0,0.0, 0.0,0.0}; // left(miu,niu), right(miu,niu)

  std::array<double,2> desired_vel = { 0.0,0.0 };  // x direction, y direction
  std::array<double,3> tipPlacementK = { 0.08, 0.0, 0.08 }; // kp, kd, kv
  std::array<double,2> pitchK = { 13.3, 1.33 }; // kp, kv
  double springAveK = 145.0;

  std::array<double,2> controlPitchStep = { 0.005, 0.005 }; // stance, swing step

  // Get the current actual hip pitch angles from IMU
  float pitchToFront = rpyImu[1];

  if(!stop){
    if((retractionLength < 0) != prevRightStandControl){
      targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromLink, desired_vel, tipPlacementK,false);

      targetPitch[(prevRightStandControl+1)%2] = 0; // previously swing, current stand phase
      targetPitch[prevRightStandControl] = pitchToFront; // previously stand, current swing

      prevRightStandControl = (prevRightStandControl+1)%2;
      
    } // if true
    else {
      if(loop_count_%100 == 0){
        targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromLink, desired_vel, tipPlacementK,false);
      }
      xyTipPlacementInControl_main(xyTipPos, xyTipPosTarget);

      targetPitch[prevRightStandControl] = 0; //-pitchToFront; // current stance phase
      targetPitch[(prevRightStandControl+1)%2] = pitchToFront; // current swing phase
      for(unsigned int i=0; i<2; ++i){
        double pitchErr = controlPitch[i] - targetPitch[i];
        if(pitchErr>0){
          controlPitch[i] -= std::min(controlPitchStep[(prevRightStandControl+i)%2], pitchErr);
        }
        else if(pitchErr<0){
          controlPitch[i] += std::min(controlPitchStep[(prevRightStandControl+i)%2], -pitchErr);
        }
        //writeToFile( std::to_string(controlPitch[i]) + " ");
      }

    } // else

    writeToFile( std::to_string(xyTipPosTarget[0]) + " ");
    writeToFile( std::to_string(xyTipPos[0]) + " ");
  
    if (retractionLength < 0){
      targetLegLength[0] = leg_0 + retractionLength;
      targetLegLength[1] = leg_0;
    } // if true
    else {
      targetLegLength[0] = leg_0;
      targetLegLength[1] = leg_0 - retractionLength;
    } // else

    switch(walkingState) {
      case 0: // Double support (right = flight, left = stance)
        controlAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0][0] - linksAngWithVert[0][1] );
        controlAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[1][0] - linksAngWithVert[1][1] );
        doubleSupportSwitch(walkingState,true, phaseSwitchConst);
        break;

      case 1: // Left leg single support (right = flight, left = stance)
        controlAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0][0] - linksAngWithVert[0][1] );
        controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1];
        singleSupportSwitch(swang, walkingState, phaseSwitchConst);
        break;

      case 2: // Double support (right = stance, left = flight)
        controlAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0][0] - linksAngWithVert[0][1] );
        controlAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[1][0] - linksAngWithVert[1][1] );
        doubleSupportSwitch(walkingState,false, phaseSwitchConst);
        break;

      case 3: // Single support (right = stance, left = flight)
        controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0];
        std::cout << xyTipPos[0] << " " << controlPitch[0] << std::endl;
        controlAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[1][0] - linksAngWithVert[1][1] );
        singleSupportSwitch(swang, walkingState, phaseSwitchConst);
        break;
    }

    //std::cout << walkingState << std::endl;

    controlAngle[1] = -controlAngle[0];
    controlAngle[3] = -controlAngle[2];    

/*
    controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0];
    controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1];

    controlStandAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0][0] - linksAngWithVert[0][1] );
    controlStandAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[1][0] - linksAngWithVert[1][1] );

    //controlAngle[0] = controlAngle[0]*(1-phaseSwitchConst) + controlStandAngle[0]*phaseSwitchConst;
    //controlAngle[1] = -controlAngle[0];
    //controlAngle[2] = controlAngle[2]*phaseSwitchConst + controlStandAngle[2]*(1-phaseSwitchConst);
    //controlAngle[3] = -controlAngle[2];

    controlStandAngle[0] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlStandAngle[1] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlStandAngle[2] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
    controlStandAngle[3] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
*/

    controlAngle[0] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[1] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[2] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
    controlAngle[3] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 

    // Apply limit to the joint angle
    for(unsigned int i=0;i<4;++i){
      controlAngle[i] = std::max(-maxAngle,std::min(controlAngle[i], maxAngle));
      controlStandAngle[i] = std::max(-maxAngle,std::min(controlStandAngle[i], maxAngle));
      writeToFile( std::to_string(controlAngle[i]) + " ");
    }
    writeToFile("\n");


  }

  /*----------Update the joint angles below------------*/
  // left_abad_joint
  commands[0] = 0;

  // right_abad_joint
  commands[1] = 0;

  // left spring rear joint
  commands[2] = (stop)? angle_0 : controlAngle[0];
  //commands[2] = -15*PI/180.0;

  // left spring front joint
  commands[3] = (stop)? angle_0 : controlAngle[1];
  //commands[3] = -15*PI/180.0;

  // left rear joint
  commands[4] = 0;

  // left front joint
  commands[5] = 0;

  // right spring rear joint
  commands[6] = (stop)? angle_0 : controlAngle[2];
  //commands[6] = 20*PI/180.0;

  // right spring front joint
  commands[7] = (stop)? angle_0 : controlAngle[3];
  //commands[7] = 20*PI/180.0;

  // right rear joint
  commands[8] = 0;

  // right front joint
  commands[9] = 0;

  return;
} // update_control_free

void writeToFile(std::string msg){
  std::ofstream myfile("test.txt", std::ios::out | std::ios::app);
  if(myfile.is_open()){
    myfile << msg;
  }
  else std::cout << "Unable to open file" << std::endl;

  return;
}

