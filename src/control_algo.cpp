#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include <ros/console.h>

void targetXYTipPlacement(std::vector<double>& xyTipPosTarget,
                          std::queue< std::vector<double> >& aveLinearVel,
                          std::deque< std::vector<double> >& linearVelFromLink,
                          const double* desired_vel,
                          const double* tipPlacementK){
  /*
  Find the average velocity during a step -> fill in the aveLinearVel queue of size 2

  Obtain average velocity of previous step and a step before previous, and

  Calculate the target tip placement along x and y direction and make sure it is within the preset range (x direction only)
  
  (test was done in x direction only)
  */
  const double maxX = 0.16;
  double diffVel[2] = {0.0, 0.0};
  std::vector<double> curVel {0.0, 0.0};

  if(!linearVelFromLink.empty()){
    for(unsigned int i=0; i<2; ++i){
      for(unsigned int j=0; j<linearVelFromLink.size(); ++j){
        curVel[i] += linearVelFromLink[j][i];
      } // for
      curVel[i] /= linearVelFromLink.size();
      if(!aveLinearVel.empty()){
        diffVel[i] = curVel[i]-aveLinearVel.front()[i];
      } // if
    } // for
  } // if
  
  linearVelFromLink.clear();
  
  for(unsigned int i=0; i<2; ++i){
    xyTipPosTarget[i] = tipPlacementK[0]*(desired_vel[i]-curVel[i]) + tipPlacementK[1]*diffVel[i] + tipPlacementK[2]*curVel[i] - 0.012;
  } // for

  if(aveLinearVel.size() > 1){
    aveLinearVel.pop();
  } // if
  aveLinearVel.push(curVel);

  xyTipPosTarget[0] = std::max(-maxX, std::min(maxX, xyTipPosTarget[0]));

  return;
} // targetXYTipPlacement

/*----------------------------------------------------------------------------------------------*/

void xyTipPlacementInControl_main(std::vector<double>& xyTipPos,
                                  const std::vector<double>& xyTipPosTarget){
  /*
  Move the leg tip progressively until it reach the target leg tip position when not switching leg
  */

  double maxStep = 0.0005;  // fix pitch
  //double maxStep = 0.001;  // free pitch

  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    double err = xyTipPos[i] - xyTipPosTarget[i];
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

void xyTipPlacementInControl_switch(std::vector<double>& xyTipPos,
                                    std::vector<double>& xyTipPosTarget,
                                    std::queue< std::vector<double> >& aveLinearVel,
                                    std::deque< std::vector<double> >& linearVelFromLink,
                                    const double* desired_vel,
                                    const double* tipPlacementK){
  /*
  Call the targetXYTipPlacement function to obtain a new tip position, and

  Switch the control of tip position with xyTipPos array by multiplying by -1
  */

  targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromLink, desired_vel, tipPlacementK);
  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    xyTipPos[i] *= -1;
  } // for
  
  return;
} // xyTipPlacementInControl_switch

/*----------------------------------------------------------------------------------------------*/

// Implement the control algorithm here
void update_control(bool& prevRightStandControl,
                    std::vector<double>& commands, 
                    std::vector<double>& xyTipPos, 
                    std::vector<double>& xyTipPosTarget,
                    std::queue< std::vector<double> >& aveLinearVel, 
                    std::deque< std::vector<double> >& linearVelFromLink,
                    const bool stop,
                    const std::vector<double>& rpyImu, 
                    const ros::Time& time,
                    const double startControlTime)
{
  const double stepFrequency = 3;
  const double PI = 3.14159;
  const double maxAngle = 2.0 *(PI/180.0);
  const double leg_0 = 0.47;     //neutral length of leg
  const double leg_maxRet = 0.1; //max retraction length of the leg
  const double angle_0 = acos((leg_0/2)/(0.26)) - PI/6;

  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)*(time.toSec() - startControlTime));

  double targetLegLength[2] = {0.0, 0.0}; // left, right
  double targetLegAngle[2] =  {0.0,0.0}; // left, right
  double controlAngle[4] = {0.0,0.0, 0.0,0.0}; // left(miu,niu), right(miu,niu)

  double f1 = 0.0; // force threshold for stance phase
  double f2 = 0.0; // force threshold for swing phase
  double ratio = 0.0; 

  double desired_vel[2] = { 0.5,0.0 };  // x direction, y direction
  double tipPlacementK[3] = { 0.18, 0.01, 0.18 }; // kp, kd, kv

  // Get the current actual hip pitch angles from IMU
  float pitchToFront = rpyImu[1];

  if(!stop){
    if((retractionLength < 0) != prevRightStandControl){
      xyTipPlacementInControl_switch(xyTipPos, xyTipPosTarget, aveLinearVel, linearVelFromLink, desired_vel,tipPlacementK);

    } // if true
    else {
      xyTipPlacementInControl_main(xyTipPos, xyTipPosTarget);

    } // else

    if (retractionLength < 0) {
      prevRightStandControl = true; // temporary 
      targetLegLength[0] = leg_0 + retractionLength;
      targetLegLength[1] = leg_0;

      controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]);
      controlAngle[1] = -controlAngle[0];

      controlAngle[2] = asin(xyTipPos[0]/targetLegLength[1]);
      controlAngle[3] = -controlAngle[2]*1.0;
    } // if true
    else{   
      prevRightStandControl = false; // temporary 
      targetLegLength[0] = leg_0;
      targetLegLength[1] = leg_0 - retractionLength;

      controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]);
      controlAngle[3] = -controlAngle[2];

      controlAngle[0] = asin(xyTipPos[0]/targetLegLength[0]);
      controlAngle[1] = -controlAngle[0]*1.0;
    } // else

    controlAngle[0] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[1] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[2] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
    controlAngle[3] += acos((targetLegLength[1]/2)/(0.26)) - PI/6;
 
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

void linksAngleAndVel(std::vector<double>& linksAngWithVert, 
                      std::vector<double>& linksAngVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel)
{
  linksAngWithVert.clear();
  linksAngVel.clear();
  linksAngWithVert.reserve(3*2);
  linksAngVel.reserve(3*2);
  for(unsigned int i=0; i<2; ++i){
    linksAngWithVert.push_back( jointPos[4+i*4]+jointPos[2+i*4]+3.14159/6.0 ); // PI/6 comes from xacro definition of the robot // left miu
    linksAngWithVert.push_back( jointPos[5+i*4]+jointPos[3+i*4]+3.14159/6.0 ); // left niu
    linksAngWithVert.push_back( jointPos[i] ); // left abad
    linksAngVel.push_back( jointVel[4+i*4]+jointVel[2+i*4] ); // right miu
    linksAngVel.push_back( jointVel[5+i*4]+jointVel[3+i*4] ); // right niu
    linksAngVel.push_back( jointVel[i] ); // right abad
  }

  return;
}

void legTipForce(std::vector<double>& tipForce, 
                 const std::vector<double>& linksAngWithBase, 
                 const std::vector<double>& jointPos, 
                 const std::vector<double>& springCoef)
{
  const double PI = 3.14159;

  // force on bottom links by top link (k*(link angle-spring angle)*link length)
  std::vector<double> force_rear{ springCoef[0]*jointPos[4]/0.26, springCoef[0]*jointPos[8]/0.26 }; // left, right
  std::vector<double> force_front{ springCoef[1]*jointPos[5]/0.2, springCoef[1]*jointPos[9]/0.2 }; // left, right

  // Angle theta on link AB in leg tip force calculation
  std::vector<double> theta { PI/2-linksAngWithBase[0]-linksAngWithBase[1], PI/2-linksAngWithBase[3]-linksAngWithBase[4] }; // left, right
  
  // Calculate leg tip force in link AB frame
  std::vector<double> linkForce; // left (x,y), right (x,y)
  for(unsigned int i=0; i<force_rear.size(); ++i){
    linkForce.push_back( force_rear[i]*cos(theta[i]) );
    linkForce.push_back( -force_rear[i]*sin(theta[i])+force_front[i] );
  }

  // Calculate leg tip force in world frame
  tipForce.clear();
  tipForce.reserve(theta.size()*2);
  for(unsigned int i=0; i<theta.size(); ++i){
    double force = sqrt(linkForce[i*2]*linkForce[i*2] + linkForce[i*2+1]*linkForce[i*2+1]);
    double alpha = linksAngWithBase[i*3+1] - atan(linkForce[i*2+1]/linkForce[i*2]);
    tipForce.push_back(-force*sin(alpha));
    tipForce.push_back(force*cos(alpha));
  }

  return;
}

void rightStandForLinearVel(bool& rightStand, const std::vector<double>& tipForce){
  double startSwingForceThreshold = 5.0;
  if(rightStand){
    rightStand = ((tipForce[3] < startSwingForceThreshold) + 1)%2;
  } else {
    rightStand = tipForce[1] < startSwingForceThreshold;
  }

  return;
}

void getLinearVelFromLink(std::deque< std::vector<double> >& linearVelFromLink, 
                           const bool& rightStand,
                           const std::vector<double>& linksAngWithVert,
                           const std::vector<double>& linksAngVel)
{
  std::vector<double> curLinearVelFromLink;
  curLinearVelFromLink.reserve(3);
  const float r0 = 0.26;
  const float rb = 0.08;

  // Calculate robot body linear velocity from joint position and velocity
  // (refer to the calculation in "velocity kinematic calculation.pptx")
  const double miu = linksAngWithVert[rightStand*3];
  const double niu = linksAngWithVert[rightStand*3+1];
  const double lambda = linksAngWithVert[rightStand*3+2];
  const double miu_dot = linksAngVel[rightStand*3];
  const double niu_dot = linksAngVel[rightStand*3+1];
  const double lambda_dot = linksAngVel[rightStand*3+2];

  // x-direction velocity
  curLinearVelFromLink.push_back(r0*cos(miu) *miu_dot -
                                  r0*cos(niu) *niu_dot); 
  // y-direction velocity
  curLinearVelFromLink.push_back( -rightStand*
                                   (-r0*sin(miu)*sin(lambda) *miu_dot -
                                   r0*sin(niu)*sin(lambda) *niu_dot +
                                   (r0*(cos(miu)+cos(niu))*cos(lambda)-rb*sin(lambda)) *lambda_dot));
  // z-direction velocity   
  curLinearVelFromLink.push_back( -r0*sin(miu)*cos(lambda) *miu_dot -
                                   r0*sin(niu)*cos(lambda) *niu_dot +
                                   (-r0*(cos(miu)+cos(niu))*sin(lambda)-rb*cos(lambda)) *lambda_dot);

  linearVelFromLink.push_back(curLinearVelFromLink);
  
  if(linearVelFromLink.size() > 10){
    linearVelFromLink.pop_front();
  }

}



