#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include <ros/console.h>

void rightStandForControl(bool& rightStandControl, 
                          bool& dropping, 
                          bool& startTouch, 
                          const std::vector<double>& tipForce){
  /*unsigned int forceIndex = (-rightStandControl*2)+3;
  if(!dropping){
    dropping = (tipForce[forceIndex] > 75);
  } // if true
  else {
    if(!startTouch){
      startTouch = (tipForce[forceIndex] < 60);
    } // if true
    else {
      if(tipForce[forceIndex] > 58){
        rightStandControl = !rightStandControl;
        dropping = false;
        startTouch = false;
      } // if 
    } // else
  } // else*/
  if(tipForce[1] > 30){
    //std::cout << "Left tip force > 30" << std::endl;
  }
  else if(tipForce[1] > 25){
    //std::cout << "Left tip force > 25" << std::endl;
  }

  //std::cout << rightStandControl << std::endl;

  return;
} // rightStandForControl

void targetXYTipPlacement(std::vector<double>& xyTipPosTarget,
                          std::queue< std::vector<double> >& aveLinearVel,
                          std::deque< std::vector<double> >& linearVelFromJoint,
                          const double* desired_vel,
                          const double* tipPlacementK){
  /*
  Find the average velocity during a step -> fill in the aveLinearVel queue of size 2

  Obtain average velocity of previous step and a step before previous, and

  Calculate the target tip placement along x and y direction and make sure it is within the preset range (x direction only)
  
  (test was done in x direction only)
  */
  //const double maxX = 0.15;
  const double maxX = 0.15;
  double diffVel[2] = {0.0, 0.0};
  std::vector<double> curVel {0.0, 0.0};

  if(!linearVelFromJoint.empty()){
    for(unsigned int i=0; i<2; ++i){
      for(unsigned int j=0; j<linearVelFromJoint.size(); ++j){
        curVel[i] += linearVelFromJoint[j][i];
      } // for
      curVel[i] /= linearVelFromJoint.size();
      if(!aveLinearVel.empty()){
        diffVel[i] = curVel[i]-aveLinearVel.front()[i];
      } // if
    } // for
  } // if
  
  linearVelFromJoint.clear();
  
  for(unsigned int i=0; i<2; ++i){
    xyTipPosTarget[i] = tipPlacementK[0]*(curVel[i]-desired_vel[i]) + tipPlacementK[1]*diffVel[i] + tipPlacementK[2]*curVel[i];
  } // for

  if(aveLinearVel.size() > 1){
    aveLinearVel.pop();
  } // if
  aveLinearVel.push(curVel);
  
  //std::cout << "\nNew target pos: " << xyTipPosTarget[0] << " " << xyTipPosTarget[1]  << std::endl;

  xyTipPosTarget[0] = std::max(-maxX, std::min(maxX, xyTipPosTarget[0]));

  //std::cout << "Variable in x: " << (desired_vel[0]-curVel[0]) << " " << diffVel[0] << " " << curVel[0] << std::endl;

  //std::cout << "Prev vs new ave x vel: " << aveLinearVel.front()[0] << " " << aveLinearVel.back()[0] << std::endl;

  return;
} // targetXYTipPlacement

/*----------------------------------------------------------------------------------------------*/

void xyTipPlacementInControl_main(std::vector<double>& xyTipPos,
                                  const std::vector<double>& xyTipPosTarget){
  /*
  Move the leg tip progressively until it reach the target leg tip position when not switching leg
  */

  double maxStep = 0.0005;

  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    if(xyTipPos[i] > xyTipPosTarget[i]){  
      xyTipPos[i] -= maxStep;
    } // if true
    else if(xyTipPos[i] < xyTipPosTarget[i]){
      xyTipPos[i] += maxStep;
    } // else if true
  } // for

  //std::cout << "Tip control step: " << xyTipPos[0] << std::endl;
  
  return;
} // xyTipPlacementInControl_main

/*----------------------------------------------------------------------------------------------*/

void xyTipPlacementInControl_switch(std::vector<double>& xyTipPos,
                                    std::vector<double>& xyTipPosTarget,
                                    std::queue< std::vector<double> >& aveLinearVel,
                                    std::deque< std::vector<double> >& linearVelFromJoint,
                                    const double* desired_vel,
                                    const double* tipPlacementK){
  /*
  Call the targetXYTipPlacement function to obtain a new tip position, and

  Switch the control of tip position with xyTipPos array by multiplying by -1
  */

  targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromJoint, desired_vel, tipPlacementK);
  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    xyTipPos[i] *= -1;
  } // for
  
  return;
} // xyTipPlacementInControl_switch

/*----------------------------------------------------------------------------------------------*/

double targetLegExtension(double thisAveLinearVel,
                          const double* desired_vel,
                          const double* extK){
  /*
  Calculate the target leg extension at stance before launch
  */

  return (extK[0]*desired_vel[0] + extK[1]*(desired_vel[0]-thisAveLinearVel));
} // legExtension

/*----------------------------------------------------------------------------------------------*/

void legExtensionInControl(std::vector<double>& currentExt, 
                           bool rightStandControl, 
                           double targetExt, 
                           const double* desired_vel,
                           const std::vector<double>& linksAngWithBase){
  /*
  When the desired velocity is larger than the allowExtVel in magnitude, progressively increase leg extension at stance phase before launch (when the neutral leg is startExtAngle (currently 5 degree) away from the vertical), and

  Progressively decrease leg extension at swing phase until it reach 0 extension
  */

  bool extend = false;
  const double maxExt = 0.04;
  const double extStep = 0.0005;
  const double startExtAngle = 5 /180*3.14159;
  const double allowExtVel = 0.5;

  double neutralAngle = (linksAngWithBase[rightStandControl*3+1]-linksAngWithBase[rightStandControl*3])/2;

  if(desired_vel[0] > allowExtVel){
    if(neutralAngle < (-startExtAngle)){
      extend = true;
    } // if
  } // if true
  else if(desired_vel[0] < (-allowExtVel)){
    if(neutralAngle > startExtAngle){
      extend = true;
    } // if
  } // else if true

  if(extend){
    if(targetExt > currentExt[rightStandControl]){
      currentExt[rightStandControl] = std::min(currentExt[rightStandControl]+extStep, maxExt);
    } // if true
  } // if true

  currentExt[(rightStandControl+1)%2] = std::max(currentExt[(rightStandControl+1)%2]-extStep, 0.0);

  //std::cout << "Current extension: " << currentExt[0] << " " << currentExt[1] << std::endl;
   
  return;

} // legExtensionInControl

/*----------------------------------------------------------------------------------------------*/

// Implement the control algorithm here
void update_control(bool& prevRightStandControl,
                    double& prevVel,
                    double& targetExt,
                    double* targetPitch,
                    double* controlPitch,
                    std::vector<double>& currentExt,
                    std::vector<double>& commands, 
                    std::vector<double>& xyTipPos, 
                    std::vector<double>& xyTipPosTarget,
                    std::queue< std::vector<double> >& aveLinearVel, 
                    std::deque< std::vector<double> >& linearVelFromJoint,
                    const bool stop,
                    const bool rightStandControl, 
                    const int loop_count_,
                    const std::vector<hardware_interface::JointHandle>& joints_, 
                    const std::vector<double>& rpyImu, 
                    const std::vector<double>& rpyVel,
                    const std::vector<double>& linksAngWithBase,
                    const ros::Time& time){
  /* commands:    a vector storing the target positions for each joint
                  to be updated in this function
     joints_:     a vector storing all the joints (motors), can use 
                  joints_[i].getPosition() to get the current angle of the
                  (i-1)th joint
     rpyImu:      a vector storing the data of IMU sensor (roll, pitch, yaw)
     time:        current time (from ROS)
     
     Call get_imu_data() to get the IMU data in this function
     example control
     commands[0] = 0.3 * sin( (double)(loop_count_/500.0));
     commands[1] = -0.3 * sin( (double)(loop_count_/500.0));                  */

  const double stepFrequency = 3;
  const double PI = 3.14159;
  const double maxAngle = 2.0 *(PI/180.0);
  const double leg_0 = 0.47;     //neutral length of leg
  const double leg_maxRet = 0.1; //max retraction length of the leg
  //const double angle_0 = acos((leg_0/2)/(0.26));
  const double angle_0 = 0.0/180*PI;
 
  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)*time.toSec());

  double targetLegLength[2] = {0.0, 0.0}; // left, right
  double targetLegAngle[2] =  {0.0,0.0}; // left, right
  double controlAngle[4] = {0.0,0.0, 0.0,0.0}; // left(miu,niu), right(miu,niu)

  double f1 = 0.0; // force threshold for stance phase
  double f2 = 0.0; // force threshold for swing phase
  double ratio = 0.0; 

  double desired_vel[2] = { 0.0,0.0 };  // x direction, y direction
  double tipPlacementK[3] = { 0.18, 0.01, 0.18 }; // kp, kd, kv
  double extK[2] = { 0.01, 0.02 }; // ke1, ke2
  double pitchK[2] = { 80.0, 0.0 }; // kp, kv
  double springAveK = 150.0;

  double controlPitchStep[2] = { 0.0005, 0.0005 }; // stance, swing step

  // Get the current actual hip pitch angles from IMU
  //float pitchToFront = 50.0 *(PI/180.0); // for program testing, input artificial pitch
  float pitchToFront = rpyImu[1];

  if(!stop){
    //if(rightStandControl != prevRightStandControl){
    if((retractionLength < 0) != prevRightStandControl){
      xyTipPlacementInControl_switch(xyTipPos, xyTipPosTarget, aveLinearVel, linearVelFromJoint, desired_vel,tipPlacementK);

      targetPitch[(prevRightStandControl+1)%2] = 0; // previously swing, current stand phase
      targetPitch[prevRightStandControl] = pitchToFront*2; // previously stand, current swing
      
      //xyTipPosTarget[(prevRightStandControl+1)%2] = 0;

      //std::cout << std::endl;
      //std::cout << xyTipPosTarget[0] << " " << xyTipPosTarget[1] << std::endl;
      //targetExt = targetLegExtension(aveLinearVel.back()[0],desired_vel,extK);
      //currentExt = 0;
      //std::cout << "Target extension: " << targetExt << std::endl;
    } // if true
    else {
      if(loop_count_%100 == 0){
        targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromJoint, desired_vel, tipPlacementK);
        //std::cout << xyTipPosTarget[0] << " " << xyTipPosTarget[1] << std::endl;
      }
      xyTipPlacementInControl_main(xyTipPos, xyTipPosTarget);
      //std::cout << xyTipPos[0] << std::endl;

      targetPitch[prevRightStandControl] = 0; //-pitchToFront; // current stance phase
      targetPitch[(prevRightStandControl+1)%2] = pitchToFront*2; // current swing phase
      for(unsigned int i=0; i<2; ++i){
        if(controlPitch[i] < targetPitch[i]){
          controlPitch[i] += controlPitchStep[(prevRightStandControl+i)%2];
        }
        else if(controlPitch[i] > targetPitch[i]){
          controlPitch[i] -= controlPitchStep[(prevRightStandControl+i)%2];
        }
      }
      //legExtensionInControl(currentExt,prevRightStandControl,targetExt,desired_vel, linksAngWithBase);

    } // else
    //std::cout << "Target pitch: " << targetPitch[0] << " " << targetPitch[1] << std::endl;
    //std::cout << controlPitch[0] << " " << controlPitch[1] << std::endl;
  
    if (retractionLength < 0){
      //targetLegLength[0] = leg_0 + retractionLength - currentExt[0];
      //targetLegLength[1] = leg_0 + currentExt[1];
      targetLegLength[0] = leg_0 + retractionLength;
      targetLegLength[1] = leg_0 + targetExt;

      //std::cout << "Left tip x pos & length: " << xyTipPos[0] << " " << targetLegLength[0] << std::endl;

    } // if true
    else {
      targetLegLength[0] = leg_0 + targetExt;
      targetLegLength[1] = leg_0 - retractionLength;

    } // else

    //std::cout << prevRightStandControl << std::endl;
    //xyTipPos[0] = 0.0;

    //if(rightStandControl){
    if (retractionLength < 0) {
      prevRightStandControl = true; // temporary 
      // Swing (left)
      controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0];
      //controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]) + pitchToFront;
      controlAngle[1] = -controlAngle[0];

      // Stand (right)
      //controlAngle[2] = asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1];
      //controlAngle[3] = -controlAngle[2]*1.0 + controlPitch[1];
      //controlAngle[2] = -controlPitch[1];
      //controlAngle[3] = controlPitch[1];
      controlAngle[2] = 0.5*( (pitchK[0]*pitchToFront+pitchK[1]*rpyVel[1])/springAveK + linksAngWithBase[3] - linksAngWithBase[4] );
      controlAngle[3] = -controlAngle[2];
    } // if true
    else{   
      prevRightStandControl = false; // temporary 
     
      // Swing (right)
      controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1];
      //controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]) - pitchToFront;
      controlAngle[3] = -controlAngle[2];

      // Stand (left)
      //controlAngle[0] = asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0];
      //controlAngle[1] = -controlAngle[0]*1.0 + controlPitch[0];
      //controlAngle[0] = -controlPitch[0];
      //controlAngle[1] = controlPitch[0];
      controlAngle[0] = 0.5*( (pitchK[0]*pitchToFront+pitchK[1]*rpyVel[1])/springAveK + linksAngWithBase[0] - linksAngWithBase[1] );
      controlAngle[1] = -controlAngle[0];
    } // else
    std::cout << prevRightStandControl*10 << std::endl;
    //std::cout << "Left leg now: " << (curLeftLegAngle/PI*180) << " Left leg to be: " << (targetLeftLegAngle/PI*180) << "\n";

    controlAngle[0] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[1] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[2] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
    controlAngle[3] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 

    //std::cout << controlAngle[0]/PI*180 << " " << controlAngle[1]/PI*180 
    //       << " " << controlAngle[2]/PI*180 << " " << controlAngle[3]/PI*180 << std::endl;
    //std::cout << controlAngle[0] << " " << controlAngle[1] 
    //      << " " << controlAngle[2] << " " << controlAngle[3] << std::endl;
    //std::cout << (prevRightStandControl? "Right": "Left") << controlAngle[prevRightStandControl*2] << " " << controlAngle[prevRightStandControl*2+1] << std::endl;
    //std::cout << std::endl;
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
      // Use finte backward numerical differentiation (with different timesteps)
      //jointVel[i] = (3*(jointPosCummulative[2][i]-jointPosCummulative[1][i])/(2*(time_ms[2]-time_ms[1])) - ((jointPosCummulative[1][i]-jointPosCummulative[0][i])/(2*(time_ms[1]-time_ms[0]))))*1000;

      // Use simple numerical differentiation
      //jointVel[i] = (jointPosCummulative[2][i]-jointPosCummulative[1][i])/(time_ms[2]-time_ms[1])*1000; 

      // Simple numerical differentiation with (storedNum) time interval
      cumuVel[i] = (cummulative.back()[i]-cummulative.front()[i])/(time_ms.back()-time_ms[time_ms.size()-storedNum])*1000;
    }
  }
  return;
}

void linksAngleAndVel(std::vector<double>& linksAngWithBase, 
                      std::vector<double>& linksVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel)
{
  linksAngWithBase.clear();
  linksVel.clear();
  linksAngWithBase.reserve(3*2);
  linksVel.reserve(3*2);
  for(unsigned int i=0; i<2; ++i){
    linksAngWithBase.push_back( jointPos[4+i*4]+jointPos[2+i*4]+0.196 ); // 0.196 comes from xacro definition of the robot
    linksAngWithBase.push_back( jointPos[5+i*4]+jointPos[3+i*4]+0.196 );
    linksAngWithBase.push_back( jointPos[0+i] );
    linksVel.push_back( jointVel[4+i*4]+jointVel[2+i*4] );
    linksVel.push_back( jointVel[5+i*4]+jointVel[3+i*4] );
    linksVel.push_back( jointVel[0+i] );

    //std::cout << linksAngWithBase[i*3]/3.14159*180 << " " << linksAngWithBase[i*3+1]/3.14159*180 << " " << linksAngWithBase[i*3+2]/3.14159*180 << " ";
  }
  //std::cout << std::endl;
  

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

  std::cout << tipForce[0] << " " << tipForce[1] << " ";
  std::cout << tipForce[2] << " " << tipForce[3] << " ";

  return;
}

void rightStandForLinearVel(bool& rightStand, const std::vector<double>& tipForce){
  /* 
  If right leg is previously standing, 
  then if the current right leg tip force is smaller than (startSwingForceThreshold), 
  it is considered that right leg leaves the ground now. 
  The left leg is then considered as standing.
  (same happens when left leg is previously standing)
  */
  double startSwingForceThreshold = 5.0;
  if(rightStand){
    rightStand = ((tipForce[3] < startSwingForceThreshold) + 1)%2;
  } else {
    rightStand = tipForce[1] < startSwingForceThreshold;
  }
  
  //std::cout << "Right Stand: " << rightStand << std::endl;

  return;
}

void getLinearVelFromJoint(std::deque< std::vector<double> >& linearVelFromJoint, 
                           const bool& rightStand,
                           const std::vector<double>& jointVel, 
                           const std::vector<double>& jointPos,
                           const std::vector<double>& rpyVel,
                           const std::vector<double>& rpyImu)
{
  std::vector<double> curLinearVelFromJoint;
  curLinearVelFromJoint.reserve(3);
  const float r0 = 0.26;
  const float rb = 0.08;

  // Calculate robot body linear velocity from joint position and velocity
  // (refer to the calculation in "velocity kinematic calculation.pptx")
  double miu = jointPos[4+rightStand*4]+jointPos[2+rightStand*4] + rpyImu[1];
  double niu = jointPos[5+rightStand*4]+jointPos[3+rightStand*4] - rpyImu[1];
  double lambda = jointPos[0+rightStand];
  double miu_dot = jointVel[4+rightStand*4]+jointVel[2+rightStand*4] + rpyVel[1];
  double niu_dot = jointVel[5+rightStand*4]+jointVel[3+rightStand*4] - rpyVel[1];
  double lambda_dot = jointVel[0+rightStand];

  // x-direction velocity
  curLinearVelFromJoint.push_back(r0*cos(miu) *miu_dot -
                                  r0*cos(niu) *niu_dot); 
  // y-direction velocity
  curLinearVelFromJoint.push_back( -rightStand*
                                   (-r0*sin(miu)*sin(lambda) *miu_dot -
                                   r0*sin(niu)*sin(lambda) *niu_dot +
                                   (r0*(cos(miu)+cos(niu))*cos(lambda)-rb*sin(lambda)) *lambda_dot));
  // z-direction velocity   
  curLinearVelFromJoint.push_back( -r0*sin(miu)*cos(lambda) *miu_dot -
                                   r0*sin(niu)*cos(lambda) *niu_dot +
                                   (-r0*(cos(miu)+cos(niu))*sin(lambda)-rb*cos(lambda)) *lambda_dot);

  linearVelFromJoint.push_back(curLinearVelFromJoint);
  
  //std::cout << linearVelFromJoint.size() << std::endl;

  //std::cout << linearVelFromJoint.back()[0] << " " << linearVelFromJoint.back()[1] << " " << linearVelFromJoint.back()[2] << "" << std::endl;

}


// Get the IMU data
IMUData get_imu_data(){
  IMUData imu;

  // get acceleration data and put in acc[3] array, and
  // get angular velocity data and put in omega[3] array


  return imu;
}


