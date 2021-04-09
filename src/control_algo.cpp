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
  const double maxX = 0.16;
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

  //linearVelFromJoint.clear();

  //std::cout << curVel[0] << std::endl;
  
  for(unsigned int i=0; i<2; ++i){
    xyTipPosTarget[i] = tipPlacementK[0]*(curVel[i]-desired_vel[i]) + tipPlacementK[1]*diffVel[i] + tipPlacementK[2]*curVel[i] - 0.012;
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

  double maxStep = 0.001;
  //double maxStep = 0.00001;

  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    double err = xyTipPos[i] - xyTipPosTarget[i];
    if(err > 0){  
      xyTipPos[i] -= std::min(err,maxStep);
    } // if true
    else if(err < 0){
      xyTipPos[i] += std::min(-err,maxStep);
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
                           const std::vector<double>& linksAngWithVert){
  /*
  When the desired velocity is larger than the allowExtVel in magnitude, progressively increase leg extension at stance phase before launch (when the neutral leg is startExtAngle (currently 5 degree) away from the vertical), and

  Progressively decrease leg extension at swing phase until it reach 0 extension
  */

  bool extend = false;
  const double maxExt = 0.04;
  const double extStep = 0.0005;
  const double startExtAngle = 5 /180*3.14159;
  const double allowExtVel = 0.5;

  double neutralAngle = (linksAngWithVert[rightStandControl*3+1]-linksAngWithVert[rightStandControl*3])/2;

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
                    bool& swang,
                    unsigned int& walkingState,
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
                    const std::vector<double>& linksAngWithVert,
                    const std::vector<double>& jointPos,
                    const std::deque<double>& leftTipYForce,
                    const std::deque<double>& rightTipYForce,
                    const ros::Time& time,
                    const double startControlTime){
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
  const double angle_0 = acos((leg_0/2)/(0.26)) - PI/6;
  //const double angle_0 = 0.0/180*PI;
 
  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)* (time.toSec() - startControlTime));
  const double phaseSwitchParam = 2;
  const double phaseSwitchConst = std::min( 0.5, std::max(retractionLength/leg_maxRet* phaseSwitchParam, -0.5)) +0.5;
  //std::cout << phaseSwitchConst << " ";
  writeToFile( std::to_string(phaseSwitchConst) + " ");

  double targetLegLength[2] = {0.0, 0.0}; // left, right
  double targetLegAngle[2] =  {0.0,0.0}; // left, right
  double controlAngle[4] = {0.0,0.0, 0.0,0.0}; // left(miu,niu), right(miu,niu)
  double controlStandAngle[4] = { 0.0,0.0,0.0,0.0 }; // left(miu,niu), right(miu,niu)

  double f1 = 0.0; // force threshold for stance phase
  double f2 = 0.0; // force threshold for swing phase
  double ratio = 0.0; 

  double desired_vel[2] = { 0.0,0.0 };  // x direction, y direction
  double tipPlacementK[3] = { 0.18, 0.0, 0.18 }; // kp, kd, kv
  double extK[2] = { 0.01, 0.02 }; // ke1, ke2
  double pitchK[2] = { 13.3, 1.33 }; // kp, kv
  double springAveK = 145.0;

  double controlPitchStep[2] = { 0.005, 0.005 }; // stance, swing step

  // Get the current actual hip pitch angles from IMU
  //float pitchToFront = 50.0 *(PI/180.0); // for program testing, input artificial pitch
  float pitchToFront = rpyImu[1];

  if(!stop){
    //if(rightStandControl != prevRightStandControl){
    if((retractionLength < 0) != prevRightStandControl){
      //xyTipPlacementInControl_switch(xyTipPos, xyTipPosTarget, aveLinearVel, linearVelFromJoint, desired_vel,tipPlacementK);
      targetXYTipPlacement(xyTipPosTarget, aveLinearVel, linearVelFromJoint, desired_vel, tipPlacementK);

      targetPitch[(prevRightStandControl+1)%2] = 0; // previously swing, current stand phase
      targetPitch[prevRightStandControl] = pitchToFront; // previously stand, current swing

      prevRightStandControl = (prevRightStandControl+1)%2;
      
      //xyTipPosTarget[(prevRightStandControl+1)%2] = 0;

      //std::cout << std::endl;
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
      targetPitch[(prevRightStandControl+1)%2] = pitchToFront; // current swing phase
      for(unsigned int i=0; i<2; ++i){
        double pitchErr = controlPitch[i] - targetPitch[i];
        if(pitchErr>0){
          controlPitch[i] -= std::min(controlPitchStep[(prevRightStandControl+i)%2], pitchErr);
        }
        else if(pitchErr<0){
          controlPitch[i] += std::min(controlPitchStep[(prevRightStandControl+i)%2], -pitchErr);
        }
      }
      //legExtensionInControl(currentExt,prevRightStandControl,targetExt,desired_vel, linksAngWithVert);

    } // else
    //std::cout << "Target pitch: " << targetPitch[0] << " " << targetPitch[1] << std::endl;
    //std::cout << controlPitch[0] << " " << controlPitch[1] << std::endl;

    //std::cout << xyTipPosTarget[0] << " " << xyTipPosTarget[1] << std::endl;
  
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
    //std::cout << targetLegLength[0] << " " << targetLegLength[1] << std::endl;

    //xyTipPos[0] = 0.0;

    //prevRightStandControl = (prevRightStandControl? phaseSwitchConst!=1.0 : phaseSwitchConst==0.0);

    switch(walkingState) {
      case 0: // Double support (right = flight, left = stance)
        controlAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0] - linksAngWithVert[1] );
        controlAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[3] - linksAngWithVert[4] );
        doubleSupportSwitch(walkingState,true, phaseSwitchConst, jointPos);
        break;

      case 1: // Left leg single support (right = flight, left = stance)
        controlAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0] - linksAngWithVert[1] );
        controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1];
        singleSupportSwitch(swang, walkingState, phaseSwitchConst, rightTipYForce);
        break;

      case 2: // Double support (right = stance, left = flight)
        controlAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0] - linksAngWithVert[1] );
        controlAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[3] - linksAngWithVert[4] );
        doubleSupportSwitch(walkingState,false, phaseSwitchConst, jointPos);
        break;

      case 3: // Single support (right = stance, left = flight)
        controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0];
        controlAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[3] - linksAngWithVert[4] );
        singleSupportSwitch(swang, walkingState, phaseSwitchConst, leftTipYForce);
        break;

    }
    controlAngle[1] = -controlAngle[0];
    controlAngle[3] = -controlAngle[2];    

/*
    controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0];
    controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1];

    //std::cout << controlPitch[0] << " " << controlPitch[1] << " " << rpyImu[1] << std::endl;
    //std::cout << controlAngle[0]/PI*180 << " " << controlAngle[2]/PI*180 << " ";


    controlStandAngle[0] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0] - linksAngWithVert[1] );
    controlStandAngle[2] = 0.5*( (pitchK[0]*rpyImu[1]+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[3] - linksAngWithVert[4] );

    //std::cout << controlStandAngle[0]/PI*180 << " " << controlStandAngle[2]/PI*180 << " ";

    controlAngle[0] = controlAngle[0]*(1-phaseSwitchConst) + controlStandAngle[0]*phaseSwitchConst;
    controlAngle[1] = -controlAngle[0];
    controlAngle[2] = controlAngle[2]*phaseSwitchConst + controlStandAngle[2]*(1-phaseSwitchConst);
    controlAngle[3] = -controlAngle[2];
*/

    //std::cout << controlAngle[0]/PI*180 << " " << controlAngle[1]/PI*180 
    //       << " " << controlAngle[2]/PI*180 << " " << controlAngle[3]/PI*180 << " ";

    //std::cout << phaseSwitchConst*5 << " ";


/*
    //std::cout << (-asin(xyTipPos[0]/targetLegLength[0]) - controlPitch[0])/PI*180 << " ";
    //std::cout << (-asin(xyTipPos[0]/targetLegLength[1]) - controlPitch[1])/PI*180 << " ";
    //std::cout << (0.5*( (pitchK[0]*pitchToFront+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0] - linksAngWithVert[1] ))/PI*180 << " ";
    //std::cout << (0.5*( (pitchK[0]*pitchToFront+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[3] - linksAngWithVert[4] ))/PI*180 << " ";

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
      controlAngle[2] = 0.5*( (pitchK[0]*pitchToFront+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[3] - linksAngWithVert[4] );
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
      controlAngle[0] = 0.5*( (pitchK[0]*pitchToFront+pitchK[1]*rpyVel[1])/springAveK + linksAngWithVert[0] - linksAngWithVert[1] );
      controlAngle[1] = -controlAngle[0];
    } // else
    //std::cout << (prevRightStandControl? "Control right stand":"") << std::endl;
    //std::cout << "Left leg now: " << (curLeftLegAngle/PI*180) << " Left leg to be: " << (targetLeftLegAngle/PI*180) << "\n";
    //std::cout << controlAngle[0]/PI*180 << " " << controlAngle[1]/PI*180 
    //       << " " << controlAngle[2]/PI*180 << " " << controlAngle[3]/PI*180 << " ";
*/
    //std::cout << prevRightStandControl*5-15 << std::endl;

    controlAngle[0] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[1] += acos((targetLegLength[0]/2)/(0.26)) - PI/6;
    controlAngle[2] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 
    controlAngle[3] += acos((targetLegLength[1]/2)/(0.26)) - PI/6; 

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

  //std::cout << commands[2] << " " << commands[3] << " " << commands[6] << " " << commands[7] << " ";

  double height[2] = { 0.26*(linksAngWithVert[0]+linksAngWithVert[1]), 0.26*(linksAngWithVert[3]+linksAngWithVert[4]) };
  //std::cout << height[0] << " " <<  height[1] << std::endl;
  
  
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

void linksAngleAndVel(std::vector<double>& linksAngWithVert, 
                      std::vector<double>& linksAngVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel,
                      const std::vector<double>& rpyImu,
                      const std::vector<double>& rpyVel)
{
  linksAngWithVert.clear();
  linksAngVel.clear();
  linksAngWithVert.reserve(3*2);
  linksAngVel.reserve(3*2);
  for(unsigned int i=0; i<2; ++i){
    linksAngWithVert.push_back( jointPos[4+i*4]+jointPos[2+i*4]+rpyImu[1]+3.14159/6.0 ); // PI/6 comes from xacro definition of the robot // left miu
    linksAngWithVert.push_back( jointPos[5+i*4]+jointPos[3+i*4]-rpyImu[1]+3.14159/6.0 ); // left niu
    linksAngWithVert.push_back( jointPos[i] ); // left abad
    linksAngVel.push_back( jointVel[4+i*4]+jointVel[2+i*4]+rpyVel[1] ); // right miu
    linksAngVel.push_back( jointVel[5+i*4]+jointVel[3+i*4]-rpyVel[1] ); // right niu
    linksAngVel.push_back( jointVel[i] ); // right abad

    //std::cout << linksAngWithVert[i*3]/3.14159*180 << " " << linksAngWithVert[i*3+1]/3.14159*180 << " " << linksAngWithVert[i*3+2]/3.14159*180 << " ";
  }
  //std::cout << std::endl;
  //std::cout << jointPos[2] << " " << jointPos[3] << " " << jointPos[6] << " " << jointPos[7] << " ";
  //std::cout << jointPos[4]+jointPos[2] << " " << jointPos[5]+jointPos[3] << " " << jointPos[8]+jointPos[6] << " " << jointPos[9]+jointPos[7] << " ";

  return;
}

void legTipForce(std::vector<double>& tipForce, 
                 std::deque<double>& leftTipYForce,
                 std::deque<double>& rightTipYForce,
                 unsigned int tipYForceSize,
                 const std::vector<double>& linksAngWithVert, 
                 const std::vector<double>& jointPos, 
                 const std::vector<double>& springCoef)
{
  const double PI = 3.14159;
  const double leg_mass_total = 0.34; // 4 leg links (0.08kg), 1 leg tip (0.02kg)

  // force on bottom links by top link (k*(link angle-spring angle)/link length)
  std::vector<double> force_rear{ springCoef[0]*jointPos[4]/0.26, springCoef[0]*jointPos[8]/0.26 }; // left, right
  std::vector<double> force_front{ springCoef[1]*jointPos[5]/0.2, springCoef[1]*jointPos[9]/0.2 }; // left, right

  // Angle theta on link AB in leg tip force calculation
  std::vector<double> theta { PI/2-linksAngWithVert[0]-linksAngWithVert[1], PI/2-linksAngWithVert[3]-linksAngWithVert[4] }; // left, right
  
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
    double alpha = linksAngWithVert[i*3+1] - atan(linkForce[i*2+1]/linkForce[i*2]);
    tipForce.push_back(-force*sin(alpha));
    tipForce.push_back(force*cos(alpha) + leg_mass_total*9.81);
  }

  // Fill in deque of tip force in vertical direction
  leftTipYForce.push_back(tipForce[1]);
  rightTipYForce.push_back(tipForce[3]);
  if(leftTipYForce.size() > tipYForceSize){
    leftTipYForce.pop_front();
  }
  if(rightTipYForce.size() > tipYForceSize){
    rightTipYForce.pop_front();
  }

  //std::cout << tipForce[0] << " " << tipForce[1] << " ";
  //std::cout << tipForce[2] << " " << tipForce[3] << " ";
  //std::cout << linkForce[0] << " " << linkForce[1] << " ";
  //std::cout << force_rear[0]+force_front[0] << " " << force_rear[1]+force_front[1] << " ";
  writeToFile( std::to_string(tipForce[0]) + " " + std::to_string(tipForce[1]) + " ");
  writeToFile( std::to_string(tipForce[2]) + " " + std::to_string(tipForce[3]) + " ");
  writeToFile( std::to_string(linkForce[0]) + " " + std::to_string(linkForce[1]) + " ");

  //std::cout << std::endl;

  return;
}

bool detectStance(const std::deque<double>& sideTipYForce){
  double threshold = 25.0; //25
  double baseline = std::accumulate(sideTipYForce.begin(), sideTipYForce.end()-1, 0.0)/(sideTipYForce.size()-1);

  bool large = (sideTipYForce.back()-baseline) > threshold;

  //std::cout << baseline << " " << sideTipYForce.back() << " " << sideTipYForce.back()-baseline << " ";
  writeToFile(std::to_string(baseline) + " " + std::to_string(sideTipYForce.back()-baseline)+ " ");
  //std::cout << large << std::endl;
  writeToFile(std::to_string(large) + " ");

  return large;
}

void singleSupportSwitch(bool& swang,
                         unsigned int& walkingState, 
                         double phaseSwitchConst,
                         const std::deque<double>& oppTipYForce){
  int controlMethod = 0;  // 0: by default sinusoidal wave;  1: by leg tip force
  switch(controlMethod)
  {
    case 0:
      if(swang){ 
        if(phaseSwitchConst > 0 && phaseSwitchConst < 1){
          walkingState = (walkingState+1)%4;
          swang = false;
          std::cout << "Double now" << std::endl;
        }
      } else {
        swang = (phaseSwitchConst == 0.0 || phaseSwitchConst == 1.0);
      }
      break;

    case 1:
      if(detectStance(oppTipYForce)){
        walkingState = (walkingState+1)%4;
      }
      break;
  }
  return;
}

void doubleSupportSwitch(unsigned int& walkingState, 
                         bool rightToSwing, 
                         double phaseSwitchConst,
                         const std::vector<double>& jointPos){

  int controlMethod = 0;  // 0: by default sinusoidal wave;  1: by leg length radial deflection
  switch(controlMethod)
  {
    case 0:
      if(rightToSwing){
        if(phaseSwitchConst > 0.6){
          walkingState = (walkingState+1)%4;  
          std::cout << "Right swing now" << std::endl;
        }
      } else {
        if(phaseSwitchConst < 0.4){
          walkingState = (walkingState+1)%4;  
          std::cout << "Left swing now" << std::endl;
        }
      }

      break;

    case 1:
      double diffThreshold = 0.0004;
      double legLen = 2*0.26*cos((jointPos[2+4*rightToSwing]+jointPos[3+4*rightToSwing]+jointPos[4+4*rightToSwing]+jointPos[5+4*rightToSwing])/2);
      double contLen = 2*0.26*cos((jointPos[2+4*rightToSwing]+jointPos[3+4*rightToSwing])/2);
      //writeToFile(std::to_string(leftLegLen) + " " + std::to_string(leftContLen) + " " + 
      //              std::to_string(rightLegLen) + " " + std::to_string(rightContLen) + " ");
      //writeToFile(std::to_string( (contLen-legLen)<diffThreshold ) + " " );
      if((contLen-legLen)<diffThreshold){
        walkingState = (walkingState+1)%4;
      }
      break;
  }

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
  //std::cout << rightStand*5 << " ";

  return;
}

void getLinearVelFromJoint(std::deque< std::vector<double> >& linearVelFromJoint, 
                           const bool& rightStand,
                           const std::vector<double>& linksAngWithVert,
                           const std::vector<double>& linksAngVel)
{
  std::vector<double> curLinearVelFromJoint;
  curLinearVelFromJoint.reserve(3);
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
  
  if(linearVelFromJoint.size() > 10){
    linearVelFromJoint.pop_front();
  }


  /*double speed[3] = {0.0,0.0,0.0};
  for(unsigned int j=0; j<3; ++j){
    for(unsigned int i=0; i<linearVelFromJoint.size(); ++i){
      speed[j] += linearVelFromJoint[i][j];
    }
    speed[j] /= linearVelFromJoint.size();
  }
  std::cout << speed[0] << " " << speed[1] << " " << speed[2] << " ";*/
  
  //std::cout << linearVelFromJoint.size() << std::endl;

  //std::cout << linearVelFromJoint.back()[0] << " " << linearVelFromJoint.back()[1] << " " << linearVelFromJoint.back()[2] << "" << std::endl;

  // print height of robot body
  //std::cout << 0.26*(cos(miu)+cos(niu)) << std::endl;

}

void writeToFile(std::string msg){
  std::ofstream myfile("test.txt", std::ios::out | std::ios::app);
  if(myfile.is_open()){
    myfile << msg;
  }
  else std::cout << "Unable to open file" << std::endl;

  return;
}


// Get the IMU data
IMUData get_imu_data(){
  IMUData imu;

  // get acceleration data and put in acc[3] array, and
  // get angular velocity data and put in omega[3] array


  return imu;
}


