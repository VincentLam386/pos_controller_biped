#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include <ros/console.h>

void xyTipPlacement(std::vector<double>& xyTipPos,
                    const std::deque< std::vector<double> >& linearVelFromJoint){
  const double kp = 0.05;
  const double kd = 0.01; 
  const double kv = 0.01;
  std::vector<double> desired_vel { 0.5,0.0 };  // x direction, y direction
  
  double curVel[2] = {0.0, 0.0};
  double diffVel[2] = {0.0, 0.0};
  if(!linearVelFromJoint.empty()){
    for(unsigned int i=0; i<2; ++i){
      for(unsigned int j=0; j<linearVelFromJoint.size(); ++j){
        curVel[i] += linearVelFromJoint[j][i];
      }
      curVel[i] /= linearVelFromJoint.size();
      diffVel[i] = linearVelFromJoint.back()[i]-linearVelFromJoint.front()[i];
    }
  }

  for(unsigned int i=0; i<xyTipPos.size(); ++i){
    xyTipPos[i] = kp*(curVel[i]-desired_vel[i]) + kd*diffVel[i] + kv*curVel[i];
  }

  std::cout << "Variable in x: " << (curVel[0]-desired_vel[0]) << " " << diffVel[0] << " " << curVel[0] << std::endl;
  
  //std::cout << xyTipPos[0] << " " << xyTipPos[1] << std::endl;

  return;
}

void rightStandForControl(bool& rightStandControl, bool& dropping, bool& startTouch, const std::vector<double>& tipForce){
  unsigned int forceIndex = (-rightStandControl*2)+3;
  if(!dropping){
    dropping = (tipForce[forceIndex] > 80);
  } 
  else {
    if(!startTouch){
      startTouch = (tipForce[forceIndex] < 60);
    }
    else {
      if(tipForce[forceIndex] > 61){
        rightStandControl = !rightStandControl;
        dropping = false;
        startTouch = false;
      }
    }
  }
  //std::cout << rightStandControl << std::endl;

  return;
}

// Implement the control algorithm here
void update_control(std::vector<double>& commands, const bool rightStandControl, const std::vector<double>& xyTipPos, const std::vector<hardware_interface::JointHandle>& joints_, const std::vector<double>& rpyImu, const ros::Time& time){
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

  const double stepFrequency = 4;
  const double PI = 3.14159;
  const double maxAngle = 2.0 *(PI/180.0);
  const double leg_0 = 0.51;     //neutral length of leg
  const double leg_maxRet = 0.1; //max retraction length of the leg
 
  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)*time.toSec());

  double forwardSpeed = 0.0;
  double targetLegLength[2] = {0.0, 0.0}; // left, right
  double targetLegAngle[2] =  {0.0,0.0}; // left, right
  double controlAngle[4] = {0.0,0.0, 0.0,0.0}; // left(miu,niu), right(miu,niu)

  double f1 = 0.0; // force threshold for stance phase
  double f2 = 0.0; // force threshold for swing phase
  double ratio = 0.0; 

  // Get the current actual leg angles using ideal geometry
  //float curLeftLegAngle = -joints_[2].getPosition() - joints_[3].getPosition()/2.0;
  //float curRightLegAngle = -joints_[6].getPosition() - joints_[7].getPosition()/2.0;
  // Get the current actual hip pitch angles from IMU
  //float pitchToFront = 50.0 *(PI/180.0); // for program testing, input artificial pitch
  float pitchToFront = rpyImu[1];
  
  if (retractionLength < 0){
    targetLegLength[0] = leg_0 + retractionLength;
    targetLegLength[1] = leg_0;

    std::cout << "Left tip x pos & length: " << xyTipPos[0] << " " << targetLegLength[0] << std::endl;

  }
  else {
    targetLegLength[0] = leg_0;
    targetLegLength[1] = leg_0 - retractionLength;

  }

  if(rightStandControl){
    controlAngle[0] = -asin(xyTipPos[0]/targetLegLength[0]);
    std::cout << "Left tip x angle: " << controlAngle[0]/PI*180 << std::endl;
    controlAngle[1] = -controlAngle[0];
    //controlAngle[2] = -controlAngle[0]*0.5;
    //controlAngle[3] = controlAngle[0]*0.5;
    controlAngle[2] = 0.0;
    controlAngle[3] = 0.0;
  }
  else{    
    controlAngle[2] = -asin(xyTipPos[0]/targetLegLength[1]);
    controlAngle[3] = -controlAngle[2];

    //controlAngle[0] = -controlAngle[2]*0.5;
    //controlAngle[1] = controlAngle[2]*0.5;
    controlAngle[0] = 0.0;
    controlAngle[1] = 0.0;
  }
  //std::cout << "Left leg now: " << (curLeftLegAngle/PI*180) << " Left leg to be: " << (targetLeftLegAngle/PI*180) << "\n";

  controlAngle[0] += acos((targetLegLength[0]/2)/(0.26));
  controlAngle[1] += acos((targetLegLength[0]/2)/(0.26));
  controlAngle[2] += acos((targetLegLength[1]/2)/(0.26)); 
  controlAngle[3] += acos((targetLegLength[1]/2)/(0.26)); 

  //std::cout << controlAngle[0]/PI*180 << " " << controlAngle[1]/PI*180 << std::endl;
  std::cout << std::endl;

  /*----------Update the joint angles below------------*/
  // left_abad_joint
  commands[0] = 0;

  // right_abad_joint
  commands[1] = 0;

  // left spring rear joint
  commands[2] = controlAngle[0];

  // left spring front joint
  commands[3] = controlAngle[1];

  // left rear joint
  commands[4] = 0;

  // left front joint
  commands[5] = 0;

  // right spring rear joint
  commands[6] = controlAngle[2];

  // right spring front joint
  commands[7] = controlAngle[3];

  // right rear joint
  commands[8] = 0;

  // right front joint
  commands[9] = 0;
  
  return;
}

void getJointVel(std::vector<double>& jointVel, std::deque< std::vector<double> >& jointPosCummulative, std::deque<uint64_t>& time_ms, const uint64_t curTime_msec, const std::vector<double>& jointPos){
  if (jointPosCummulative.size() < 15) {
    // Fill up the vectors of size 15
    if (!time_ms.empty()){
      if((time_ms.back() - curTime_msec) != 0) {
        // Push back vector if timestamp is different
        jointPosCummulative.push_back(jointPos);
        time_ms.push_back(curTime_msec);
      } 
      else { 
        // Replace last element with the new joint position when timestamp is the same as previous one
        jointPosCummulative.pop_back();
        jointPosCummulative.push_back(jointPos);
      }
    } 
    else {
      // Start filling up vectors (very first time only)
      jointPosCummulative.push_back(jointPos);
      time_ms.push_back(curTime_msec);
    }
  } 
  else {
    // Start estimating velocity and Allowing pop front when vectors have size 15
    if((time_ms.back() - curTime_msec) != 0) {
      // Pop front and push back when timestamp is different
      jointPosCummulative.pop_front();
      time_ms.pop_front();
      jointPosCummulative.push_back(jointPos);
      time_ms.push_back(curTime_msec);
    } else {
      // Replace last element with the new joint position when timestamp is the same as previous one
      jointPosCummulative.pop_back();
      jointPosCummulative.push_back(jointPos);
    }
    // Estimate joint velocity
    for(unsigned int i=0; i<jointVel.size(); ++i){
      // Use finte backward numerical differentiation (with different timesteps)
      //jointVel[i] = (3*(jointPosCummulative[2][i]-jointPosCummulative[1][i])/(2*(time_ms[2]-time_ms[1])) - ((jointPosCummulative[1][i]-jointPosCummulative[0][i])/(2*(time_ms[1]-time_ms[0]))))*1000;

      // Use simple numerical differentiation
      //jointVel[i] = (jointPosCummulative[2][i]-jointPosCummulative[1][i])/(time_ms[2]-time_ms[1])*1000; 

      // Simple numerical differentiation with 15 time interval
      jointVel[i] = (jointPosCummulative[14][i]-jointPosCummulative[0][i])/(time_ms[14]-time_ms[0])*1000;
    }
  }

  //std::cout << jointVel[1]/PI*180 << " " << truejointVel[1]/PI*180 << " " << (jointVel[8]+jointVel[6])/PI*180 << " " << (truejointVel[8]+truejointVel[6])/PI*180 << " " << (jointVel[9]+jointVel[7])/PI*180 << " " << (truejointVel[9]+truejointVel[7])/PI*180 << "" << std::endl;

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

  //std::cout << "Tip force: " << tipForce[0] << " " << tipForce[1] << "\n" << std::endl;
  //std::cout << tipForce[1] << " " << tipForce[3] << std::endl;

  return;
}

void rightStandForLinearVel(bool& rightStand, const std::vector<double>& tipForce){
  double startSwingForceThreshold = -8.0;
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
                           const std::vector<double>& jointPos)
{
  if(linearVelFromJoint.size() >= 15){
    linearVelFromJoint.pop_front();
  }
  std::vector<double> curLinearVelFromJoint;
  curLinearVelFromJoint.reserve(3);
  const float r0 = 0.26;
  const float rb = 0.08;

  // Calculate robot body linear velocity from joint position and velocity
  // (refer to the calculation in "velocity kinematic calculation.pptx")
  double miu = jointPos[4+rightStand*4]+jointPos[2+rightStand*4];
  double niu = jointPos[5+rightStand*4]+jointPos[3+rightStand*4];
  double lambda = jointPos[0+rightStand];
  double miu_dot = jointVel[4+rightStand*4]+jointVel[2+rightStand*4];
  double niu_dot = jointVel[5+rightStand*4]+jointVel[3+rightStand*4];
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

  //std::cout << linearVelFromJoint[0] << " " << linearVelFromJoint[1] << " " << linearVelFromJoint[2] << "" << std::endl;

}


// Get the IMU data
IMUData get_imu_data(){
  IMUData imu;

  // get acceleration data and put in acc[3] array, and
  // get angular velocity data and put in omega[3] array


  return imu;
}

