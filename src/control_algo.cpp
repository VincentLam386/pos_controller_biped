#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include <ros/console.h>

// Implement the control algorithm here
void update_control(std::vector<double>& commands, const std::vector<hardware_interface::JointHandle>& joints_, const std::vector<double>& rpyImu, const ros::Time& time){
  // commands:    a vector storing the target positions for each joint
  //              to be updated in this function
  // joints_:     a vector storing all the joints (motors), can use 
  //              joints_[i].getPosition() to get the current angle of the
  //              (i-1)th joint
  // rpyImu:      a vector storing the data of IMU sensor (roll, pitch, yaw)
  // time:        current time (from ROS)
  // 
  // Call get_imu_data() to get the IMU data in this function
  // example control
  // commands[0] = 0.3 * sin( (double)(loop_count_/500.0));
  // commands[1] = -0.3 * sin( (double)(loop_count_/500.0));

  const float stepFrequency = 3;
  const float PI = 3.14159;
  const float maxAngle = 2.0 *(PI/180.0);
  const float leg_0 = 0.51; //neutral length of leg
  const float leg_maxRet = 0.1; //max retraction length of the leg
 
  const float retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)*time.toSec());

  float forwardSpeed = 0.0;
  float targetLeftLegLength;
  float targetRightLegLength;
  float targetLeftLegAngle = -0.0;
  float targetRightLegAngle = -0.0;

  // Get the current actual leg angles using ideal geometry
  float curLeftLegAngle = -joints_[2].getPosition() - joints_[3].getPosition()/2.0;
  float curRightLegAngle = -joints_[6].getPosition() - joints_[7].getPosition()/2.0;
  // Get the current actual hip pitch angles from IMU
  //float pitchToFront = 50.0 *(PI/180.0); // for program testing, input artificial pitch
  float pitchToFront = rpyImu[1];
  
  if (retractionLength < 0){
    targetLeftLegLength = leg_0 + retractionLength;
    targetRightLegLength = leg_0;
  /*  if (pitchToFront < 0){
      // if deviate from horizontal by -30 deg, move vertical link by +30 deg to make it vertical
      //   and move further by +30 degree to provide returning force (?)
      targetLeftLegAngle = std::max(2*pitchToFront-curLeftLegAngle,-maxAngle) + curLeftLegAngle;
    }
    else {
      targetLeftLegAngle = std::min(2*pitchToFront-curLeftLegAngle,maxAngle) + curLeftLegAngle;  
    }
    if (curRightLegAngle < 0){
      targetRightLegAngle = std::min(0-curRightLegAngle, maxAngle) + curRightLegAngle;
    }
    else {    
      targetRightLegAngle = std::max(0-curRightLegAngle,-maxAngle) + curRightLegAngle;
    }
    */
  }
  else {
    targetRightLegLength = leg_0 - retractionLength;
    targetLeftLegLength = leg_0;
  /*  if (pitchToFront < 0) {
      targetRightLegAngle = std::max(2*pitchToFront-curRightLegAngle,-maxAngle) + curRightLegAngle;
    }
    else{
      targetRightLegAngle = std::min(2*pitchToFront-curRightLegAngle,maxAngle) + curRightLegAngle;
    }
    if (curLeftLegAngle < 0){
      targetLeftLegAngle = std::min(0-curLeftLegAngle, maxAngle) + curLeftLegAngle;
    }
    else {    
      targetLeftLegAngle = std::max(0-curLeftLegAngle,-maxAngle) + curLeftLegAngle;
    }*/
  }
  //std::cout << "Left leg now: " << (curLeftLegAngle/PI*180) << " Left leg to be: " << (targetLeftLegAngle/PI*180) << "\n";
  //std::cout << "Pitch now: " << (pitchToFront/PI*180) << std::endl;

  float leftAngle = acos((targetLeftLegLength/2)/(0.26)); 
  float targetLeftBottomLinkAngle = -2 * leftAngle;  

  float rightAngle = acos((targetRightLegLength/2)/(0.26)); 
  float targetRightBottomLinkAngle = -2 * rightAngle;

  /*----------Update the joint angles below------------*/
  // left_abad_joint
  commands[0] = 0;

  // right_abad_joint
  commands[1] = 0;

  // left top joint
  commands[2] = leftAngle - targetLeftLegAngle;
  
  // left bottom joint
  commands[3] = targetLeftBottomLinkAngle;

  // left front top joint
  commands[4] = -leftAngle - targetLeftLegAngle;

  // left front bottom joint
  commands[5] = -targetLeftBottomLinkAngle;

  // right top joint
  commands[6] = rightAngle - targetRightLegAngle;

  // right bottom joint
  commands[7] = targetRightBottomLinkAngle;

  // right front top joint
  commands[8] = -rightAngle - targetRightLegAngle;

  // right front bottom joint
  commands[9] = -targetRightBottomLinkAngle;

  /*
  int argc=0;
  char* str = "empty";
  char** argv= &str;
  ros::init(argc,argv,"talker");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("command_publisher",1000);
  std_msgs::Float64MultiArray msg;
  for (int i=0;i<10;++i){
    msg.data.push_back(commands[i]);
  }		 
  pub.publish(msg);
  ros::spinOnce();
  */
  
  return;
}


// Get the IMU data
IMUData get_imu_data(){
  IMUData imu;

  // get acceleration data and put in acc[3] array, and
  // get angular velocity data and put in omega[3] array


  return imu;
}

