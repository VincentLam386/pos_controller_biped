#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"

// Implement the control algorithm here
void update_control(int loop_count_,std::vector<double>& commands, const std::vector<hardware_interface::JointHandle>& joints_, ros::Time time){
  // loop_count_: number of loops passed in simulation (may not be needed 
  //              in actual implementation)
  // commands:    a vector storing the target positions for each joint
  // joints_:     a vector storing all the joints (motors), can use 
  //              joints_[i].getPosition() to get the current angle of the
  //              (i-1)th joint
  // 
  // Call get_imu_data() to get the IMU data in this function


  // example control
//  int div = commands.size()/2;
 // commands[0] = 0.3 * sin( (double)(loop_count_/500.0));
 // commands[1] = -0.3 * sin( (double)(loop_count_/500.0));
  const float offset = 0.4;
  const float stepFrequency = 3;

  const float leg_0 = 0.51; //neutral length of leg
  const float leg_maxRet = 0.1; //max retraction length of the leg
 
  const float retractionLength = leg_maxRet * sin(2*3.14159*(stepFrequency/2.0)*time.toSec());

  float targetLeftLegLength = retractionLength < 0? (leg_0 + retractionLength) : leg_0;
  float targetRightLegLength = retractionLength > 0? (leg_0 - retractionLength) : leg_0;

  //float targetLeftLegAngle= 0.5*sin(2*3.14159*(stepFrequency/8.0)*time.toSec());
  float targetLeftLegAngle = 0;
  //float targetRightLegAngle= -0.5*sin(2*3.14159*(stepFrequency/8.0)*time.toSec());
  float targetRightLegAngle = 0;

  float leftAngle = acos((targetLeftLegLength/2)/(0.26)); 
  float targetLeftBottomLinkAngle = 2 * leftAngle;  
  float targetLeftTopLinkAngle = targetLeftLegAngle - leftAngle;

  float rightAngle = acos((targetRightLegLength/2)/(0.26)); 
  float targetRightBottomLinkAngle = 2 * rightAngle;
  float targetRightTopLinkAngle = targetRightLegAngle - rightAngle;




  // left_abad_joint
  commands[0] = 0;

  // right_abad_joint
  commands[1] = 0;

  // left top joint
  commands[2] = -targetLeftTopLinkAngle;
  
  // left bottom joint
  commands[3] = -targetLeftBottomLinkAngle;

  // left front top joint
  commands[4] = -targetLeftLegAngle - leftAngle;
  //commands[4] = 0;

  // left front bottom joint
  commands[5] = targetLeftBottomLinkAngle;
  //commands[5] = 0;

  // right top joint
  commands[6] = -targetRightTopLinkAngle;

  // right bottom joint
  commands[7] = -targetRightBottomLinkAngle;

  // right front top joint
  commands[8] = -targetRightLegAngle - rightAngle;
  //commands[8] = 0;

  // right front bottom joint
  commands[9] = targetRightBottomLinkAngle;
  //commands[9] = 0;

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

//  for (int i=0;i<div;++i){
//    commands[i] =0.3 * sin( (double)(loop_count_/500.0)); 
//    commands[i+1] = -0.3*sin( (double)(loop_count_/500.0));
//  }
//  for (int i=div;i<commands.size();++i){
//    commands[i] = -0.3* sin( (double)(loop_count_/500.0));
  //  commands[i+1] = 0.3*sin( (double)(loop_count_/500.0));
//  }
  
  return;
}


// Get the IMU data
IMUData get_imu_data(){
  IMUData imu;

  // get acceleration data and put in acc[3] array, and
  // get angular velocity data and put in omega[3] array


  return imu;
}
