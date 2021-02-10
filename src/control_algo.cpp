#include "pos_controller_biped/control_algo.h"
#include "std_msgs/Float64MultiArray.h"
#include "ros/ros.h"
#include <ros/console.h>

// Implement the control algorithm here
void update_control(std::vector<double>& commands, const double niu, const std::vector<hardware_interface::JointHandle>& joints_, const std::vector<double>& linearVelFromJoint, const std::vector<double>& rpyImu, const ros::Time& time){
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

  const double stepFrequency = 3;
  const double PI = 3.14159;
  const double maxAngle = 2.0 *(PI/180.0);
  const double leg_0 = 0.51; //neutral length of leg
  const double leg_maxRet = 0.1; //max retraction length of the leg
 
  const double retractionLength = leg_maxRet * sin(2*PI*(stepFrequency/2.0)*time.toSec());

  double forwardSpeed = 0.0;
  double targetLeftLegLength;
  double targetRightLegLength;
  double targetLeftLegAngle = -0.0;
  double targetRightLegAngle = -0.0;

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
  //float targetLeftBottomLinkAngle = -2 * leftAngle;  

  float rightAngle = acos((targetRightLegLength/2)/(0.26)); 
  //float targetRightBottomLinkAngle = -2 * rightAngle;

  /*----------Update the joint angles below------------*/
  // left_abad_joint
  commands[0] = 0;

  // right_abad_joint
  commands[1] = 0;

  // left spring rear joint
  commands[2] = leftAngle - targetLeftLegAngle;

  // left spring front joint
  commands[3] = leftAngle - targetLeftLegAngle;

  // left rear joint
  commands[4] = 0;

  // left front joint
  commands[5] = 0;

  // right spring rear joint
  commands[6] = rightAngle - targetRightLegAngle;

  // right spring front joint
  commands[7] = rightAngle - targetRightLegAngle;

  // right rear joint
  commands[8] = 0;

  // right front joint
  commands[9] = 0;

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

void motorAngleAndVel(std::vector<double>& motorAngWithBase, 
                      std::vector<double>& motorVel, 
                      const std::vector<double>& jointPos, 
                      const std::vector<double>& jointVel)
{
  motorAngWithBase.clear();
  motorVel.clear();
  motorAngWithBase.reserve(3*2);
  motorVel.reserve(3*2);
  for(unsigned int i=0; i<3; ++i){
    motorAngWithBase.push_back( jointPos[4+i*4]+jointPos[2+i*4] );
    motorAngWithBase.push_back( jointPos[5+i*4]+jointPos[3+i*4] );
    motorAngWithBase.push_back( jointPos[0+i] );
    motorVel.push_back( jointVel[4+i*4]+jointVel[2+i*4] );
    motorVel.push_back( jointVel[5+i*4]+jointVel[3+i*4] );
    motorVel.push_back( jointVel[0+i] );
  }

  return;
}

void legTipForce(std::vector<double>& tipForce, 
                 const std::vector<double>& motorAngWithBase, 
                 const std::vector<double>& jointPos, 
                 const std::vector<double>& springCoef)
{
  const double PI = 3.14159;
  std::cout << "Link angle diff: " << jointPos[8] << " " << jointPos[9] << std::endl;

  // force on bottom links by top link (k*(link angle-spring angle)*link length)
  std::vector<double> force_front{ springCoef[0]*jointPos[5]*0.2, springCoef[0]*jointPos[9]*0.2 }; // left, right
  std::vector<double> force_rear{ springCoef[1]*jointPos[4]*0.26, springCoef[1]*jointPos[8]*0.26 }; // left, right

  std::cout << "Link angle: " << motorAngWithBase[3] << " " << motorAngWithBase[4] << std::endl;
  // Angle theta and phi (same) on link AB in leg tip force calculation
  std::vector<double> angles { motorAngWithBase[0]+motorAngWithBase[1]-PI/2, motorAngWithBase[3]+motorAngWithBase[4]-PI/2 }; // left, right
  
  // Calculate leg tip force in link AB frame
  std::vector<double> linkForce; // left (x,y), right (x,y)
  for(unsigned int i=0; i<force_rear.size(); ++i){
    linkForce.push_back( force_rear[i]*cos(angles[i])-force_front[i]*sin(angles[i]) );
    linkForce.push_back( force_rear[i]*sin(angles[i])+force_front[i]*cos(angles[i]) );
  }

  // Calculate leg tip force in world frame
  tipForce.clear();
  tipForce.reserve(angles.size()*2);
  for(unsigned int i=0; i<angles.size(); ++i){
    double force = sqrt(linkForce[i*2]*linkForce[i*2] + linkForce[i*2+1]*linkForce[i*2+1]);
    double alpha = motorAngWithBase[(i+1)*4] - atan(linkForce[i*2+1]/linkForce[i*2]);
    tipForce.push_back(force*sin(alpha));
    tipForce.push_back(force*cos(alpha));
  }
  std::cout << "Tip force: " << tipForce[2] << " " << tipForce[3] << "\n" << std::endl;

  return;
}

void getLinearVelFromJoint(std::vector<double>& linearVelFromJoint, 
                           bool& rightStand,
                           const std::vector<double>& jointVel, 
                           const std::vector<double>& jointPos, 
                           const double duration)
{
  const float r0 = 0.26;
  const float rb = 0.08;

  // From joint differences (spring and link angle difference -> force), determine which leg is in stance phase
      // ***may not be the right approach, because angle diff when leg start to raise (but still on ground) > angle diff when leg start to rest (but still in air)***
  double leftRearDiff = jointPos[4];
  double leftFrontDiff = jointPos[5];
  double rightRearDiff = jointPos[8];
  double rightFrontDiff = jointPos[9];
  //double prevleftRearDiff = prevjointPos[4] - prevjointPos[2];
  //double prevleftFrontDiff = prevjointPos[5] - prevjointPos[3];
  //double prevrightRearDiff = prevjointPos[8] - prevjointPos[6];
  //double prevrightFrontDiff = prevjointPos[9] - prevjointPos[7];
  
      // Observe the typical difference value at stance phase from the cout message below
  //std::cout << leftRearDiff*180/3.1415926535 << " " << leftFrontDiff*180/3.1415926535 << " " << rightRearDiff*180/3.1415926535 << " " << rightFrontDiff*180/3.1415926535 << std::endl;
  //std::cout << jointPos[8]*180/3.1415926535 << " " << jointPos[6]*180/3.1415926535 << std::endl;
  //std::cout << leftRearDiff*180/3.1415926535 << " " << leftFrontDiff*180/3.1415926535 << " " << rightRearDiff*180/3.1415926535 << " " << rightFrontDiff*180/3.1415926535 << std::endl;
  //std::cout << prevleftRearDiff*180/3.1415926535 << " " << prevleftFrontDiff*180/3.1415926535 << " " << prevrightRearDiff*180/3.1415926535 << " " << prevrightFrontDiff*180/3.1415926535 << std::endl;

  double stanceAngle = 10.0*3.1415926535/180.0;
  double stanceAngleThreshold = 4.0*3.1415926535/180.0;
  double startSwingAngleThreshold = -3.0*3.1415926535/180.0;
  if(rightStand){
    //rightStand = (((abs(rightRearDiff+rightFrontDiff-stanceAngle*2)>abs(leftRearDiff+leftFrontDiff-stanceAngle*2)) && (abs(leftRearDiff+leftFrontDiff) < (stanceAngleThreshold*2)))+1)%2;
    //rightStand = (((abs(rightRearDiff+rightFrontDiff-stanceAngle*2)>abs(leftRearDiff+leftFrontDiff-stanceAngle*2)) && true)+1)%2;
    //rightStand = (( ((leftRearDiff+leftFrontDiff) > (2*stanceAngleThreshold)) && 
    //                ((rightRearDiff+rightFrontDiff) < (2*startSwingAngleThreshold)) ) + 1)%2;
    rightStand = (( ((rightRearDiff+rightFrontDiff) < (2*startSwingAngleThreshold)) ) + 1)%2;
  } else {
    //rightStand = ((rightRearDiff+rightFrontDiff) > (2*stanceAngleThreshold)) &&
    //             ((leftRearDiff+leftFrontDiff) < (2*startSwingAngleThreshold));
    rightStand = (leftRearDiff+leftFrontDiff) < (2*startSwingAngleThreshold);
    //rightStand = (abs(rightRearDiff+rightFrontDiff-stanceAngle*2)<abs(leftRearDiff+leftFrontDiff-stanceAngle*2)) && (abs(rightRearDiff+rightFrontDiff) < (stanceAngleThreshold*2));
    //rightStand = (abs(rightRearDiff+rightFrontDiff-stanceAngle*2)<abs(leftRearDiff+leftFrontDiff-stanceAngle*2)) && true;
  }
  //rightStand = (abs(rightRearDiff+rightFrontDiff-stanceAngle*2)>abs(leftRearDiff+leftFrontDiff-stanceAngle*2)) && (abs(prevrightRearDiff+prevrightFrontDiff) < abs(rightRearDiff+rightFrontDiff));
  //std::cout << rightStand << std::endl;

  // Calculate robot body linear velocity from joint position and velocity
  // (refer to the calculation in "velocity kinematic calculation.pptx")
  // x-direction velocity
  double miu = jointPos[4+rightStand*4]+jointPos[2+rightStand*4];
  double niu = jointPos[5+rightStand*4]+jointPos[3+rightStand*4];
  double lambda = jointPos[0+rightStand];
  double miu_dot = jointVel[4+rightStand*4]+jointVel[2+rightStand*4];
  double niu_dot = jointVel[5+rightStand*4]+jointVel[3+rightStand*4];
  double lambda_dot = jointVel[0+rightStand];

  // x-direction velocity
  linearVelFromJoint[0] = r0*cos(miu) *miu_dot -
                          r0*cos(niu) *niu_dot; 
  // y-direction velocity
  linearVelFromJoint[1] = -rightStand*
                          (-r0*sin(miu)*sin(lambda) *miu_dot -
                          r0*sin(niu)*sin(lambda) *niu_dot +
                          (r0*(cos(miu)+cos(niu))*cos(lambda)-rb*sin(lambda)) *lambda_dot);
  // z-direction velocity   
  linearVelFromJoint[2] = -r0*sin(miu)*cos(lambda) *miu_dot -
                          r0*sin(niu)*cos(lambda) *niu_dot +
                          (-r0*(cos(miu)+cos(niu))*sin(lambda)-rb*cos(lambda)) *lambda_dot;

  //std::cout << linearVelFromJoint[0] << " " << linearVelFromJoint[1] << " " << linearVelFromJoint[2] << "" << std::endl;

}


// Get the IMU data
IMUData get_imu_data(){
  IMUData imu;

  // get acceleration data and put in acc[3] array, and
  // get angular velocity data and put in omega[3] array


  return imu;
}

