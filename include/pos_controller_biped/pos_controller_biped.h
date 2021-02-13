/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once


#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>   //This might not be needed anymore
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>
#include <pos_controller_biped/control_algo.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <math.h>
#include <valarray>
#include <deque>



namespace pos_controller_biped_ns
{

/**
 * \brief Forward command controller for a set of effort controlled joints (torque or force).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \section ROS interface
 *
 * \param type Must be "JointGroupEffortController".
 * \param joints List of names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
 */
class GrpPosController : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
									       hardware_interface::ImuSensorInterface>
{
public:
  GrpPosController();
  ~GrpPosController();

//  void update_control(std::vector<double>& command);

  //bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n); //Old init function before MultiInterfaceController is used

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n); //Init of MultiInterfaceController takes a RobotHW fist argument.
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
  void starting(const ros::Time& /*time*/);

  std::vector< std::string > joint_names_;
  std::vector< hardware_interface::JointHandle > joints_;
  realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
  unsigned int n_joints_;

private:
  const double PI = 3.1415926535;
  int loop_count_;
  //int updateAcc = 10;
  ros::Time lastTime;
  ros::Time curTime;
  ros::Duration dur;
  double dur_t;
  std::deque<uint64_t> time_ms;
  double niu;
  bool rightStand;
  bool rightStandControl;
  bool dropping;
  bool startTouch;
  ros::Subscriber sub_command_;

  std::vector<control_toolbox::Pid> pid_controllers_;       /**< Internal PID controllers. */

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  void enforceJointLimits(double &command, unsigned int index);

  std::vector<double> truejointVel; // size of 10
  std::vector<double> jointPos; // size of 10
  std::vector<double> jointVel; // size of 10
  std::vector<double> linksAngWithBase; // size of 6 (left 3 (miu, niu, abad), right 3)
  std::vector<double> linksVel; // size of 6 (left 3, right 3)

  std::vector<double> springCoef; // size of 2 (front, rear)

  std::vector<double> tipForce; // size of 4 (left (x,y), right (x,y))
  
  std::deque< std::vector<double> > jointPosCummulative; // size of 15

  std::vector<hardware_interface::ImuSensorHandle> sensors_;
  std::vector<double> rpyImu; // size of 3
  std::vector<double> linearAcc; // size of 3

  //std::vector<double> linearVelFromJoint; // size of 3
  std::deque< std::vector<double> > linearVelFromJoint;
  //std::vector<double> prevLinearVelFromJoint; // size of 3

  std::vector<double> xyTipPos; // size of 2 (x and y)


  //std::vector<double> linearDisFromAcc; // size of 3

}; // class

} // namespace
