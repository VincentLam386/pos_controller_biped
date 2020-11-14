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

#include <pos_controller_biped/pos_controller_biped.h>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>


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
  GrpPosController::GrpPosController(): loop_count_(0) {}
  GrpPosController::~GrpPosController() {sub_command_.shutdown();}

  bool GrpPosController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
  {


    hardware_interface::EffortJointInterface* eff = robot_hw->get<hardware_interface::EffortJointInterface>();
    hardware_interface::ImuSensorInterface* imu = robot_hw->get<hardware_interface::ImuSensorInterface>();



    //IMU Portion
    const std::vector<std::string>& sensor_names = imu->getNames();
    for (unsigned i=0; i<sensor_names.size(); i++)
      ROS_DEBUG("Got sensor %s", sensor_names[i].c_str());

    for (unsigned i=0; i<sensor_names.size(); i++){
      // sensor handle
      sensors_.push_back(imu->getHandle(sensor_names[i]));
    }




    // List of controlled joints
    if(!n.getParam("joints", joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam 'joints' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // Get URDF
    urdf::Model urdf;
    if (!urdf.initParamWithNodeHandle("robot_description", n))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    pid_controllers_.resize(n_joints_);

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      try
      {
        joints_.push_back(eff->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);

      // Load PID Controller using gains set on parameter server
      if (!pid_controllers_[i].init(ros::NodeHandle(n, joint_name + "/pid")))
      {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_name + "/pid");
        return false;
      }
    }

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &GrpPosController::commandCB, this);
    return true;
  }

  void GrpPosController::starting(const ros::Time& time)
  {
    std::vector<double> current_positions(n_joints_, 0.0);
    for (std::size_t i = 0; i < n_joints_; ++i)
    {
      current_positions[i] = joints_[i].getPosition();
      enforceJointLimits(current_positions[i], i);
      pid_controllers_[i].reset();
    }
    commands_buffer_.initRT(current_positions);
  }

  void GrpPosController::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();

    update_control(loop_count_, commands, joints_, time);

    for(unsigned int i=0; i<n_joints_; i++)
    {
        double command_position = commands[i];

        double error;
        double commanded_effort;

        double current_position = joints_[i].getPosition();

        // Make sure joint is within limits if applicable
        enforceJointLimits(command_position, i);

        // Compute position error
        if (joint_urdfs_[i]->type == urdf::Joint::REVOLUTE)
        {
          angles::shortest_angular_distance_with_large_limits(
            current_position,
            command_position,
            joint_urdfs_[i]->limits->lower,
            joint_urdfs_[i]->limits->upper,
            error);
        }
        else if (joint_urdfs_[i]->type == urdf::Joint::CONTINUOUS)
        {
          error = angles::shortest_angular_distance(current_position, command_position);
        }
        else //prismatic
        {
          error = command_position - current_position;
        }

        // Set the PID error and compute the PID command with nonuniform
        // time step size.
        commanded_effort = pid_controllers_[i].computeCommand(error, period);

        joints_[i].setCommand(commanded_effort);
    }


   //IMU Portion
   if (sensors_[0].getOrientation()){
	   /*ROS_INFO_STREAM("Orientation X,Y,Z,W = "	<< sensors_[0].getOrientation()[0] <<", "
							<< sensors_[0].getOrientation()[1] <<", "
							<< sensors_[0].getOrientation()[2] <<", "
							<< sensors_[0].getOrientation()[3] <<", "
	   );*/
	double roll,pitch,yaw;
	tf2::Quaternion quat{	sensors_[0].getOrientation()[0],
				sensors_[0].getOrientation()[1], 
				sensors_[0].getOrientation()[2], 
				sensors_[0].getOrientation()[3], 
				};

	tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	
	ROS_INFO_STREAM("Orientation RPY = "	<< roll*(180.0/3.14159)<<", "<<pitch*(180.0/3.14159)<<", "<<yaw*(180.0/3.14159));
	
   }
   else{
	ROS_WARN("No Orientation Data");
   }	


    ++loop_count_;
  }

  void GrpPosController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
  {
    if(msg->data.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    commands_buffer_.writeFromNonRT(msg->data);
  }

  void GrpPosController::enforceJointLimits(double &command, unsigned int index)
  {
    // Check that this joint has applicable limits
    if (joint_urdfs_[index]->type == urdf::Joint::REVOLUTE || joint_urdfs_[index]->type == urdf::Joint::PRISMATIC)
    {
      if( command > joint_urdfs_[index]->limits->upper ) // above upper limnit
      {
        command = joint_urdfs_[index]->limits->upper;
      }
      else if( command < joint_urdfs_[index]->limits->lower ) // below lower limit
      {
        command = joint_urdfs_[index]->limits->lower;
      }
    }
  }

} // namespace

PLUGINLIB_EXPORT_CLASS( pos_controller_biped_ns::GrpPosController, controller_interface::ControllerBase)
