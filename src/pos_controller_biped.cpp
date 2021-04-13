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
  GrpPosController::GrpPosController(): loop_count_(0), max_torque(35.0), swang(false), walkingState(0){
    rpyImu.reserve(3);
    rpyVel.reserve(3);
    for(unsigned int i=0;i<3;++i){
      rpyImu.push_back(0.0);
      rpyVel.push_back(0.0);
    }

    std::array<double,2> zeroArray2 = {0.0,0.0};
    tipForce = {zeroArray2,zeroArray2};
    std::array<double,3> zeroArray3 = {0.0,0.0,0.0};
    linksAngWithVert = {zeroArray3,zeroArray3};
    linksAngVel = {zeroArray3,zeroArray3};

  }
  GrpPosController::~GrpPosController() {sub_command_.shutdown();}

  bool GrpPosController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
  {
    prevRightStandControl = false;

    hardware_interface::EffortJointInterface* eff = robot_hw->get<hardware_interface::EffortJointInterface>();
    hardware_interface::ImuSensorInterface* imu = robot_hw->get<hardware_interface::ImuSensorInterface>();

    // Set up spring coefficient
    springCoef = {154.6986, 143.2394};
    
    //IMU Portion
    const std::vector<std::string>& sensor_names = imu->getNames();
    sensors_.reserve(sensor_names.size());
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

    truejointVel.resize(n_joints_);
    jointPos.resize(n_joints_);
    jointVel.resize(n_joints_);

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
    joint_urdfs_.reserve(n_joints_);
    joints_.reserve(n_joints_);

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
    curTime = ros::Time::now();
    startTime = ros::Time::now();
    //std::cout << "Number of joints: " << n_joints_ << std::endl;
    ROS_INFO_STREAM("Number of joints = " << n_joints_  );
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
    curTime = ros::Time::now();
    uint64_t curTime_msec = curTime.toNSec()/1000000;

    //IMU Portion
    if (sensors_[0].getOrientation()){
	   /*ROS_INFO_STREAM("Orientation X,Y,Z,W = "	<< sensors_[0].getOrientation()[0] <<", "
							<< sensors_[0].getOrientation()[1] <<", "
							<< sensors_[0].getOrientation()[2] <<", "
							<< sensors_[0].getOrientation()[3] <<", "
	   );*/
	double roll,pitch,yaw;
	tf2::Quaternion quat{sensors_[0].getOrientation()[0],
		sensors_[0].getOrientation()[1], 
		sensors_[0].getOrientation()[2], 
		sensors_[0].getOrientation()[3], 
	};

	tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	rpyImu[0] = roll; rpyImu[1] = pitch; rpyImu[2] = yaw;
	
	//ROS_INFO_STREAM("Orientation RPY = "	<< roll*(180.0/3.14159)<<"," <<pitch*(180.0/3.14159)<<", "<<yaw*(180.0/3.14159));
	
    }
    else{
 	 ROS_WARN("No Orientation Data");
    }	
    
    if (sensors_[0].getLinearAcceleration()){
	   /*ROS_INFO_STREAM("Orientation X,Y,Z,W = "	<< sensors_[0].getOrientation()[0] <<", "
							<< sensors_[0].getOrientation()[1] <<", "
							<< sensors_[0].getOrientation()[2] <<", "
							<< sensors_[0].getOrientation()[3] <<", "
	   );*/

	const double* acc = sensors_[0].getLinearAcceleration();
	//std::valarray<double> newacc(acc);
	//std::cout<< sizeof *acc / sizeof acc[0] << std::endl;
	//std::vector<double> acc{ sensors_[0].getLinearAcceleration() };

        /*if (loop_count_ > 1000){
 	  for (unsigned int i=0;i<3;++i){
          //linearAcc[i] += acc[i];
 	  //if (loop_count_%updateAcc == 0){
 	    //linearAcc[i] /= updateAcc;
	    //std::cout << (endAcc-startAcc) << std::endl;
	    //std::cout << linearAcc[i] << " ";

            linearDisFromAcc[i] = linearDisFromAcc[i] + linearVelFromAcc[i]*dur_t + acc[i]*dur_t*dur_t/2;
            //linearDisFromAcc[i] = linearDisFromAcc[i] + linearVelFromAcc[i]*dur_t;
            linearVelFromAcc[i] = linearVelFromAcc[i] + acc[i]*dur_t;

            //std::cout << "Linear Vel " << i << ": " << linearVelFromAcc[i] << "  " << 
            //     "Linear Dis " << i << ": " << linearDisFromAcc[i] << std::endl;
 	  //}
	  //std::cout << acc[i] << " " << linearAcc[i] << " ";
	  }
        }*/

        //if (loop_count_%updateAcc == 0){
        //std::cout<< std::endl;
	//}
	//std::cout<< linearAcc[0] << " " << linearAcc[1] << " " << linearAcc[2] << std::endl;

	//ROS_INFO_STREAM("Orientation RPY = "	<< roll*(180.0/3.14159)<<"," <<pitch*(180.0/3.14159)<<", "<<yaw*(180.0/3.14159));

	
    }
    else{
 	 ROS_WARN("No linear acceleration data");
    }	

    /*--------------------------------------------------------------------------------------*/
    // Get joint position (assume encoder)  (and true joint velocity for testing)
    //std::cout << "Joint vel: ";
    for(unsigned int i=0; i<n_joints_;++i){
      jointPos[i] = joints_[i].getPosition();
      truejointVel[i] = joints_[i].getVelocity();
    }

    /*--------------------------------------------------------------------------------------*/
    // Estimate joint velocity
    // Update time deque
    timeUpdated = cummulativeTimeUpdate(time_ms, 15, curTime_msec);

    // Estimate joint velocity (with time interval of 10 loops)
    getVel(jointVel, jointPosCummulative, timeUpdated, 10, jointPos, time_ms);

    // Estimate roll, pitch and yaw velocity (with time interval of 10 loops)
    getVel(rpyVel, rpyCummulative, timeUpdated, 10, rpyImu, time_ms);

    /*--------------------------------------------------------------------------------------*/
    // Get the angle position and velocity of the motor in world frame
    linksAngleAndVel(linksAngWithVert,linksAngVel,  jointPos,jointVel, rpyImu,rpyVel);

    /*--------------------------------------------------------------------------------------*/
    // Get leg tip force in world frame
    legTipForce(tipForce,  linksAngWithVert,jointPos,springCoef);

    /*--------------------------------------------------------------------------------------*/
    // Get linear velocity in world frame from joint position and velocity
    rightStandForLinearVel(rightStand,tipForce); // determine if right(or left) leg is standing

    getLinearVelFromLink(linearVelFromLink, rightStand, linksAngWithVert, linksAngVel);

    /*--------------------------------------------------------------------------------------*/
    // Get the updated set of joints for the motor to move to
    double interval = 0.2;
    bool stop = ((curTime-startTime).toSec()) < interval;

    update_control(prevRightStandControl, commands, xyTipPos, xyTipPosTarget, aveLinearVel, linearVelFromLink, stop, rpyImu, time, startTime.toSec()+interval);

    // Uncomment below for free pitch control
/*
    bool temp = prevRightStandControl;
    update_control_free(prevRightStandControl, swang, walkingState, targetPitch, controlPitch, commands,  xyTipPos,  xyTipPosTarget, aveLinearVel, linearVelFromLink, stop, loop_count_, rpyImu, rpyVel, linksAngWithVert, time, startTime.toSec()+interval);

    // Change PID parameters based on stance or swing phase
    if(temp != prevRightStandControl){
      pid_controllers_[prevRightStandControl].setGains(abadStancePid[0],abadStancePid[1], abadStancePid[2],0.0,0.0,false);
      pid_controllers_[(prevRightStandControl+1)%2].setGains(abadSwingPid[0], abadSwingPid[1], abadSwingPid[2], 0.0,0.0,false);
      pid_controllers_[prevRightStandControl*4+2].setGains(springStancePid[0], springStancePid[1], springStancePid[2], 0.0,0.0,false);
      pid_controllers_[prevRightStandControl*4+3].setGains(springStancePid[0], springStancePid[1], springStancePid[2], 0.0,0.0,false);
      pid_controllers_[((prevRightStandControl+1)%2)*4+2].setGains(springSwingPid[0], springSwingPid[1], springSwingPid[2], 0.0,0.0,false);
      pid_controllers_[((prevRightStandControl+1)%2)*4+3].setGains(springSwingPid[0], springSwingPid[1], springSwingPid[2], 0.0,0.0,false);
    }
*/

    /*--------------------------------------------------------------------------------------*/
    for(unsigned int i=0; i<n_joints_; i++)
    {
        double command_position = commands[i];

        double error;
        double commanded_effort;

        double current_position = jointPos[i];

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

        // Make sure the commanded torque is within allowable range
        commanded_effort = std::max(-max_torque, std::min(max_torque, commanded_effort));

        joints_[i].setCommand(commanded_effort);
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
