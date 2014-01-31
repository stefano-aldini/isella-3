#include "isella3_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace my_controller_ns;


/// Controller initialization in non-realtime
bool Isella3Controller::init(pr2_mechanism_model::RobotState *robot,
                            ros::NodeHandle &n)
{
  std::string joint_name;
  if (!n.getParam("joint_name", joint_name))
  {
    ROS_ERROR("No joint given in namespace: '%s')",
              n.getNamespace().c_str());
    return false;
  }

  joint_state_ = robot->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("MyController could not find joint named '%s'",
              joint_name.c_str());
    return false;
  }
  
  srv_ = n.advertiseService("set_amplitude", 
                            &Isella3Controller::setAmplitude, this);

  amplitude_ = 0.5;

  // copy robot pointer so we can access time
  robot_ = robot;

  // construct pid controller
  if (!pid_controller_.init(ros::NodeHandle(n, "pid_parameters"))){
    ROS_ERROR("MyController could not construct PID controller for joint '%s'",
              joint_name.c_str());
    return false;
  }

  capture_srv_ = n.advertiseService("capture", &Isella3Controller::capture, this);
  mystate_pub_ = n.advertise<isella3::MyStateMessage>("mystate_topic", StoreLen);
  storage_index_ = StoreLen;

  return true;
}

/// Controller startup in realtime
void Isella3Controller::starting()
{
  init_pos_ = joint_state_->position_;
  time_of_last_cycle_ = robot_->getTime();
  pid_controller_.reset();
}


/// Controller update loop in realtime
void Isella3Controller::update()
{
  double desired_pos = init_pos_ + amplitude_ * sin(ros::Time::now().toSec());
  double current_pos = joint_state_->position_;
  ros::Duration dt = robot_->getTime() - time_of_last_cycle_;
  time_of_last_cycle_ = robot_->getTime();
  joint_state_->commanded_effort_ = pid_controller_.updatePid(current_pos-desired_pos, dt);

  int index = storage_index_;
  if ((index >= 0) && (index < StoreLen))
    {
      storage_[index].dt               = 0.0;
      storage_[index].position         = joint_state_->position_;
      storage_[index].desired_position = desired_pos;
      storage_[index].velocity         = joint_state_->velocity_;
      storage_[index].desired_velocity = 0.0;
      storage_[index].commanded_effort = joint_state_->commanded_effort_;
      storage_[index].measured_effort  = joint_state_->measured_effort_;

      // Increment for the next cycle.
      storage_index_ = index+1;
     }
}


/// Controller stopping in realtime
void Isella3Controller::stopping()
{}

/// Service call to set amplitude of sin
bool Isella3Controller::setAmplitude(isella3::SetAmplitude::Request& req,
                                     isella3::SetAmplitude::Response& resp)
{
  if (fabs(req.amplitude) < 2.0){
    amplitude_ = req.amplitude;
    ROS_INFO("Mycontroller: set amplitude too %f", req.amplitude);
  }
  else
    ROS_WARN("MyController: requested amplitude %f too large", req.amplitude);

  resp.amplitude = amplitude_;
  return true;
}

/// Service call to capture and extract the data
bool Isella3Controller::capture(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp)
{
  /* Record the starting time. */
  ros::Time started = ros::Time::now();

  /* Mark the buffer as clear (which will start storing). */
  storage_index_ = 0;

  /* Now wait until the buffer is full. */
  while (storage_index_ < StoreLen)
    {
      /* Sleep for 1ms as not to hog the CPU. */
      ros::Duration(0.001).sleep();

      /* Make sure we don't hang here forever. */
      if (ros::Time::now() - started > ros::Duration(20.0))
        {
          ROS_ERROR("Waiting for buffer to fill up took longer than 20 seconds!");
          return false;
        }
    }

  /* Then we can publish the buffer contents. */
  int  index;
  for (index = 0 ; index < StoreLen ; index++)
    mystate_pub_.publish(storage_[index]);

  return true;
}



/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(isella3,MyControllerPlugin, my_controller_ns::Isella3Controller, pr2_controller_interface::Controller)
