#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/joint.h>
#include <ros/ros.h>
#include <isella3/SetAmplitude.h>
#include <control_toolbox/pid.h>
#include <std_srvs/Empty.h>
#include "isella3/MyStateMessage.h"


namespace my_controller_ns{

enum
{
  StoreLen = 5000
};

class Isella3Controller: public pr2_controller_interface::Controller
{
private:
  bool setAmplitude(isella3::SetAmplitude::Request& req,
                    isella3::SetAmplitude::Response& resp);

  pr2_mechanism_model::JointState* joint_state_;
  double init_pos_;
  double amplitude_;
  ros::ServiceServer srv_;
  pr2_mechanism_model::RobotState *robot_;
  control_toolbox::Pid pid_controller_;
  ros::Time time_of_last_cycle_;
  bool capture(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& resp);
  ros::ServiceServer capture_srv_;
  ros::Publisher     mystate_pub_;
  isella3::MyStateMessage  storage_[StoreLen];
  volatile int storage_index_;

public:
  virtual bool init(pr2_mechanism_model::RobotState *robot,
                   ros::NodeHandle &n);
  virtual void starting();
  virtual void update();
  virtual void stopping();
};
}
