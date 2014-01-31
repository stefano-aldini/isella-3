#include <ros/ros.h>
#include <sys/time.h>
#include "ISELLA3_Position_Control.h"
#include "ISELLA3_ArmModule.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <std_srvs/Empty.h>
#include "isella3/MyStateMessage.h"

namespace my_controller_ns{

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Isella3Controller
{
private:

  unsigned long last_time;
  double d_last_time;
 
  Vector6d x1;
  Vector6d x2;
  Vector6d x3;
  Vector6d x4;
  double qd1;
  double qd2;
  double qd3;
  double qd4;
  float kp1,ki1,kd1,kp2,ki2,kd2,kp3,ki3,kd3,kp4,ki4,kd4;
  
public:
  bool init(ros::NodeHandle &n);
  void starting(double time_fin, double current_time);
  void update(double d_last_time);
  void stopping();
  Vector6d Trajectory(double desired_pos, unsigned long time_in, unsigned long time_fin, int iArmModuleId, int iAxisId);
};
}
