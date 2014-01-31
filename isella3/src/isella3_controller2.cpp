#include "isella3_controller2.h"
#include <Eigen/Dense>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <fstream>

using namespace my_controller_ns;


/// Controller initialization in non-realtime
bool Isella3Controller::init(ros::NodeHandle &n)
{
  
  kp1 = 200; ki1 = 0.7; kd1 = 18;
  kp2 = 200; ki2 = 0.7; kd2 = 18;
  kp3 = 110; ki3 = 0.2; kd3 = 9;
  kp4 = 110; ki4 = 0.2; kd4 = 9;
  
  theApp.InitInstance(kp1,ki1,kd1,kp2,ki2,kd2,kp3,ki3,kd3,kp4,ki4,kd4);

  return true;
}

/// Controller startup in realtime
void Isella3Controller::starting(double time_fin, double d_last_time)
{

  ROS_INFO("update ---> d_last_time = %d, time_fin = %d", (int)d_last_time, (int)time_fin);
 
  theApp.OpenDevice();
  theApp.Enable();
  
  double qd1 = 150.0; //beta
  double qd2 = 180.0; //alpha
  double qd3 = 180.0;  //beta
  double qd4 = 180.0;  //alpha

  x1 = this->Trajectory(qd1,d_last_time, time_fin, 0, 0);
  x2 = this->Trajectory(qd2,d_last_time, time_fin, 0, 1);
  x3 = this->Trajectory(qd3,d_last_time, time_fin, 1, 0);
  x4 = this->Trajectory(qd4,d_last_time, time_fin, 1, 1);
  
  theApp.Move();

  return;
}

/// Controller update loop in realtime
void Isella3Controller::update(double d_last_time)
{
  double desired_pos1 = x1[0] + x1[1]*d_last_time + x1[2]*pow(d_last_time,2) + x1[3]*pow(d_last_time,3) + x1[4]*pow(d_last_time,4) + x1[5]*pow(d_last_time,5);
  double desired_pos2 = x2[0] + x2[1]*d_last_time + x2[2]*pow(d_last_time,2) + x2[3]*pow(d_last_time,3) + x2[4]*pow(d_last_time,4) + x2[5]*pow(d_last_time,5);
  double desired_pos3 = x3[0] + x3[1]*d_last_time + x3[2]*pow(d_last_time,2) + x3[3]*pow(d_last_time,3) + x3[4]*pow(d_last_time,4) + x3[5]*pow(d_last_time,5);
  double desired_pos4 = x4[0] + x4[1]*d_last_time + x4[2]*pow(d_last_time,2) + x4[3]*pow(d_last_time,3) + x4[4]*pow(d_last_time,4) + x4[5]*pow(d_last_time,5);

  std::cout << "\nDesired position 1: " << desired_pos1 << "\nDesired position 2: " << desired_pos2 << "\nDesired position 3: " << desired_pos3 << "\nDesired position 4: " << desired_pos4 << endl;

  std::cout << "\nActual position 1: " << theApp.fGetAngleAxisIs(0,0) << "\nActual position 2: " << theApp.fGetAngleAxisIs(0,1) << "\nActual position 3: " << theApp.fGetAngleAxisIs(1,0) << "\nActual position 4: " << theApp.fGetAngleAxisIs(1,1) << endl;

  theApp.SetAngleMust(0, 0, desired_pos1);
  theApp.SetAngleMust(0, 1, desired_pos2);
  theApp.SetAngleMust(1, 0, desired_pos3);
  theApp.SetAngleMust(1, 1, desired_pos4);
  theApp.OnTimer(1);

  return;
}


/// Controller stopping in realtime
void Isella3Controller::stopping()
{
  theApp.Disable();
  ROS_INFO("Device disabled");
  theApp.CloseDevice();
  ROS_INFO("Device closed");
}

/// Trajectory Planner
Vector6d Isella3Controller::Trajectory(double desired_pos, unsigned long time_in, unsigned long time_fin, int iArmModuleId, int iAxisId)
{    
  Vector6d b;
  Vector6d x;
  Eigen::Matrix<double, 6, 6> A;
  b << theApp.fGetAngleAxisIs(iArmModuleId,iAxisId), 0, 0, desired_pos, 0, 0;
  A << 1,time_in,pow(time_in,2),pow(time_in,3),pow(time_in,4),pow(time_in,5),
    0,1,2*time_in,3*pow(time_in,2),4*pow(time_in,3),5*pow(time_in,4),
    0,0,2,6*time_in,12*pow(time_in,2),20*pow(time_in,3),
    1,time_fin,pow(time_fin,2),pow(time_fin,3),pow(time_fin,4),pow(time_fin,5),
    0,1,2*time_fin,3*pow(time_fin,2),4*pow(time_fin,3),5*pow(time_fin,4),
    0,0,2,6*time_fin,12*pow(time_fin,2),20*pow(time_fin,3);
  x = A.inverse()*b;

  ROS_INFO("Trajectory ---> desired_pos = %d, AngleAxisIs = %d, time_in = %d, time_fin = %d", (int)desired_pos, (int)theApp.fGetAngleAxisIs(iArmModuleId,iAxisId), (int)time_in, (int)time_fin);
  std::cout << "Matrix A:\n" << A << "\nVector b:\n" << b << "\nVector x:\n" << x << "\nVector b = A*x:\n" << A*x << endl;

  return x;
}

int main(int argc, char ** argv)
{
	Isella3Controller isella3;
	ros::init(argc, argv, "isella_node");
	ROS_INFO("Node initialized");
	ros::NodeHandle n_;
	if(!isella3.init(n_))
	{
		ROS_ERROR("Cannot initialize!");
		return false;
	};

	unsigned long current_time = theApp.ulGetTime();
	unsigned long time_fin = 20000000 + current_time;
	ROS_INFO("Current Time = %d", (int)current_time);
	ROS_INFO("Final Time = %d", (int)time_fin);	
	isella3.starting((double)time_fin, (double)current_time);
	ROS_INFO("Controller started");

	while(ros::ok() && (current_time <= time_fin))
	{
		isella3.update((double)current_time);
		ROS_INFO("Still going to the desired position");
		current_time = theApp.ulGetTime();
		ROS_INFO("time ---------------------> %d", (int)current_time);

	};

	ROS_INFO("Stopping...");
	isella3.stopping();
	ROS_INFO("End");
};
