#ifndef TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H

#include <cmath>
#include <nlopt.hpp>
#include <vector>
#include <utility>
#include <algorithm>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>

// Touchpad message type
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/Empty.h>

namespace trajectory_generator
{

class TrajGen
{
public:
  TrajGen(const ros::NodeHandle& nh);
  ~TrajGen();
  
  bool getCurrPos(double& ox, double& oy, double& ovx, double& ovy, double& oax, double& oay, bool& oled);
  
protected:
  struct Position
  {
    double x;
    double y;
    double vx;
    double vy;
    double ax;
    double ay;
    bool led;
    
    Position(double ix, double iy, double ivx, double ivy, double iax, double iay, bool iled) : x(ix), y(iy), vx(ivx), vy(ivy), ax(iax), ay(iay), led(iled) {}
    Position() : x(0.0), y(0.0), vx(0.0), vy(0.0), ax(0.0), ay(0.0), led(false) {}
  };
  
private:
  static double costFunction(const std::vector<double> &a, std::vector<double> &grad, void* param);
  static double accConstraint(const std::vector<double> &a, std::vector<double> &grad, void* param);
  static double velConstraint(const std::vector<double> &a, std::vector<double> &grad, void* param);
  void optProcess();
  void getPointCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
  // bool startTrajCallback(std_srvs::Empty::Request& req,
  //                        std_srvs::Empty::Response& resp);
  
  // bool started;
  static double delta_t;
  
  double point_min_x;
  double point_max_x;
  double point_min_y;
  double point_max_y;
  double window_min_x;
  double window_max_x;
  double window_min_y;
  double window_max_y;
  double window_init_x;
  double window_init_y;
  double window_max_a; // maximum acceleration
  double window_min_v; // minimum velocity
  double window_max_v; // maximum velocity
  double window_normal_v; // normal velocity
  double window_min_ds; // minimum distance per step
  double window_max_ds; // maximum distance per step
  double window_max_d; // maximum distance inside a trajectory
  
  double point_x;
  double point_y;
  double window_x;
  double window_y;
  double window_vx;
  double window_vy;
  double window_ax;
  double window_ay;
  double last_window_x;
  double last_window_y;
  double last_window_vx;
  double last_window_vy;
  double last_window_ax;
  double last_window_ay;
  double curr_window_x;
  double curr_window_y;
  
  double window_d;
  double window_a;
  
  double* cost_param;
  double** acc_param;
  double** vel_param;
  int optimize_num;
  std::vector<double> acc;
  std::vector<Position> opt_buffer;
  nlopt::opt optimizer;
  
  ros::NodeHandle root_nh;
  ros::NodeHandle traj_nh;
  
  boost::mutex traj_lock;
  
  std::vector<Position> trajectory;
  
  ros::Subscriber sub_point;
  // ros::ServiceServer srv_start;

};

double TrajGen::delta_t = 0.02;

}

#endif // TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H
