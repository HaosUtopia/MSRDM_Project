#ifndef TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H

#include <cmath>
#include <vector>
#include <utility>
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
  TrajGen(const ros::NodeHandle& nh, double delta_t);
  ~TrajGen();
  
  bool getCurrPos(double& ox, double& oy, bool& oled);
  
protected:
  struct Position
  {
    double x;
    double y;
    bool led;
    
    Position(double ix, double iy, bool iled) : x(ix), y(iy), led(iled) {}
    Position() : x(0.0), y(0.0), led(false) {}
  };
  
private:
  void lowPassFilter(double& value, const double& prev_value);
  void getPointCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
  bool startTrajCallback(std_srvs::Empty::Request& req,
                         std_srvs::Empty::Response& resp);
  
  bool started;
  
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
  double window_min_v; // minimum velocity
  double window_max_v; // maximum velocity
  double window_min_ds; // minimum distance per step
  double window_max_ds; // maximum distance per step
  double window_max_d; // maximum distance inside a trajectory
  
  double point_x;
  double point_y;
  double window_x;
  double window_y;
  double last_window_x;
  double last_window_y;
  
  double distance;
  
  ros::NodeHandle root_nh;
  ros::NodeHandle traj_nh;
  
  boost::mutex traj_lock;
  
  std::vector<Position> trajectory;
  
  ros::Subscriber sub_point;
  ros::ServiceServer srv_start;

};

}

#endif // TRAJECTORY_GENERATOR_TRAJECTORY_GENERATOR_H
