#include <trajectory_generator/trajectory_generator.h>

namespace trajectory_generator
{

TrajGen::TrajGen(const ros::NodeHandle& nh, double dt)
  : root_nh(nh)
  , traj_nh(nh, "trajectory_generator")
  , started(false)
  , delta_t(dt)
{
  ROS_INFO("Initializing trajectory generator...");
  
  bool ok = true;
  
  if (!traj_nh.getParam("point_min_x", point_min_x))
  {
    ROS_ERROR_STREAM("Could not find 'point_min_x' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("point_max_x", point_max_x))
  {
    ROS_ERROR_STREAM("Could not find 'point_max_x' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("point_min_y", point_min_y))
  {
    ROS_ERROR_STREAM("Could not find 'point_min_y' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("point_max_y", point_max_y))
  {
    ROS_ERROR_STREAM("Could not find 'point_max_y' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_min_x", window_min_x))
  {
    ROS_ERROR_STREAM("Could not find 'window_min_x' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_max_x", window_max_x))
  {
    ROS_ERROR_STREAM("Could not find 'window_max_x' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_min_y", window_min_y))
  {
    ROS_ERROR_STREAM("Could not find 'window_min_y' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_max_y", window_max_y))
  {
    ROS_ERROR_STREAM("Could not find 'window_max_y' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_init_x", window_init_x))
  {
    ROS_ERROR_STREAM("Could not find 'window_init_x' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_init_y", window_init_y))
  {
    ROS_ERROR_STREAM("Could not find 'window_init_y' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_min_v", window_min_v))
  {
    ROS_ERROR_STREAM("Could not find 'window_min_v' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_max_v", window_max_v))
  {
    ROS_ERROR_STREAM("Could not find 'window_max_v' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  if (!traj_nh.getParam("window_max_d", window_max_d))
  {
    ROS_ERROR_STREAM("Could not find 'window_max_d' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
  window_min_ds = window_min_v * delta_t;
  window_max_ds = window_max_v * delta_t;
  last_window_x = window_init_x;
  last_window_y = window_init_y;
  last_window_vx = 0.0;
  last_window_vy = 0.0;
  last_window_ax = 0.0;
  last_window_ay = 0.0;
  
  sub_point = root_nh.subscribe("finger_position", 100, &TrajGen::getPointCallback, this);
  srv_start = traj_nh.advertiseService("trajectory_start", &TrajGen::startTrajCallback, this);
  
  if (ok)
  {
    ROS_INFO("Initialized trajectory generator successfully");
  }
  else
  {
    ROS_INFO("Initialized trajectory generator with some problems");
  }
}

TrajGen::~TrajGen()
{
}

bool TrajGen::getCurrPos(double& ox, double& oy, double& ovx, double& ovy, double& oax, double& oay, bool& oled)
{
  boost::mutex::scoped_lock guard(traj_lock);
  
  if (!trajectory.empty())
  {
    ox = trajectory.front().x;
    oy = trajectory.front().y;
    ovx = trajectory.front().vx;
    ovy = trajectory.front().vy;
    oax = trajectory.front().ax;
    oay = trajectory.front().ay;
    oled = trajectory.front().led;
    
    trajectory.erase(trajectory.begin());
    
    return true;
  }
  else
  {
    return false;
  }
}

void TrajGen::lowPassFilter(double& value, const double& pre_value)
{
  value = (value + pre_value) / 2;
}

void TrajGen::getPointCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
  // if (!started)
  // {
  //   return;
  // }
  
  point_x = msg->data[0];
  point_y = msg->data[1];
  window_x = (point_x - point_min_x) / (point_max_x - point_min_x) * (window_max_x - window_min_x) + window_min_x;
  window_y = (point_y - point_min_y) / (point_max_y - point_min_y) * (window_max_y - window_min_y) + window_min_y;
  
  boost::mutex::scoped_lock guard(traj_lock);
  
  distance = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
  
  if (distance > window_max_d)
  {
    while (distance > window_max_ds)
    {
      last_window_x += window_max_ds * (window_x - last_window_x) / distance;
      last_window_y += window_max_ds * (window_y - last_window_y) / distance;
      window_vx = window_max_v * (window_x - last_window_x) / distance;
      window_vy = window_max_v * (window_y - last_window_y) / distance;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      last_window_vx = window_vx;
      last_window_vy = window_vy;
      
      trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, false));
      distance = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
    }
  }
  else if (distance > window_max_ds)
  {
    while (distance > window_max_ds)
    {
      last_window_x += window_max_ds * (window_x - last_window_x) / distance;
      last_window_y += window_max_ds * (window_y - last_window_y) / distance;
      window_vx = window_max_v * (window_x - last_window_x) / distance;
      window_vy = window_max_v * (window_y - last_window_y) / distance;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      last_window_vx = window_vx;
      last_window_vy = window_vy;
    
      trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, true));
      distance = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
    }
  }
  
  if (distance > window_min_ds)
  {
    last_window_x = window_x;
    last_window_y = window_y;
    window_vx = window_max_v * (window_x - last_window_x) / distance;
    window_vy = window_max_v * (window_y - last_window_y) / distance;
    last_window_ax = (window_vx - last_window_vx) / delta_t;
    last_window_ay = (window_vy - last_window_vy) / delta_t;
    last_window_vx = window_vx;
    last_window_vy = window_vy;

    trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, true));
  }
}

bool TrajGen::startTrajCallback(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp)
{
  started = true;
  
  return true;
}

}
