#include <trajectory_generator/trajectory_generator.h>

namespace trajectory_generator
{

TrajGen::TrajGen(const ros::NodeHandle& nh)
  : root_nh(nh)
  , traj_nh(nh, "trajectory_generator")
  // , started(false)
{
  ROS_INFO("Initializing trajectory generator...");
  
  bool ok = true;
  
  double publish_rate;
  if (!traj_nh.getParam("publish_rate", publish_rate))
  {
    ROS_ERROR_STREAM("Could not find 'publish_rate' parameter (namespace: " << traj_nh.getNamespace() << ").");
    ok = false;
  }
  
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
  
  delta_t = 1.0 / publish_rate;
  window_min_ds = window_min_v * delta_t;
  window_max_ds = window_max_v * delta_t;
  window_max_a = window_max_v / delta_t;
  last_window_x = window_init_x;
  last_window_y = window_init_y;
  last_window_vx = 0.0;
  last_window_vy = 0.0;
  last_window_ax = 0.0;
  last_window_ay = 0.0;
  window_v = 0.0;
  
  sub_point = root_nh.subscribe("finger_position", 100, &TrajGen::getPointCallback, this);
  // srv_start = traj_nh.advertiseService("trajectory_start", &TrajGen::startTrajCallback, this);
  
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
  else if(sqrt(pow(last_window_vx, 2) + pow(last_window_vy, 2)) > 0.0)
  {
    ROS_INFO("stop the trajectory");
    window_vx = 0.0;
    window_vy = 0.0;
    last_window_ax = (window_vx - last_window_vx) / delta_t;
    last_window_ay = (window_vx - last_window_vy) / delta_t;
    window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      
    if (window_a > window_max_a)
    {
      ROS_WARN_STREAM("Acceleration is: " << window_a);
    }
    
    ox = last_window_x;
    oy = last_window_y;
    ovx = last_window_vx;
    ovy = last_window_vy;
    oax = last_window_ax;
    oay = last_window_ay;
    oled = false;
    
    last_window_x += last_window_vx * delta_t + last_window_ax * delta_t * delta_t / 2;
    last_window_y += last_window_vy * delta_t + last_window_ay * delta_t * delta_t / 2;
    last_window_vx = window_vx;
    last_window_vy = window_vy;
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
  
  if (point_x == 10000)
  {
    last_window_x = trajectory.front().x;
    last_window_y = trajectory.front().y;
    last_window_vx = trajectory.front().vx;
    last_window_vy = trajectory.front().vy;
    last_window_ax = trajectory.front().ax;
    last_window_ay = trajectory.front().ay;
    trajectory.clear();
    trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, false));
    trajectory.push_back(Position(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, false));
    return;
  }
  
  window_d = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
  
  if (window_d > window_max_d)
  {
    window_v = window_max_v;
    window_vx = window_v * (window_x - last_window_x) / window_d;
    window_vy = window_v * (window_y - last_window_y) / window_d;
    last_window_ax = (window_vx - last_window_vx) / delta_t;
    last_window_ay = (window_vy - last_window_vy) / delta_t;
    window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));

    while (window_a > window_max_a)
    {
      ROS_WARN_STREAM("Acceleration is: " << window_a);
      ROS_INFO_STREAM("Velocity is: " << window_v);
      window_v = window_v * 0.8;
      window_vx = window_v * (window_x - last_window_x) / window_d;
      window_vy = window_v * (window_y - last_window_y) / window_d;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
    }
      
    trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, true));
      
    last_window_vx = window_vx;
    last_window_vy = window_vy;
    last_window_x += window_max_ds * (window_x - last_window_x) / window_d;
    last_window_y += window_max_ds * (window_y - last_window_y) / window_d;
    window_d = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
      
    while (window_d > window_max_ds)
    {
      window_v = window_max_v;
      window_vx = window_v * (window_x - last_window_x) / window_d;
      window_vy = window_v * (window_y - last_window_y) / window_d;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));

      while (window_a > window_max_a)
      {
        ROS_WARN_STREAM("Acceleration is: " << window_a);
        ROS_INFO_STREAM("Velocity is: " << window_v);
        window_v = window_v * 0.8;
        window_vx = window_v * (window_x - last_window_x) / window_d;
        window_vy = window_v * (window_y - last_window_y) / window_d;
        last_window_ax = (window_vx - last_window_vx) / delta_t;
        last_window_ay = (window_vy - last_window_vy) / delta_t;
        window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      }
      
      trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, false));
      
      last_window_vx = window_vx;
      last_window_vy = window_vy;
      last_window_x += window_max_ds * (window_x - last_window_x) / window_d;
      last_window_y += window_max_ds * (window_y - last_window_y) / window_d;
      window_d = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
    }
    
    if (window_d > window_min_ds)
    {
      window_vx = (window_x - last_window_x) / delta_t;
      window_vy = (window_y - last_window_y) / delta_t;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      
      if (window_a > window_max_a)
      {
        ROS_WARN_STREAM("Acceleration is: " << window_a);
      }
      
      trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, false));
      
      last_window_vx = window_vx;
      last_window_vy = window_vy;
      last_window_x = window_x;
      last_window_y = window_y;
    }
    
  }
  else
  {
    while (window_d > window_max_ds)
    {
      window_v = window_max_v;
      window_vx = window_v * (window_x - last_window_x) / window_d;
      window_vy = window_v * (window_y - last_window_y) / window_d;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      
      while (window_a > window_max_a)
      {
        ROS_WARN_STREAM("Acceleration is: " << window_a);
        ROS_INFO_STREAM("Velocity is: " << window_v);
        window_v = window_v * 0.8;
        window_vx = window_v * (window_x - last_window_x) / window_d;
        window_vy = window_v * (window_y - last_window_y) / window_d;
        last_window_ax = (window_vx - last_window_vx) / delta_t;
        last_window_ay = (window_vy - last_window_vy) / delta_t;
        window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      }
      
      trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, true));
      
      last_window_vx = window_vx;
      last_window_vy = window_vy;
      last_window_x += window_max_ds * (window_x - last_window_x) / window_d;
      last_window_y += window_max_ds * (window_y - last_window_y) / window_d;
      window_d = sqrt(pow(window_y - last_window_y, 2) + pow(window_x - last_window_x, 2));
    }
    
    if (window_d > window_min_ds)
    {
      window_vx = (window_x - last_window_x) / delta_t;
      window_vy = (window_y - last_window_y) / delta_t;
      last_window_ax = (window_vx - last_window_vx) / delta_t;
      last_window_ay = (window_vy - last_window_vy) / delta_t;
      window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      
      trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, true));
      
      last_window_vx = window_vx;
      last_window_vy = window_vy;
      last_window_x = window_x;
      last_window_y = window_y;
    }
  }
}
// bool TrajGen::startTrajCallback(std_srvs::Empty::Request& req,
//                                 std_srvs::Empty::Response& resp)
// {
//   started = true;
//   
//   return true;
// }

}
