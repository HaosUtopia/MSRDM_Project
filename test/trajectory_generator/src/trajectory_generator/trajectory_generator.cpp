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
  
  if (!traj_nh.getParam("optimize_num", optimize_num))
  {
    ROS_ERROR_STREAM("Could not find 'optimize_num' parameter (namespace: " << traj_nh.getNamespace() << ").");
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
  
  if (!traj_nh.getParam("window_max_a", window_max_a))
  {
    ROS_ERROR_STREAM("Could not find 'window_max_a' parameter (namespace: " << traj_nh.getNamespace() << ").");
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
  window_normal_v = window_max_v * 0.8;
  last_window_x = window_init_x;
  last_window_y = window_init_y;
  last_window_vx = 0.0;
  last_window_vy = 0.0;
  last_window_ax = 0.0;
  last_window_ay = 0.0;
  curr_window_x = window_init_x;
  curr_window_y = window_init_y;
  
  ROS_INFO("Initializing opetimizer parameters...");
  cost_param = new double[optimize_num * 2 + 4];
  acc_param = new double*[optimize_num];
  vel_param = new double*[optimize_num];
  
  for (unsigned int i = 0; i < optimize_num; ++i)
  {
    acc_param[i] = new double[2];
    vel_param[i] = new double[4];
    
    acc_param[i][0] = i;
    acc_param[i][1] = window_max_a;
    vel_param[i][0] = i;
    vel_param[i][3] = window_max_v;
  }
  
  opt_buffer.push_back(Position(window_init_x, window_init_y, 0.0, 0.0, 0.0, 0.0, false));
  
  ROS_INFO("Initialized opetimizer parameters successfully");
  
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
  for (unsigned int i = 0; i < optimize_num; ++i)
  {
    delete [] acc_param[i];
    delete [] vel_param[i];
  }
  
  delete [] cost_param;
  delete [] acc_param;
  delete [] vel_param;
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
  else if (!opt_buffer.empty())
  {
    optProcess();
    
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
    last_window_ax = (0.0 - last_window_vx) / delta_t;
    last_window_ay = (0.0 - last_window_vy) / delta_t;
    window_a = sqrt(pow(last_window_ax, 2) + pow(last_window_ay, 2));
      
    if (window_a > window_max_a)
    {
      ROS_WARN_STREAM("Acceleration is: " << window_a);
      last_window_ax *= window_max_a / window_a;
      last_window_ay *= window_max_a / window_a;
    }
    
    last_window_x += last_window_vx * delta_t + last_window_ax * delta_t * delta_t / 2;
    last_window_y += last_window_vy * delta_t + last_window_ay * delta_t * delta_t / 2;
    last_window_vx += last_window_ax * delta_t;
    last_window_vy += last_window_ay * delta_t;
    curr_window_x = last_window_x;
    curr_window_y = last_window_y;
    
    ox = last_window_x;
    oy = last_window_y;
    ovx = last_window_vx;
    ovy = last_window_vy;
    oax = last_window_ax;
    oay = last_window_ay;
    oled = false;
  }
  else
  {
    return false;
  }
}

double TrajGen::costFunction(const std::vector<double> &a, std::vector<double> &grad, void* param)
{
  double* data = reinterpret_cast<double*>(param);
  
  double cost = 0.0;
  std::vector<double> x;
  std::vector<double> v;
  
  x.push_back(data[0]);
  x.push_back(data[1]);
  v.push_back(data[2]);
  v.push_back(data[3]);
  
  for (unsigned int i = 0; i < a.size() / 2; ++i)
  {
    x.push_back(x[2*i] + v[2*i] * delta_t + a[2*i] * delta_t * delta_t / 2);
    x.push_back(x[2*i+1] + v[2*i+1] * delta_t + a[2*i+1] * delta_t * delta_t / 2);
    v.push_back(v[2*i] + a[2*i] * delta_t);
    v.push_back(v[2*i+1] + a[2*i+1] * delta_t);
  }
  
  if (!grad.empty())
  {
    for (unsigned int i = 0; i < a.size() / 2; ++i)
    {
      grad[2*i] = 0.0;
      grad[2*i+1] = 0.0;
      
      for (unsigned int j = a.size() / 2; j > i; --j)
      {
        grad[2*i] += 2 * (x[2*j] - data[2*j+2]) * delta_t * delta_t * (2*(j-i)-1) / 2;
        grad[2*i+1] += 2 * (x[2*j+1] - data[2*j+3]) * delta_t * delta_t * (2*(j-i)-1) / 2;
//        ROS_INFO_STREAM("i = " << i);
//        ROS_INFO_STREAM("j = " << j);
//        ROS_INFO_STREAM("grad[" << 2*i << "] = " <<  grad[2*i]);
//        ROS_INFO_STREAM("grad[" << 2*i+1 << "] = " <<  grad[2*i+1]);
//        ROS_INFO_STREAM("x" << j << "= " << x[2*j]);
//        ROS_INFO_STREAM("xr" << j << "= " << data[2*j+2]);
//        ROS_INFO_STREAM("y" << j << "= " << x[2*j+1]);
//        ROS_INFO_STREAM("yr" << j << "= " << data[2*j+3]);
//        ROS_INFO_STREAM("a[0] = " << a[0]);
//        ROS_INFO_STREAM("a[1] = " << a[1]);
//        ROS_INFO_STREAM("a[2] = " << a[2]);
//        ROS_INFO_STREAM("a[3] = " << a[3]);
      }
    }
  }
  
  for (unsigned int i = 0; i < a.size() / 2; ++i)
  {
    cost += pow(data[2*i+4] - x[2*i+2], 2);
    cost += pow(data[2*i+5] - x[2*i+3], 2);
  }
  
  return cost;
}

double TrajGen::accConstraint(const std::vector<double> &a, std::vector<double> &grad, void* param)
{
  double* data = reinterpret_cast<double*>(param);
  double index = data[0];
  double acc_max = data[1];
  double acc_sum = sqrt(pow(a[2*index], 2) + pow(a[2*index+1], 2));
  
  if (!grad.empty())
  {
    for (unsigned int i = 0; i < a.size() / 2; ++i)
    {
      if (i == index)
      {
        if (acc_sum != 0.0)
        {
          grad[2*i] = a[2*i] / acc_sum;
          grad[2*i+1] = a[2*i+1] / acc_sum;
        }
        else
        {
          grad[2*i] = 0.0;
          grad[2*i+1] = 0.0;
        }
      }
      else
      {
        grad[2*i] = 0.0;
        grad[2*i+1] = 0.0;
      }
    }
  }
  
  return acc_sum - acc_max;
}

double TrajGen::velConstraint(const std::vector<double> &a, std::vector<double> &grad, void* param)
{
  double* data = reinterpret_cast<double*>(param);
  double index = data[0] + 1;
  double vel_x = data[1];
  double vel_y = data[2];
  double vel_max = data[3];
  
  for (unsigned int i = 0; i < index; ++i)
  {
    vel_x += a[2*i] * delta_t;
    vel_y += a[2*i+1] * delta_t;
  }
  
  double vel_sum = sqrt(pow(vel_x, 2) + pow(vel_y, 2));
  
  if (!grad.empty())
  {
    for (unsigned int i = 0; i < a.size() / 2; ++i)
    {
      if (i < index)
      {
        if (vel_sum != 0.0)
        {
          grad[2*i] = vel_x * delta_t / vel_sum;
          grad[2*i+1] = vel_y * delta_t / vel_sum;
        }
        else
        {
          grad[2*i] = 0.0;
          grad[2*i+1] = 0.0;
        }
      }
      else
      {
        grad[2*i] = 0.0;
        grad[2*i+1] = 0.0;
      }
    }
  }
  
  return vel_sum - vel_max;
}

void TrajGen::optProcess()
{
  ROS_INFO("Run optimization...");
  
  if (opt_buffer.empty())
  {
    ROS_WARN("Buffer for optimization is empty, can not optimize");
    return;
  }
  
  optimizer = nlopt::opt(nlopt::LD_SLSQP, opt_buffer.size() * 2);
  acc.resize(opt_buffer.size() * 2);
  
  cost_param[0] = last_window_x;
  cost_param[1] = last_window_y;
  cost_param[2] = last_window_vx;
  cost_param[3] = last_window_vy;
  
  for (unsigned int i = 0; i < opt_buffer.size(); ++i)
  {
    cost_param[2*i+4] = opt_buffer[i].x;
    cost_param[2*i+5] = opt_buffer[i].y;
    ROS_INFO_STREAM("Buffer[" << i << "]:\nx = " << opt_buffer[i].x << "\ny = " << opt_buffer[i].y);
  }
  
//  ROS_INFO("Enter cost function...");
//  std::vector<double> grad;
//  grad.resize(acc.size());
//  delta_t = 1;
//  acc[0] = 1;
//  acc[1] = 0;
//  acc[2] = 1;
//  acc[3] = 2;
//  cost_param[0] = 0;
//  cost_param[1] = 0;
//  cost_param[2] = 0;
//  cost_param[3] = 0;
//  cost_param[4] = 1;
//  cost_param[5] = 0;
//  cost_param[6] = 2;
//  cost_param[7] = 2;
//  double k = costFunction(acc, grad, cost_param);
//  ROS_INFO_STREAM("Cost = " << k);
//  ROS_INFO_STREAM("grad[0] = " << grad[0]);
//  ROS_INFO_STREAM("grad[1] = " << grad[1]);
//  ROS_INFO_STREAM("grad[2] = " << grad[2]);
//  ROS_INFO_STREAM("grad[3] = " << grad[3]);
//  ROS_INFO("Leave cost function...");
  
  optimizer.set_min_objective(costFunction, cost_param);
  
  for (unsigned int i = 0; i < opt_buffer.size(); ++i)
  {
    vel_param[i][1] = last_window_vx;
    vel_param[i][2] = last_window_vy;
    
    optimizer.add_inequality_constraint(accConstraint, acc_param[i], 1e-8);
    optimizer.add_inequality_constraint(velConstraint, vel_param[i], 1e-8);
  }
  
  optimizer.set_xtol_rel(1e-4);
  double min_cost;
  
  try
  {
    nlopt::result result = optimizer.optimize(acc, min_cost);
    ROS_INFO_STREAM("Minimum cost: " << min_cost);
  }
  catch (std::exception &e)
  {
    ROS_ERROR_STREAM("Failed to find solution in optimization problem: " << e.what());
    return;
  }
  
  trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, acc[0], acc[1], opt_buffer.front().led));
  opt_buffer.erase(opt_buffer.begin());
  last_window_x += last_window_vx * delta_t + acc[0] * delta_t * delta_t / 2;
  last_window_y += last_window_vy * delta_t + acc[1] * delta_t * delta_t / 2;
  last_window_vx += acc[0] * delta_t;
  last_window_vy += acc[1] * delta_t;
  ROS_INFO("Optimization successfully");
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
    curr_window_x = last_window_x;
    curr_window_y = last_window_y;
    opt_buffer.clear();
    trajectory.clear();
    trajectory.push_back(Position(last_window_x, last_window_y, last_window_vx, last_window_vy, last_window_ax, last_window_ay, false));
    trajectory.push_back(Position(100.0, 0.0, 0.0, 0.0, 0.0, 0.0, false));
    return;
  }
  
  window_d = sqrt(pow(window_y - curr_window_y, 2) + pow(window_x - curr_window_x, 2));
  
  if (window_d > window_max_d)
  {   
    while (window_d > window_max_ds)
    {
      curr_window_x += window_normal_v * delta_t * (window_x - curr_window_x) / window_d;
      curr_window_y += window_normal_v * delta_t * (window_y - curr_window_y) / window_d;
      
      if (opt_buffer.size() < optimize_num)
      {
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, false));
      }
      else
      {
        optProcess();
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, false));
      }

      window_d = sqrt(pow(window_y - curr_window_y, 2) + pow(window_x - curr_window_x, 2));
    }
    
    if (window_d > window_min_ds)
    {
      curr_window_x = window_x;
      curr_window_y = window_y;
      
      if (opt_buffer.size() < optimize_num)
      {
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, false));
      }
      else
      {
        optProcess();
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, false));
      }
    }
  }
  else
  {
    while (window_d > window_max_ds)
    {
      curr_window_x += window_normal_v * delta_t * (window_x - curr_window_x) / window_d;
      curr_window_y += window_normal_v * delta_t * (window_y - curr_window_y) / window_d;
      
      if (opt_buffer.size() < optimize_num)
      {
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, true));
      }
      else
      {
        optProcess();
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, true));
      }

      window_d = sqrt(pow(window_y - curr_window_y, 2) + pow(window_x - curr_window_x, 2));
    }
    
    if (window_d > window_min_ds)
    {
      curr_window_x = window_x;
      curr_window_y = window_y;
      
      if (opt_buffer.size() < optimize_num)
      {
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, true));
      }
      else
      {
        optProcess();
        opt_buffer.push_back(Position(curr_window_x, curr_window_y, 0.0, 0.0, 0.0, 0.0, true));
      }
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
