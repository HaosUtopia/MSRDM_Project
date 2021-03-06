#include <trajectory_generator/trajectory_generator.h>
#include <trajectory_generator/TrajectoryPosition.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  double rate = 50;
  
  if (!nh.getParam("trajectory_generator/publish_rate", rate))
  {
    ROS_ERROR_STREAM("Could not find 'trajectory_generator/publish_rate' parameter (namespace: " << nh.getNamespace() << ").");
    return 0;
  }
  
  ros::Rate r(rate);
  
  ros::Publisher pub_position = nh.advertise<trajectory_generator::TrajectoryPosition>("trajectory_generator/position", 100);
  
  trajectory_generator::TrajGen node(nh);
  
  trajectory_generator::TrajectoryPosition msg;
  double x;
  double y;
  double vx;
  double vy;
  double ax;
  double ay;
  bool led;
  
  while(ros::ok())
  {
    if (node.getCurrPos(x, y, vx, vy, ax, ay, led))
    {
      msg.x = x;
      msg.y = y;
      msg.vx = vx;
      msg.vy = vy;
      msg.ax = ax;
      msg.ay = ay;
      msg.led = led;
      
      pub_position.publish(msg);
    }
    
    ros::spinOnce();
    
    r.sleep();
  
  }
}
