#include <trajectory_generator/trajectory_generator.h>
#include <trajectory_generator/TrajectoryPosition.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  
  double rate = 50;
  ros::Rate r(rate);
  
  ros::Publisher pub_position = nh.advertise<trajectory_generator::TrajectoryPosition>("trajectory_generator/position", 100);
  
  trajectory_generator::TrajGen node(nh, 1.0 / rate);
  
  trajectory_generator::TrajectoryPosition msg;
  double x;
  double y;
  bool led;
  
  while(ros::ok())
  {
    if (node.getCurrPos(x, y, led))
    {
      msg.x = x;
      msg.y = y;
      msg.led = led;
      
      pub_position.publish(msg);
    }
    
    ros::spinOnce();
    
    r.sleep();
  
  }
}
