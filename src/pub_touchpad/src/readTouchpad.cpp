// system library
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/input.h>

#define EVENT_DEVICE    "/dev/input/event8"
#define EVENT_TYPE      EV_ABS
#define EVENT_CODE_X    ABS_X
#define EVENT_CODE_Y    ABS_Y

// ros library
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/Empty.h>

// matplotlib
//#include <pub_touchpad/matplotlibcpp.h>
//namespace plt = matplotlibcpp;
/* TODO: Close fd on SIGINT (Ctrl-C), if it's open */
bool started = false;


bool startTrajCallback(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp)
{
  started = true;
  
  return true;
}


int main(int argc, char *argv[])
{
    struct input_event ev;
    int fd;
    char name[256] = "Unknown";


    if ((getuid ()) != 0) {
        fprintf(stderr, "You are not root! This may not work...\n");
        return EXIT_SUCCESS;
    }

    /* Open Device */
    fd = open(EVENT_DEVICE, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "%s is not a vaild device\n", EVENT_DEVICE);
        return EXIT_FAILURE;
    }

    /* Print Device Name */
    ioctl(fd, EVIOCGNAME(sizeof(name)), name);
    printf("Reading from:\n");
    printf("device file = %s\n", EVENT_DEVICE);
    printf("device name = %s\n", name);

    ros::init(argc, argv, "Finger_pub");
    ros::NodeHandle n;
    ros::Publisher finger_pub = n.advertise<std_msgs::Int32MultiArray>("finger_position", 1000);
    ros::ServiceServer srv_start = n.advertiseService("trajectory_start", startTrajCallback);
    std_msgs::Int32MultiArray msg;

    double currX = -1;
    double currY = -1; 
    bool newX = false;
    bool newY = false;
    ros::Rate r(50);
    
    while(ros::ok()) {

    	while(started && ros::ok()){
	        const size_t ev_size = sizeof(struct input_event);
	        ssize_t size;

	        /* TODO: use select() */

	        size = read(fd, &ev, ev_size);
	        if (size < ev_size) {
	            fprintf(stderr, "Error size when reading\n");
	            goto err;
	        }

	        if (ev.type == EVENT_TYPE && (ev.code == EVENT_CODE_X
	                      || ev.code == EVENT_CODE_Y)) {
	            /* TODO: convert value to pixels */
	            // printf("%s = %d\n", ev.code == EVENT_CODE_X ? "X" : "Y",
	            //         ev.value);
	            switch(ev.code){
	            	case EVENT_CODE_X:  currX = ev.value;
	            						newX = true;
	            					    break;
	            	case EVENT_CODE_Y:  currY = ev.value;
	            						newY = true;
	            					    break;
	            }
	            if (newX && newY){
	                // publish the point
	                msg.data.clear();
	            	msg.data.push_back(currX);
	                msg.data.push_back(currY);
	            	ROS_INFO("X = %d, Y = %d, send out this postion", msg.data[0], msg.data[1]);
	   				finger_pub.publish(msg);
	   				newX = false;
	   				newY = false;         
	            } 
	    	}    
            
        }
        ros::spinOnce();

        r.sleep();
    }
    
    return EXIT_SUCCESS;

err:
    close(fd);
    return EXIT_FAILURE;
}
