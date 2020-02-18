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
bool reset = false;

bool startTrajCallback(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp)
{
  ROS_INFO_STREAM("Start to collect touchpad");
  started = true; 
  return true;
}

bool resetCallback(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp)
{
  ROS_INFO_STREAM("Reset");
  reset = true; 
  return true;
}


int main(int argc, char *argv[])
{
    struct input_event ev;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    fd_set set;
    int fd = -77;
    int result;
    char name[256] = "Unknown";


    if ((getuid ()) != 0) {
        fprintf(stderr, "You are not root! This may not work...\n");
        return EXIT_SUCCESS;
    }


    ros::init(argc, argv, "Finger_pub");
    ros::NodeHandle n;
    ros::Publisher finger_pub = n.advertise<std_msgs::Int32MultiArray>("finger_position", 1000);
    ros::ServiceServer srv_start = n.advertiseService("trajectory_start", startTrajCallback);
    ros::ServiceServer srv_reset = n.advertiseService("trajectory_reset", resetCallback);
    std_msgs::Int32MultiArray msg;

    double currX = -1;
    double currY = -1;
    bool newX = false;
    bool newY = false;
    bool largeDistX = false;
    bool largeDistY = false;
    int step = 0;
    ros::Rate r(50);
    
    while(ros::ok()) {

    	while(started && ros::ok()){
            /* Open Device */
            if(fd == -77)
            {
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
            }
            if(reset)
            {
                // ROS_INFO_STREAM("Publish reset message.");
                msg.data.clear();
                msg.data.push_back(10000);
                msg.data.push_back(10000);
                finger_pub.publish(msg);
                reset = false;

            }


	        /* TODO: use select() */
            // ROS_INFO_STREAM("LOOP RUNNING1");
	        // size = read(fd, &ev, ev_size);
            FD_ZERO(&set);
            FD_SET(fd, &set);
            result = select(fd+1, &set, NULL, NULL, &tv);
            if(result == -1)
                ROS_INFO_STREAM("error.");
            // else if (result == 0);
            //     ROS_INFO_STREAM("No data received");
            else if (result != 0)
            {
                const size_t ev_size = sizeof(struct input_event);
                ssize_t size;
                size = read(fd, &ev, ev_size);
                if (size < ev_size) 
                {
                    fprintf(stderr, "Error size when reading\n");
                    goto err;
                }
            }
	        if (ev.type == EVENT_TYPE && (ev.code == EVENT_CODE_X
	                      || ev.code == EVENT_CODE_Y)) {
	            /* TODO: convert value to pixels */
	            // printf("%s = %d\n", ev.code == EVENT_CODE_X ? "X" : "Y",
	            //         ev.value);
	            switch(ev.code){
	            	case EVENT_CODE_X:  if(currX!=ev.value)
                                        {
                                            if(currX!=-1 && abs(currX - ev.value)>300)
                                                largeDistX = true;

                                            currX = ev.value;
                                            newX = true;
                                        }
            
	            					    break;
	            	case EVENT_CODE_Y:  
                                        if(currY!=ev.value)
                                        {
                                            if(currY!=-1 && abs(currY - ev.value)>300)
                                                largeDistY = true;
                                            currY = ev.value;
                                            newY = true;
                                        }
	            					    break;
	            }
	            if (newX && newY)
                {
	                // publish the point
                    if(largeDistY||largeDistX)
                    {
                        if(step++ == 1)
                        {
                            msg.data.clear();
                            msg.data.push_back(currX);
                            msg.data.push_back(currY);
                            finger_pub.publish(msg);
                            newX = false;
                            newY = false;
                            largeDistX = false;
                            largeDistY = false;
                            step = 0;
                        }
                    }
                    else if(!largeDistY&&!largeDistX)
                    {
                        msg.data.clear();
                        msg.data.push_back(currX);
                        msg.data.push_back(currY);
                        finger_pub.publish(msg);
                        newX = false;
                        newY = false;   
                    }
                          
	            } 
                    
	    	}  
            ros::spinOnce();  
            
        }
        ros::spinOnce();

        r.sleep();
    }
    
    return EXIT_SUCCESS;

err:
    close(fd);
    return EXIT_FAILURE;
}
