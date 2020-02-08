// system library
#include <vector>
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

// matplotlib
#include <pub_touchpad/matplotlibcpp.h>
namespace plt = matplotlibcpp;
/* TODO: Close fd on SIGINT (Ctrl-C), if it's open */

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
    std_msgs::Int32MultiArray msg;

    double currX = -1;
    double currY = -1;
    std::vector<double> img_x;
    std::vector<double> img_y;
    plt::Plot fig("handwriting");
    plt::title("display trajectory");
    plt::xlim(0,5000);
    plt::ylim(-5000,0);
    
    while(ros::ok()) {
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
            					    break;
            	case EVENT_CODE_Y:  currY = ev.value;
            					    break;
            }
            if (currX !=-1 && currY!=-1){
            	  //printf("X = %d, ", currX);
            	  //printf("Y = %d\n", currY);

                // publish the point
                msg.data.clear();
            	  msg.data.push_back(currX);
                msg.data.push_back(currY);
            	  ROS_INFO("X = %d, Y = %d, send out this postion", msg.data[0], msg.data[1]);
   				      finger_pub.publish(msg);

                // display the point
                //plt::figure(fig);
                img_x.push_back(currX-1300);
                img_y.push_back(currY-1200);
                fig.update(img_x, img_y);
                //plt::plot({img_x}, {img_y}, ".k");
                //plt::show();         
            } 
            
        }
    }
    
    return EXIT_SUCCESS;

err:
    close(fd);
    return EXIT_FAILURE;
}
