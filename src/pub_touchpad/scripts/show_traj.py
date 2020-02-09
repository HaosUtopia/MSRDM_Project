#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Int32MultiArray
import numpy as np
from threading import Thread, Lock


mutex = Lock()

class displayer:
    def __init__(self):
        self.currX = -1
        self.currY = -1
        self.prevX = -1
        self.prevY = -1
        self.fig = plt.gcf()
        self.fig.show()
        self.fig.canvas.draw()
        self.h = []
        self.curr_plot_idx = -1
        plt.xlim([0,5000])
        plt.ylim([-5000,0])

        self.subscriber = rospy.Subscriber("finger_position", Int32MultiArray, self.callback)

    def callback(self, msg):
        #mutex.acquire()
        self.currX = msg.data[0]-1300
        self.currY = 1200-msg.data[1]
        #mutex.release()
    def draw(self):
        # plt.plot(self.currX, self.currY,'k.')
        #mutex.acquire()
        currX = self.currX
        currY = self.currY
        prevX = self.prevX
        prevY = self.prevY

        self.prevX = currX
        self.prevY = currY
        #mutex.release()
        dist = np.sqrt((currX - prevX)**2 + (currY - prevY)**2)
        if not self.h or dist > 300:
            print("Distance:",dist)
            self.h.append(plt.plot([],[]))
            self.curr_plot_idx +=1
        
        self.h[self.curr_plot_idx][0].set_xdata(np.append(self.h[self.curr_plot_idx][0].get_xdata(), currX))
        self.h[self.curr_plot_idx][0].set_ydata(np.append(self.h[self.curr_plot_idx][0].get_ydata(), currY))
        self.h[self.curr_plot_idx][0].set_color('k')
        self.h[self.curr_plot_idx][0].set_linewidth(5)
        self.fig.canvas.draw()
if __name__ == '__main__':
    rospy.init_node('traj_disp')
    displayer = displayer()
    while not rospy.is_shutdown():
        displayer.draw()