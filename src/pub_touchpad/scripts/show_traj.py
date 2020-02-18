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
        self.h = []
        self.fig = plt.gcf()
        self.fig.show()
        self.curr_plot_idx = -1
        plt.xlim([1200,5500])
        plt.ylim([-5000,-1200])
        self.fig.canvas.draw()
        self.mutex_h = Lock()
        self.mutex_xy = Lock()
        self.subscriber = rospy.Subscriber("finger_position", Int32MultiArray, self.callback)

    def callback(self, msg):
        if msg.data[0] == 10000 and msg.data[1] == 10000:
            rospy.loginfo("Reset the figure.")
            self.mutex_h.acquire()
            self.curr_plot_idx = -1
            self.currX = -1
            self.currY = -1
            self.prevX = -1
            self.prevY = -1
            for plot_obj in self.h:
                plot_obj[0].set_xdata([])
                plot_obj[0].set_ydata([])
            self.h = []
            self.mutex_h.release()
            rospy.loginfo("All cleared.")
        else:
            self.mutex_xy.acquire()
            self.currX = msg.data[0]
            self.currY = -msg.data[1]
            self.mutex_xy.release()
    def draw(self):
        # plt.plot(self.currX, self.currY,'k.')
        self.mutex_h.acquire()
        self.mutex_xy.acquire()
        currX = self.currX
        currY = self.currY
        self.mutex_xy.release()
        prevX = self.prevX
        prevY = self.prevY
        self.prevX = currX
        self.prevY = currY
        dist = np.sqrt((currX - prevX)**2 + (currY - prevY)**2)
        if not self.h:
            print("New figure. Creat new plot object array.")
            self.h.append(plt.plot([],[]))
            self.curr_plot_idx +=1
            self.fig.canvas.draw()
        elif dist > 400:
            print("New character. Append the plot object array.")
            self.h.append(plt.plot([],[]))
            self.curr_plot_idx +=1
            self.fig.canvas.draw()

        if dist != 0 and currX !=-1:
            self.h[self.curr_plot_idx][0].set_xdata(np.append(self.h[self.curr_plot_idx][0].get_xdata(), currX))
            self.h[self.curr_plot_idx][0].set_ydata(np.append(self.h[self.curr_plot_idx][0].get_ydata(), currY))
            self.h[self.curr_plot_idx][0].set_color('k')
            self.h[self.curr_plot_idx][0].set_linewidth(5)
            self.fig.canvas.draw()
        self.mutex_h.release()
if __name__ == '__main__':
    rospy.init_node('traj_disp')
    displayer = displayer()
    while not rospy.is_shutdown():
        displayer.draw()