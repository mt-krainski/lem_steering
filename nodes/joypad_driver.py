#!/usr/bin/python

import time
import pygame
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from lem_steering.msg import LemArm
import os

os.environ["SDL_VIDEODRIVER"] = "dummy"

pygame.init()
pygame.joystick.init()

from contextlib import contextmanager
import sys, os

@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:
            yield
        finally:
            sys.stdout = old_stdout

class PadDriver:
    def __init__(self, name='PLAYSTATION(R)3 Controller'):
        self.shutdownCounter = 0
        self.Pad = []
	self.publish = True
	self.selectPressed=False
	self.R3Pressed = False
	self.limitPower = [1.0, 1.0]

        joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

        for joy in joysticks:
            if 'PLAYSTATION(R)3 Controller' in joy.get_name():
                self.Pad = joy
                break

        self.Pad.init()

    def sendValues(self):
        pygame.event.get()

        numAxes = self.Pad.get_numaxes()
        numButtons = self.Pad.get_numbuttons()

        Axes = []
        Buttons = []

        for i in range(numAxes):
            Axes = np.append(Axes, self.Pad.get_axis(i))
            if abs(Axes[i])<0.2:
                Axes[i] = 0


        for i in range(numButtons):
            Buttons = np.append(Buttons, self.Pad.get_button(i))

    #--------------- teleop part
        linear = 0.7 * (Axes[13] - Axes[12]) + Buttons[14] * 0.3 * (Axes[13] - Axes[12])

        angular = 0.7 * Axes[0] + Buttons[14] * 0.3 * Axes[0]

    	if Buttons[0]:
    	    if not self.selectPressed:
       	        if self.publish:
    	            self.publish=False
    	        else:
    	            self.publish=True
    		self.selectPressed=True
    	else:
    	    if self.selectPressed:
    	        self.selectPressed=False

    	if Buttons[2]:
            if not self.R3Pressed:
                if self.limitPower[0]==1.0:
                    self.limitPower[0] = 0.3
                    self.limitPower[1] = 0.6
                else:
                    self.limitPower[0]=1.0
                    self.limitPower[1]=1.0
                self.R3Pressed=True
        else:
            if self.R3Pressed:
                self.R3Pressed=False


    #--------------- system part

        if Buttons[3]:
    	    self.shutdownCounter += 1
    	else:
    	    self.shutdownCounter = 0

    	if self.shutdownCounter>30:
            print "Shutdown command detected!"
            os.system("sudo shutdown -h now")


    #--------------- arm part

        base = Axes[15] - Axes[14]
        joint0 = -Axes[3]
        joint1 = -Axes[1]
        grasperRoll = Axes[8] - Axes[10]
        grasperPitch = Axes[9] - Axes[11]
        grasperEnd = LemArm.GRASPER_IDLE
        if Buttons[15]:
            grasperEnd = LemArm.GRASPER_CLOSE
        elif Buttons[13]:
            grasperEnd = LemArm.GRASPER_OPEN
        else:
            grasperEnd = LemArm.GRASPER_IDLE


    	if self.publish:
    	    self.ROS_teleop_Publish(self.limitPower[0]*linear, self.limitPower[1]*angular)
            self.ROS_arm_cmd_Publish(base, joint0, joint1, grasperRoll, grasperPitch, grasperEnd)


    def ROS_InitNode(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.arm_cmd_pub = rospy.Publisher('arm_cmd', LemArm, queue_size=10)
        rospy.init_node('Pad_driver', anonymous=True)
        self.rate = rospy.Rate(10)

    def ROS_teleop_Publish(self, linear, angular):
        message = Twist()

        message.linear.x = linear
        message.angular.z = angular
        #rospy.loginfo(message)

        self.cmd_vel_pub.publish(message)

    def ROS_arm_cmd_Publish(self, base, joint0, joint1, grasperRoll, grasperPitch, grasperEnd):
        message = LemArm()
        message.header.stamp = rospy.Time.now()

        message.base = base
        message.joint0 = joint0
        message.joint1 = joint1
        message.grasperRoll = grasperRoll
        message.grasperPitch = grasperPitch
        message.grasperEnd = grasperEnd
        #rospy.loginfo(message)

        self.arm_cmd_pub.publish(message)



    def ROS_spin(self):
        self.rate.sleep()


try:
    Pad = PadDriver()
    Pad.ROS_InitNode()

    while not rospy.is_shutdown():
        Pad.sendValues()
        Pad.ROS_spin()

except rospy.ROSInterruptException:
    pass
