#!/usr/bin/python

import time
import pygame
import rospy
import numpy as np
from geometry_msgs.msg import Twist
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
	with suppress_stdout():
	    numAxes = self.Pad.get_numaxes()
	with suppress_stdout():
            numButtons = self.Pad.get_numbuttons()
        
        Axes = []
        Buttons = []

        for i in range(numAxes):
            Axes = np.append(Axes, self.Pad.get_axis(i))
        
        for i in range(numButtons):
            Buttons = np.append(Buttons, self.Pad.get_button(i))
        
        linear = 0.7 * (Axes[13] - Axes[12]) + Buttons[14] * 0.3 * (Axes[13] - Axes[12])
        if abs(Axes[0])<0.2:
		Axes[0] = 0
	angular = 0.7 * Axes[0] + Buttons[14] * 0.3 * Axes[0]

	grasperRoll = Axes[2]
        grasperPitch = Axes[3]

	if Buttons[3]:
	    self.shutdownCounter += 1
	else:
	    self.shutdownCounter = 0

	if self.shutdownCounter>30:
            print "Shutdown command detected!"
            os.system("sudo shutdown -h now")        

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


	if self.publish:
	    self.ROS_Publish(self.limitPower[0]*linear, self.limitPower[1]*angular,  grasperRoll, grasperPitch)
    
    def ROS_InitNode(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('Pad_driver', anonymous=True)
        self.rate = rospy.Rate(10)
        
    def ROS_Publish(self, linear, angular, grasperRoll, grasperPitch):
        message = Twist()        
        message.linear.x = linear
        message.angular.z = angular
	message.angular.y = grasperRoll
        message.angular.x = grasperPitch
        #rospy.loginfo(message)
	if self.publish:
	    self.pub.publish(message)
        
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
    
        
        
        
        
