#!/usr/bin/python


import pygame
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import os

class PadDriver:
    def __init__(self, name='PLAYSTATION(R)3 Controller'):
        self.shutdownCounter = 0
        self.Pad = []
        pygame.init()
        pygame.joystick.init()
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
        
        for i in range(numButtons):
            Buttons = np.append(Buttons, self.Pad.get_button(i))
        
        linear = 0.5 * (Axes[13] - Axes[12]) + Buttons[14] * 0.5 * (Axes[13] - Axes[12])
        angular = 0.5 * Axes[0] + Buttons[14] * 0.5 * Axes[0]

	if Buttons[3]:
	    self.shutdownCounter += 1
	else:
	    self.shutdownCounter = 0

	if self.shutdownCounter>30:
            print "Shutdown command detected!"
            os.system("sudo shutdown -h now")        

        self.ROS_Publish(linear, angular)
    
    def ROS_InitNode(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('Pad_driver', anonymous=True)
        self.rate = rospy.Rate(10)
        
    def ROS_Publish(self, linear, angular):
        message = Twist()        
        message.linear.x = linear
        message.angular.z = angular
        rospy.loginfo(message)
        self.pub.publish(message)
        
    def ROS_spin(self):
        self.rate.sleep()
        
        
Pad = PadDriver()
Pad.ROS_InitNode()

try:
    while not rospy.is_shutdown():
        Pad.sendValues()
        Pad.ROS_spin()
        
except rospy.ROSInterruptException:
    pass
    
        
        
        
        
