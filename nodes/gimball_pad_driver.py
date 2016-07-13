#!/usr/bin/python

import time
from time import sleep
import pygame
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import os

import serial

gimball_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

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

        azimuth = int((Axes[2]+1.0)/2.0 * 180)
        elevation = int((Axes[3]+1.0)/2.0 * 180)

        gimball_port.write('P'+str(azimuth)+'\n')
        print gimball_port.readline()
        gimball_port.write('Y'+str(elevation)+'\n')

        print gimball_port.readline()


try:
    Pad = PadDriver()

    while 1:
        Pad.sendValues()
        sleep(0.01)

except rospy.ROSInterruptException:
    pass
