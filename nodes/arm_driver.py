#!/usr/bin/python

import serial
from select import select
import socket
import listPorts
from time import sleep
from time import time
import threading
from lem_steering.msg import LemArm

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('Arm_republisher', anonymous=True)

ENABLE_MOTORS = False
DEBUG = True

MAXRANGE = 1.0

jointMaxPower = 240.0
grasperMaxPower = 20

lastDataReceivedTime = time()

def callback(data):
    global lastDataReceivedTime
    lastDataReceivedTime = time()
    base = int((grasperMaxPower * data.base)/MAXRANGE)
    joint0 = int((jointMaxPower * data.joint0)/MAXRANGE)
    joint0 = min((joint0, jointMaxPower))
    joint1 = int((jointMaxPower * data.joint1)/MAXRANGE)
    joint1 = min((joint1, jointMaxPower))

    graspR = int(grasperMaxPower*(data.grasperRoll - data.grasperPitch)/MAXRANGE)
    graspR = min((graspR, grasperMaxPower))
    graspR = max((graspR, -grasperMaxPower))

    graspL = int(grasperMaxPower*(data.grasperRoll + data.grasperPitch)/MAXRANGE)
    graspL = min((graspL, grasperMaxPower))
    graspL = max((graspL, -grasperMaxPower))
    graspClose = data.grasperEnd

    if DEBUG:
        rospy.loginfo('Direct: ' + str(base) + ' ' + str(joint0) + ' ' + str(joint1) + ' ' + str(graspR) + ' ' + str(graspL) + ' ' + str(graspClose))

    if ENABLE_MOTORS:
        ArmDriverPort.write('Set: ' + str(base) + ' ' + str(joint0) + ' ' + str(joint1) + ' ' + str(graspR) + ' ' + str(graspL) + ' ' + str(graspClose) +'\n')

def watchdog():
    global lastDataReceivedTime
    while not rospy.is_shutdown():
        if (lastDataReceivedTime + 0.5) < time():
            if ENABLE_MOTORS:
                ArmDriverPort.write('Set 0 0 0 0 0 0 0\n')
            if DEBUG:
                rospy.loginfo('Watchdog: 0 0 0 0 0 0 0')
		sleep(0.1)

watchdog_thread = threading.Thread(target=watchdog)

def anyEqual(list, value):
    for i in range(0, len(list)):
        if list[i] == value:
            return 1
    return 0

print 'Connecting device...'

DAU_connected = False

if ENABLE_MOTORS:

    try:
    	while not DAU_connected and not rospy.is_shutdown():

    		portnames = listPorts.serial_ports()
    		openedPort = []
    		for i in range(0, len(portnames)):
    		   if '/dev/ttyACM' in portnames[i]:
    		      openedPort.append(serial.Serial(portnames[i], 115200, timeout=1))

    		sleep(2)

    		armDriverIndex = -1
    		portFunctions = []

    		for i in range(0, len(openedPort)):
    		   if openedPort[i].isOpen():
    		      openedPort[i].write('GET\n')
    		      sleep(0.1)
    		      portFunctions.append(openedPort[i].readline())
    		   else:
    		      portFunctions.append('Closed')

    		rospy.loginfo(portFunctions)

    		for i in range(0, len(portFunctions)):
    		   if portFunctions[i] == "Arduino Arm Driver\r\n":
    		      armDriverIndex = i
    		      rospy.loginfo("ArmDriver detected on port" + portnames[i])
    		   else:
    		      openedPort[i].close()

    		if armDriverIndex == -1:
    		   continue

    		ArmDriverPort = openedPort[armDriverIndex]
    		DAU_connected = True

    finally:
    	pass


if not DEBUG:
   rospy.loginfo('Silent mode activated...')


try:
    watchdog_thread.start()
    rospy.Subscriber("/arm_cmd", LemArm, callback)
    rospy.spin()

finally:
    ArmDriverPort.write('ChwytStop\n')
    ArmDriverPort.write('PowerOff 0 0 0 0 0 0 0\n')
    ArmDriverPort.close()
