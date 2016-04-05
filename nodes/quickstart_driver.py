#!/usr/bin/python

import serial
from select import select
import socket
from time import sleep
from time import time
import threading

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('Teleop_listener', anonymous=True)

ENABLE_MOTORS = True
DEBUG = True

MAXRANGE=1.0
DEADZONE = 15.0

motorPower = 255.0


lastDataReceivedTime = time()

def callback(data):
	global lastDataReceivedTime
	lastDataReceivedTime = time()
	RPower = int((motorPower * data.linear.x)/MAXRANGE) - int((motorPower * data.angular.z)/MAXRANGE)
	LPower = int((motorPower * data.linear.x)/MAXRANGE) + int((motorPower * data.angular.z)/MAXRANGE)
	if abs(RPower) < DEADZONE:
	    RPower = 0
        if abs(LPower) < DEADZONE:
            LPower = 0
	
        if DEBUG:	
		print 'Direct: ' + str(RPower) + ' ' + str(LPower)
	if ENABLE_MOTORS:
		MotorDriverPort.write('Set '+ str(LPower) +' '+ str(RPower) +' '+ str(LPower) +' '+ str(RPower) +' '+ str(LPower) +' '+ str(RPower) +'\n')
		sleep(0.05)
		MotorDriverPort.write('Set '+ str(LPower) +' '+ str(RPower) +' '+ str(LPower) +' '+ str(RPower) +' '+ str(LPower) +' '+ str(RPower) +'\n')


def watchdog():
	global lastDataReceivedTime
	while not rospy.is_shutdown():
		if (lastDataReceivedTime + 0.5) < time():
			MotorDriverPort.write('Set 0 0 0 0 0 0\n')
		sleep(0.1)

watchdog_thread = threading.Thread(target=watchdog)

def anyEqual(list, value):
    for i in range(0, len(list)):
        if list[i] == value:
            return 1
    return 0

print 'Connecting device...'

DAU_connected = False

try:
	while not DAU_connected:

#		portnames = listPorts.serial_ports()
#		openedPort = []
#		for i in range(0, len(portnames)):
#		   if '/dev/ttyACM' in portnames[i]:
#		      openedPort.append(serial.Serial(portnames[i], 115200, timeout=1))
#
#		sleep(2)
#
#		motorDriverIndex = -1
#		SensorsIndex = 0
#		portFunctions = []
#
#		for i in range(0, len(openedPort)):
#		   if openedPort[i].isOpen():
#		      openedPort[i].write('GET\n')
#		      sleep(0.1)
#		      portFunctions.append(openedPort[i].readline())
#		   else:
#		      portFunctions.append('Closed')
#
#		print portFunctions
#		   
#		for i in range(0, len(portFunctions)):
#		   if portFunctions[i] == "Arduino Motor Driver\r\n":
#		      motorDriverIndex = i
#		      print "MotorDriver detected on port", portnames[i]
#		   else:
#		      openedPort[i].close()
#
#		if motorDriverIndex == -1:
#		   continue

		MotorDriverPort = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
		MotorDriverPort.write('GET\n')
                sleep(0.1)
                print MotorDriverPort.readline()
		DAU_connected = True

finally:
	pass


if not DEBUG:
   print 'Silent mode activated...'


try:
   if MotorDriverPort.isOpen():
      watchdog_thread.start()

      rospy.Subscriber("/cmd_vel", Twist, callback)

      rospy.spin()

finally:
      MotorDriverPort.write('Set 0 0 0 0 0 0\n')
      MotorDriverPort.close()
