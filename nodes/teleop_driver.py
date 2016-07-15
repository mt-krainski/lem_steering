#!/usr/bin/python

import serial
from select import select
import socket
import listPorts
from time import sleep
from time import time
import threading

import rospy
from geometry_msgs.msg import Twist
try:
	from sensor_msgs.msg import BatteryState
except(ImportError):
	from lem_steering.msg import BatteryState
except:
	print("Can't find proper message type declaration!", sys.exc_info()[0])


rospy.init_node('Teleop_listener', anonymous=True)

battery_pub = rospy.Publisher('battery', BatteryState, queue_size=10)

ENABLE_MOTORS = True
DEBUG = False

MAXRANGE = 1.0

motorPower = 240.0


lastDataReceivedTime = time()

def callback(data):
	global lastDataReceivedTime
	lastDataReceivedTime = time()
	RPower = int((motorPower * data.linear.x)/MAXRANGE) - int((motorPower * data.angular.z)/MAXRANGE)
	RPower = min((RPower, 255))
	LPower = int((motorPower * data.linear.x)/MAXRANGE) + int((motorPower * data.angular.z)/MAXRANGE)
	LPower = min((LPower, 255))
	if DEBUG:
		print 'Direct: ' + str(RPower) + ' ' + str(LPower)
	if ENABLE_MOTORS:
		MotorDriverPort.write('Set '+ str(LPower) +' '+ str(RPower) +' '+ str(LPower) +' '+ str(int(1.5*RPower)) +' '+ str(LPower) +' '+ str(RPower) +'\n')
		sleep(0.05)
		MotorDriverPort.write('Set '+ str(LPower) +' '+ str(RPower) +' '+ str(LPower) +' '+ str(int(1.5*RPower)) +' '+ str(LPower) +' '+ str(RPower) +'\n')


def watchdog():
	global lastDataReceivedTime
	while not rospy.is_shutdown():
		if (lastDataReceivedTime + 0.5) < time():
			MotorDriverPort.write('Set 0 0 0 0 0 0\n')
			print 'Direct: 0 0 '
		sleep(0.1)

watchdog_thread = threading.Thread(target=watchdog)

def battery_republisher():
	global MotorDriverPort

	#print "Battery republisher launched"
	bat_status = BatteryState()
	BAT_FULL = 6*4.2
	BAT_MIN = 6*3.3

	bat_status.header.frame_id = 'robot'

	bat_status.current = float('nan')
	bat_status.charge = float('nan')
	bat_status.capacity = float('nan')
	bat_status.design_capacity = float('nan')

	bat_status.power_supply_technology = BatteryState().POWER_SUPPLY_TECHNOLOGY_LIPO

	for cell in range(0, 6):
		bat_status.cell_voltage.append(float('nan'))

	bat_status.location = 'Main Battery'

	bat_status.serial_number = "NA"

	while 1:

		MotorDriverPort.write('GetBatTotal\n')
		sleep(0.05)
		recv_data = MotorDriverPort.readline()
		print recv_data

	#	print recv_data

		bat_status.header.stamp = rospy.Time.now()


		try:
			bat_status.voltage = float(recv_data)
			bat_status.percentage = bat_status.voltage/BAT_FULL
			bat_status.present = bat_status.voltage<BAT_FULL and bat_status.voltage>BAT_MIN

			battery_pub.publish(bat_status)

		except:
			if DEBUG:
				print "Receive Error"
			else:
				pass

		sleep(0.01)

battery_republisher_thread = threading.Thread(target=battery_republisher)

def anyEqual(list, value):
    for i in range(0, len(list)):
        if list[i] == value:
            return 1
    return 0

print 'Connecting device...'

DAU_connected = False

try:
	while not DAU_connected and not rospy.is_shutdown():

		portnames = listPorts.serial_ports()
		openedPort = []
		for i in range(0, len(portnames)):
		   if '/dev/ttyACM' in portnames[i]:
		      openedPort.append(serial.Serial(portnames[i], 115200, timeout=1))

		sleep(2)

		motorDriverIndex = -1
		SensorsIndex = 0
		portFunctions = []

		for i in range(0, len(openedPort)):
		   if openedPort[i].isOpen():
		      openedPort[i].write('GET\n')
		      sleep(0.1)
		      portFunctions.append(openedPort[i].readline())
		   else:
		      portFunctions.append('Closed')

		print portFunctions

		for i in range(0, len(portFunctions)):
		   if portFunctions[i] == "Arduino Motor Driver\r\n":
		      motorDriverIndex = i
		      print "MotorDriver detected on port", portnames[i]
		   else:
		      openedPort[i].close()

		if motorDriverIndex == -1:
		   continue

		MotorDriverPort = openedPort[motorDriverIndex]
		DAU_connected = True

finally:
	pass


if not DEBUG:
   print 'Silent mode activated...'


try:
   if MotorDriverPort.isOpen():
	   watchdog_thread.start()
	   battery_republisher_thread.start()

	   rospy.Subscriber("/cmd_vel", Twist, callback)

	   rospy.spin()

finally:
      MotorDriverPort.write('Set 0 0 0 0 0 0\n')
      MotorDriverPort.close()
