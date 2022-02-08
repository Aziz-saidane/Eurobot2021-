#!/usr/bin/env python
import roboclaw_driver.roboclaw_driver as roboclaw
from roboclaw_node.msg import Polar
from roboclaw_node.msg import Encoders
from math import *
import rospy
import time




#print(c)

def callback(data):
	c.X = data.X
	c.Y = data.Y
	c.Theta = data.Theta
	
def calc():	
	global c
	global X1
	global Y1
	global dist_ticks
	global arc_ticks
	r = sqrt((X1-c.X)**2+(Y1-c.Y)**2)
	theta = 2*atan((Y1-c.Y)/((X1-c.X)+sqrt((X1-c.X)**2+(Y1-c.Y)**2))) - c.Theta
	arc_ticks = long(1581 * theta)
	dist_ticks = long(r * 16300)

#def return_to_horizontal():
	
	
if __name__ == '__main__':
	global c
	global X1
	global Y1
	global dist_ticks
	global arc_ticks
	global a
	a = 0
	b = 0
	dist_ticks = 0 
	arc_ticks = 0

	c = Polar()
	c.X = 0
	c.Y = 0
	c.Theta= -pi/2
	X1 = 0
	Y1 = 0.9
	rospy.init_node("navig")
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        baud_rate = int(rospy.get_param("~baud", "38400"))
	address = int(rospy.get_param("~address", "128"))
	roboclaw.Open(dev_name, baud_rate)
	roboclaw.ResetEncoders(address)
	r=rospy.Rate(0.5)
	r1 = rospy.Rate(10)
	rospy.Subscriber("/odom1", Polar, callback)
	en = Encoders()
	p = rospy.Publisher('/encoders', Encoders, queue_size=10)
	en.Status1, en.Status2, en.Enc1, en.Enc2, en.Crc1, en.Crc2 = 0, 0, 0, 0, 0, 0
	for X1,Y1 in [(1,0)]:
		print (c)
		calc()
		print('mission 1')
		while roboclaw.ReadBuffers(address)[1] is 128:
			print(roboclaw.ReadBuffers(address)[1])
			en.Status1, en.Enc1, en.Crc1 = roboclaw.ReadEncM1(address)
			en.Status2, en.Enc2, en.Crc2 = roboclaw.ReadEncM2(address)
			p.publish(en)
			roboclaw.SpeedAccelDeccelPositionM1M2(address, 5000, 10000, 5000, a+arc_ticks, 5000, 7000, 5000, b-arc_ticks,1)
			r1.sleep()
		roboclaw.SpeedAccelDeccelPositionM1M2(address, 5000, 8000, 5000, a+arc_ticks, 5000, 8000, 5000, b-arc_ticks,1)
		while roboclaw.ReadBuffers(address)[1] is not 128:
			r1.sleep()
			en.Status1, en.Enc1, en.Crc1 = roboclaw.ReadEncM1(address)
			en.Status2, en.Enc2, en.Crc2 = roboclaw.ReadEncM2(address)
			p.publish(en)			
			r1.sleep()
			continue
		time.sleep(2)
		a = a+arc_ticks
		b = b-arc_ticks



		'''print('mission 2')'''
		'''while roboclaw.ReadBuffers(address)[1] is 128:
			roboclaw.SpeedAccelDeccelPositionM1M2(address, 5000, 10000, 5000, a+dist_ticks, 5000, 7000, 5000, b+dist_ticks,1)
			en.Status1, en.Enc1, en.Crc1 = roboclaw.ReadEncM1(address)
			en.Status2, en.Enc2, en.Crc2 = roboclaw.ReadEncM2(address)
			p.publish(en)
			r1.sleep()'''
		'''roboclaw.SpeedAccelDeccelPositionM1M2(address, 5000, 7000, 5000, a+dist_ticks, 5000, 7000, 5000, b+dist_ticks,1)
		while roboclaw.ReadBuffers(address)[1] is not 128:
			r1.sleep()
			en.Status1, en.Enc1, en.Crc1 = roboclaw.ReadEncM1(address)
			en.Status2, en.Enc2, en.Crc2 = roboclaw.ReadEncM2(address)
			p.publish(en)
			r1.sleep()
			continue
		time.sleep(2)
		a = a+dist_ticks
		b = b+dist_ticks'''
		
		


      
	'''arc_ticks = long(1630 * ( c.Theta))
	roboclaw.SpeedAccelDeccelPositionM1M2(address, 2000, 5000, 2000, a+arc_ticks, 2000, 5000, 2000, b-arc_ticks,1)
	a = a+arc_ticks
	b = b-arc_ticks
	time.sleep(5)'''
	'''dist_ticks = long(0.06 * 16070)
	roboclaw.SpeedAccelDeccelPositionM1M2(address, 5000, 7000, 5000, a+dist_ticks, 5000, 7000, 5000, b+dist_ticks,1)'''
