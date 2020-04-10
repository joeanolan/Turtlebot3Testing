#!/usr/bin/env python
#
#
#

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
#from geometry_msgs.msg import Odometry

import time

twist = Twist()

def callback_pos():
	position = Point()
	return position

def callback_odom():
	odom = Odometry()
	return odom

def right():
	global move_cmd
	global cmd_vel
	rospy.loginfo("right")  
	# turn at 0.15 rad/s
        move_cmd.angular.z = 0.15
	move_cmd.linear.x = 0.0
        cmd_vel.publish(move_cmd)
        r.sleep()

def forward():
	global move_cmd
	global cmd_vel
	rospy.loginfo("forward")
        move_cmd.linear.x = 1.5
	move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)
        r.sleep()
                  
def stop():
        rospy.loginfo("Stopped")
	move_cmd.linear.x=0.0
	move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)
   

def main():
	global twist
	twist.linear.x = 1.5
	twist.angular.z = 0.0
	pub.publish(twist)
	time.sleep(5)
	twist.linear.x = 0 
	twist.angular.z = 0.15
	pub.publish(twist)
	time.sleep(2)
	twist.angular.z=0.0
	twist.linear.x = 1.5
	pub.publish(twist)
	time.sleep(5)
	twist.linear.x = 0.0
	pub.publish(twist)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.Subscriber('/eul',Point,callback_pos)
#rospy.Subscriber('/odom',Odometry,callback_odom)

rospy.init_node('JNOLAN')
if __name__ == '__main__':
	try:
		print('before while')
		while not rospy.is_shutdown():
			main()
			#pub.publish(twist)
			
	except rospy.ROSInterruptException:
        	rospy.loginfo("node terminated")
		pass
