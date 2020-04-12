#!/usr/bin/env python
#
#
#
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from beginner_tutorials.msg import Position
x=0.0
y=0.0

yaw = 0.0
goal = Point()
goal.x = (0,0,2,3)
goal.y = (0,1,2,-3)
move = Twist()
yawKp = 4.5
forwardKp = 1.5
old_yaw = 0
num_wraps = 0



def odom(data,pub):
  global x
  global y
  global old_yaw,num_wraps,yaw
  threshold = 15*math.pi/180
  pos_msg = Position()
  x = data.pose.pose.position.x
  y = data.pose.pose.position.y
  rot_quat = data.pose.pose.orientation
  (roll,pitch,yaw) = euler_from_quaternion ([rot_quat.x,rot_quat.y,rot_quat.z,rot_quat.w])
  if old_yaw < -threshold and yaw > threshold: # from -pi to pi (increasing negative)
    num_wraps = num_wraps - 1
  elif old_yaw > threshold and yaw < -threshold:
    num_wraps = num_wraps + 1
  wrapped_yaw = yaw + 2*math.pi*num_wraps
  old_yaw = yaw
  pos_msg.angular.roll  = roll * 180/math.pi
  pos_msg.angular.pitch = pitch * 180/math.pi
  pos_msg.angular.yaw   = wrapped_yaw * 180/math.pi
  pub1.publish(pos_msg)

rospy.init_node("Controller")
pub= rospy.Publisher('/cmd_vel',Twist,queue_size = 10)
pub1 = rospy.Publisher('/eul', Position, queue_size=10)
sub = rospy.Subscriber('/odom',Odometry,odom,(pub1))
rate = rospy.Rate(100)


while not rospy.is_shutdown():
  for i,gy in enumerate(goal.y):
    for i,gx in enumerate(goal.x):
      print('Goal is',goal.x[i],goal.y[i])
      while True:
        deltaX = goal.x[i] - x
        deltaY = goal.y[i] - y
        bearingToGoal = math.atan2(deltaY,deltaX) 
        if x <= goal.x[i]+0.1 and x >= goal.x[i]-0.01:
            if y <= goal.y[i]+0.1 and y >= goal.y[i]-0.01:
              move.angular.z = 0.0
              move.linear.x=0.0
              print("GOAL!")
              break
        if abs(bearingToGoal - yaw) > 0.15:
          move.angular.z= (bearingToGoal - yaw) * yawKp
          move.linear.x = 0.00
          #print('turning to',bearingToGoal,'from',yaw)
        else:
          move.angular.z=0.0
          move.linear.x = 0.2#((math.hypot(goal.x-x,goal.y-y)) * forwardKp)
          #print(x,y)
        pub.publish(move)
        rate.sleep()
