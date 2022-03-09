#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
        global x
        global y
        global theta
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

Turned = False
Finish = False

goal = Point()
goal.x = 0
goal.y = 7.2

goal2 = Point()
goal2.x = 8
goal2.y = 8

def CombinedMovement(ang_to_goal, phi, speed_ang_z):
        if abs(ang_to_goal - phi) > 0.1:
                speed.linear.x = 0.2
                speed.angular.z = speed_ang_z
        else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

def StopMovement():
        speed.linear.x = 0.0
        speed.angular.z = 0.0

while not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y -y

        inc_x2 = goal2.x -x
        inc_y2 = goal2.y -y
        
        angle_to_goal = atan2(inc_y, inc_x)
        angle_to_goal2 = atan2(inc_y2, inc_x2)
        
        #----------------------------------------
        # speed.angular.z: 
        # +ve value -> car rotate anti-clockwise
        # -ve value -> car rotate clockwise
        #----------------------------------------
        
        if Turned == False:
                CombinedMovement(angle_to_goal, theta, 0.3)
                if abs(inc_y) < 0.075:
                        Turned = True
                        StopMovement()
        elif Turned == True:
                CombinedMovement(angle_to_goal2, theta, -0.3)
                if abs(inc_x2) < 0.15:
                        Finish = True
                        StopMovement()
        elif Finish == True:
                StopMovement()
        
        # This statement prevents the car from self-rotating when it arrive destination
        #if inc_x < 0.1 and inc_y < 0.1:
                #speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep()
