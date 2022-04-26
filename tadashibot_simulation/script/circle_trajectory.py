#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from mars_bot_simulation.msg import drive
import time

class odometry_data:
        def __init__(self):
                self.x=0.0
                self.y=0.0

                self.goal = Point()
                self.goal.x = 0
                self.goal.y = 0

                self.lin_vel = 0.0
                self.ang_vel = 0.0

                self.roll=0.0
                self.pitch=0.0
                self.yaw=0.0

                self.kp=0.0
                self.ki=0.0
                self.error=0.0
                self.integral_prior=0
                self.integral=0

                self.target=0
                self.current=0
                self.t=time.time()

                rospy.init_node("odom_publisher", anonymous=False)
                self.odom_sub=rospy.Subscriber("odom", Odometry, self.current_odometry)
                self.velocity_pub=rospy.Publisher("cmd_vel",Twist,queue_size=10)
                self.position_pub=rospy.Publisher("position",drive,queue_size=10)

        def current_odometry(self,msg):
                self.x = msg.pose.pose.position.x
                self.y = msg.pose.pose.position.y

                self.rot_q = msg.pose.pose.orientation
                (self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])

        def set_pi(self,kp,ki):
                self.kp=kp
                self.ki=ki

        def set_target(self,x_target,y_target,theta_target):
                self.x_target=x_target
                self.y_target=y_target
                self.theta_target=theta_target
                cmd_vel=Twist()
                dt=self.t-time.time()

                if (dt<=0.02):
                        self.error=self.x_target-self.x
                        self.integral=self.integral_prior+self.error*dt

                        output=self.kp*self.error+self.ki*self.integral
                        cmd_vel.linear.x=output
                        cmd_vel.angular.z=0.0
                        self.integral_prior=self.integral
                        time.time()=
                self.velocity_pub.publish(cmd_vel)
                # time.time()=self.t
                        
                
                

                print("Goal Achieved: pos_x=%.2f pos_y=%.2f vel_lin=%.2f vel_ang=%.2f"%(self.x,self.y,self.lin_vel,self.ang_vel))
                # return output




     


if __name__=="__main__":
        odom_data=odometry_data() #input command from the machine learning.
        rate=rospy.Rate(100)
        while not rospy.is_shutdown():
                odom_data.set_pi(0.1,0.1)
                odom_data.set_target(1,0,0)
                rate.sleep()
