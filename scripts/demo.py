#!/usr/bin/env python  
#coding=utf8
import roslib

import rospy  

import actionlib  

from actionlib_msgs.msg import *  

from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped  

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  

from std_msgs.msg import Int16, String

from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

from math import radians, pi  

class RosEchoNav():  

    def pose_callback(self, data):
        if self.pose_flag == 0:
            self.pose_flag = 1
            self.init_pose = data.pose.pose
        else:
            self.pose = data.pose.pose
        
    def asr_callback(self, data):
        ros_msg = data.data.strip('"')
        cmd = u"去端茶"
        if not isinstance(ros_msg, unicode):
            ros_msg = unicode(ros_msg, "utf8")
        print(ros_msg)
        print(len(ros_msg))
        print(cmd)
        print(len(cmd))
        if cmd == ros_msg:
            self.move_base.cancel_goal()
    
            goal = MoveBaseGoal()
    
            goal.target_pose.header.frame_id = 'map'
    
            goal.target_pose.header.stamp = rospy.Time.now()
    
    
            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.position.x = 2.134
            goal.target_pose.pose.position.y = 7.17
            goal.target_pose.pose.position.z = 0
    
            self.move(goal)

        if ros_msg == u"回家":
            self.move_base.cancel_goal()
 
            goal = MoveBaseGoal()

	    goal.target_pose.header.frame_id = 'map'
 
            goal.target_pose.header.stamp = rospy.Time.now()
 
            goal.target_pose.pose.orientation.x = 0
            goal.target_pose.pose.orientation.y = 0
            goal.target_pose.pose.orientation.z = 0
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.position.x = 1.614
            goal.target_pose.pose.position.y = 1.536
            goal.target_pose.pose.position.z = 0
 
            self.move(goal)

    
    def wakeup_callback(self, data):

        self.move_base.cancel_goal()
        
        goal = MoveBaseGoal()  

        goal.target_pose.header.frame_id = 'map'  

        goal.target_pose.header.stamp = rospy.Time.now()  

        q_orig = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        q_rot = quaternion_from_euler(0, 0, float(data.data)/180.0*pi)
        q_new = quaternion_multiply(q_rot, q_orig)
        
        goal.target_pose.pose.orientation.x = q_new[0]
        goal.target_pose.pose.orientation.y = q_new[1]
        goal.target_pose.pose.orientation.z = q_new[2]
        goal.target_pose.pose.orientation.w = q_new[3]
        goal.target_pose.pose.position = self.pose.position

        self.move(goal)  
        
        
    def __init__(self):  

        self.pose_flag = 0

        self.pose = Pose()
        
        rospy.init_node('rosecho_nav', anonymous=False)  

        rospy.on_shutdown(self.shutdown)  

        # Subscribe to the move_base action server  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # Wait 60 seconds for the action server to become available  

        self.move_base.wait_for_server(rospy.Duration(60))  

        rospy.loginfo("Connected to move base server")  

        rospy.loginfo("Starting navigation test")  

        #stop robot when exit
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)

        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/rosecho/wakeup_pos', Int16, self.wakeup_callback)
        rospy.Subscriber('/rosecho/asr', String, self.asr_callback)

    def move(self, goal):  

            # Send the goal pose to the MoveBaseAction server  
            self.move_base.send_goal(goal)  

            # Allow 1 minute to get there  
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(60))   

            # If we don't get there in time, abort the goal  
            if not finished_within_time:  

                self.move_base.cancel_goal()  

                rospy.loginfo("Timed out achieving goal")  

            else:  

                # We made it!  

                state = self.move_base.get_state()  

                if state == GoalStatus.SUCCEEDED:  

                    rospy.loginfo("Goal succeeded!")  

    def shutdown(self):  

        rospy.loginfo("Stopping the robot...")  

        # Cancel any active goals  

        self.move_base.cancel_goal()
        rospy.sleep(2)

        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)


if __name__ == '__main__':  

    try:  
        RosEchoNav() 
        rospy.spin()

    except rospy.ROSInterruptException:  

        rospy.loginfo("Navigation test finished.")
