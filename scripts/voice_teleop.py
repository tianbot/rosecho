#!/usr/bin/env python
#coding=utf8
import rospy
import actionlib
from actionlib_msgs.msg import *  
from rosecho.msg import ttsAction, ttsGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, String

# msg = 
# """
# 用中文语音控制你的机器人！
# 终端并不需要置于最前激活状态！
# ---------------------------
# 前进
# 后退
# 左转
# 右转
# 停下来
# CTRL-C 退出
# """


class RosEchoVoiceTeleop():
    def asr_callback(self, data):
        ros_msg = data.data.strip('"')
        if not isinstance(ros_msg, unicode):
            ros_msg = unicode(ros_msg, "utf8")
        linear_vel   = 0.0
        angular_vel  = 0.0

        if ros_msg == u"前进":
            self.rosecho_tts.cancel_goal()

            text = ttsGoal()
            text.text = "前进！"
            self.speak(text)
            
            linear_vel = 0.3

        if ros_msg == u"后退":
            self.rosecho_tts.cancel_goal()

            text = ttsGoal()
            text.text = "后退！"
            self.speak(text)

            linear_vel = -0.3

        if ros_msg == u"左转":
            self.rosecho_tts.cancel_goal()

            text = ttsGoal()
            text.text = "左转！"
            self.speak(text)

            angular_vel = 0.3

        if ros_msg == u"右转":
            self.rosecho_tts.cancel_goal()

            text = ttsGoal()
            text.text = "右转！"
            self.speak(text)

            angular_vel = -0.3
        
        if ros_msg == u"停止":
            self.rosecho_tts.cancel_goal()

            text = ttsGoal()
            text.text = "停止！"
            self.speak(text)
            linear_vel = 0.0
            angular_vel = 0.0  
  
        twist = Twist()
        twist.linear.x = linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = angular_vel

        self.cmd_vel_pub.publish(twist)

    def shutdown(self):  

        rospy.loginfo("Stopping the robot...")
        twist = Twist()  
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def speak(self, text):  

        # Send the goal pose to the tts server  
        self.rosecho_tts.send_goal(text)
        rospy.loginfo(text.text);  
        # Allow 1 minute to speak  
        finished_within_time = self.rosecho_tts.wait_for_result(rospy.Duration(5))   
        # If we don't finish in time, abort the goal  
        if not finished_within_time:  
            self.rosecho_tts.cancel_goal()  
            rospy.loginfo("TTS Time Out!!")  
        else:  
            # We made it!  
            state = self.rosecho_tts.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("TTS finished!") 

    def __init__(self):

        rospy.init_node('rosecho_teleop', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # tts server to speak!
        self.rosecho_tts = actionlib.SimpleActionClient("rosecho/tts",ttsAction)
        rospy.loginfo("Waiting for rosecho tts action server...")  

        self.rosecho_tts.wait_for_server(rospy.Duration(6))
        rospy.loginfo("Connected to rosecho tts server")

        # stop robot when exit
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        rospy.Subscriber('/rosecho/asr', String, self.asr_callback)


if __name__ == "__main__":
    try:
        RosEchoVoiceTeleop()
        rospy.spin()

    except rospy.ROSInterruptException:

        rospy.loginfo(" test finished.")
