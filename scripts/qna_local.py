#!/usr/bin/env python  
#coding=utf8
import roslib
import rospy  
import actionlib
from actionlib_msgs.msg import *  
from rosecho.msg import ttsAction, ttsGoal
from std_msgs.msg import Int16, String
from std_srvs.srv import Empty

class RosEchoQnA():  
    def asr_callback(self, data):
        ros_msg = data.data.strip('"')
        if not isinstance(ros_msg, unicode):
            ros_msg = unicode(ros_msg, "utf8")
            
        if ros_msg == u"做个自我介绍吧":
            #self.rosecho_sleep_client()
            #self.rosecho_tts.cancel_goal()
            text = ttsGoal()
            text.text = "大家好，我叫波弟，就是大家的小助手，我具备人脸识别、手势识别、自主导航、视觉避障、语音交互等功能，更多功能正在解锁中，希望日后能在生活中帮助到大家！"
            self.speak(text)

    def rosecho_sleep_client(self):
        rospy.wait_for_service('rosecho/sleep')
        try:
            rosecho_sleep = rospy.ServiceProxy('rosecho/sleep',Empty)
            resp1 = rosecho_sleep()
            return resp1
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)        
        
    def __init__(self):  
        
        rospy.init_node('rosecho_qnd', anonymous=False)  
        rospy.on_shutdown(self.shutdown) 

        # Subscribe to the move_base action server  
        self.rosecho_tts = actionlib.SimpleActionClient("rosecho/tts",ttsAction)

        rospy.loginfo("Waiting for move_base and rosecho tts action server...")  

        self.rosecho_tts.wait_for_server(rospy.Duration(6))
        rospy.loginfo("Connected to rosecho tts server")     

        rospy.Subscriber('/rosecho/asr', String, self.asr_callback)

    def speak(self, text):  

        # Send the goal pose to the tts server  
        self.rosecho_tts.send_goal(text)
        rospy.loginfo(text.text);  
        # Allow 30 seconds to speak. maybe shorter, the interact timeout is 30 sec.  
        finished_within_time = self.rosecho_tts.wait_for_result(rospy.Duration(30))   
        # If we don't finish in time, abort the goal  
        if not finished_within_time:  
            #self.rosecho_tts.cancel_goal()  
            rospy.loginfo("TTS Time Out!!")  
        else:  
            # We made it!  
            state = self.rosecho_tts.get_state()  
            if state == GoalStatus.SUCCEEDED:  
                rospy.loginfo("TTS finished!") 

    def shutdown(self):  

        rospy.loginfo("Stopping rosecho...")  

        # Cancel any active goals  
        state = self.rosecho_tts.get_state()  
        if not state == GoalStatus.SUCCEEDED:
            self.rosecho_tts.cancel_goal()
            rospy.sleep(2)


if __name__ == '__main__':  

    try:  
        RosEchoQnA() 
        rospy.spin()

    except rospy.ROSInterruptException:  
        rospy.loginfo("Navigation test finished.")
