#!/usr/bin/env python
# license removed for brevity

# motion:
#   -2 : BREAK
#   -1 : STOP
# #  0 : .
#   4 : Thank you
#   15 : Sit downn
#   1 : Stand up
#   9 : Walk Ready
#   23 : Yes Go
#   27 : Oops
# # soccer
#   80 : Init pose
#   122 : Get up(Front)
#   123 : Get up(Back)
#   121 : Right Kick
#   120 : Left Kick
#   60 : Keeper Ready
#   61 : Defence to Right
#   62 : Defence to Left
# # action demo
#   200 : Foot play
#   202 : Roll back
#   204 : Look
#   126 : Push up
# motion_shortcut:
# # motion_index : button id (0 ~ 9)

import rospy
from std_msgs.msg import String,Float64,Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from robotis_controller_msgs.msg import StatusMsg

class KSA_CONTROL:

    def __init__(self):

        rospy.loginfo("manual node started.....!")
        # Create a publisher to initialize the robot before starting motion.
        self.ini_pose_publisher = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=10)
        
        # Defining a ROS msg of a string data type 
        self.ini_pose_msg = String()
        self.ini_pose_msg.data = "ini_pose"

        # Publishing the initialization msg to the op2_manager
        self.ini_pose_publisher.publish(self.ini_pose_msg)
        rospy.loginfo("Published ini_pose: %s",self.ini_pose_msg.data)
        
        # Wait for the robot to finish
        rospy.sleep(5)
        
        # Create a publisher to enable the motion_module to be able to send motion commands.
        self.enable_ctrl_module_publisher = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size=10)
        
        # Defining a ROS msg of a string data type 
        self.enable_ctrl_module_msg = String()
        self.enable_ctrl_module_msg.data = "action_module"
        
        # Publishing the initialization msg to the op2_manager
        self.enable_ctrl_module_publisher.publish(self.enable_ctrl_module_msg)
        rospy.loginfo("Published ctrl_module %s:",self.enable_ctrl_module_msg.data)
        
        # Wait for the robot to finish
        rospy.sleep(5)


        # Create a publisher to send the index of the needed, Motion index table is defined above in this file.
        self.page_num_publisher = rospy.Publisher('/robotis/action/page_num', Int32, queue_size=10)
        
        # Defining a ROS msg of a string data type 
        self.page_num_msg = Int32()
        self.page_num_msg.data = 1

        # Send an index of 1 which means "Stand Up".
        self.page_num_publisher.publish(self.page_num_msg)
        rospy.loginfo("Published page_num: %s",self.page_num_msg.data)

        # Send demo motion commands
        # self.demo_command_publisher = rospy.Publisher('/robotis/demo_command', String, queue_size=10)
        # self.demo_command_publisher.publish("/robotis/demo_command")

        # Set the motion_module for each joint
        # self.set_joint_ctrl_module_publisher = rospy.Publisher('/robotis/set_joint_ctrl_module', String, queue_size=10)
        # self.set_joint_ctrl_modulepublisher.publish("/robotis/set_joint_ctrl_module")
        # self.go_subscriber = rospy.Subscriber("start", String, self.callback_go_subscriber)
        
        # Get the status of the op2 after each action.
        # self.status_subscriber = rospy.Subscriber("robotis/status", StatusMsg, self.callback_status_subscriber)


        # The valid emotions and the corresonding motion Index, Can be changed according to the Motion table as we want.
        self.responses_dict = {
            "Angry": 15,
            "Disgusted": 4,
            "Fearful": 1,
            "Happy": 9,
            "Neutral": 23,
            "Sad": 121,
            "Surprised": 27}

        # Create a subscriber to get the emotions that the robot sees right now.
        self.emotion_subscriber = rospy.Subscriber("emotions", String, self.callback_emotion_subscriber)

        rospy.sleep(5)

        self.rate = rospy.Rate(10) # 10hz


    # A method that gets the emotion that is recieved on the /emotions topic and returns the coresponding motion Index from the responses_dict.
    def get_motion_index(self, emotion):
        for emo in self.responses_dict.keys():
            if emo == self.recieved_emotion:
                return self.responses_dict[emo]
        return "Emotion not found"

    # The callback that is called when a new emotion is recieved.
    def callback_emotion_subscriber(self, msg):

        # Prin the recieved emotion to the terminal.
        rospy.loginfo("Received emotion: %s", msg.data)

        # Get the data field from the msg into a normal string variable.
        self.recieved_emotion = str(msg.data)

        # Pass the string to the get_motion_index function to get the index corresponding to the recieved emotion from the responses_dict.
        self.get_motion_index_output = self.get_motion_index(self.recieved_emotion)

        # Unsubscribe from the emotions topic until we finish responding to the last emotion recieved.
        self.emotion_subscriber.unregister()

        # define a ROS msg of type int
        self.page_num_msg = Int32()
        self.page_num_msg.data = self.get_motion_index_output

        # Publish the motione index for the robot to move
        self.page_num_publisher.publish(self.page_num_msg)
        rospy.loginfo("Published page_num: %s",self.page_num_msg.data)

        # Wait for the robot to finish the action.
        rospy.sleep(10)


        rospy.loginfo("Done...!")

        # Resubscribe to the emotions topic to recieve a new emotion and respond to it.
        self.emotion_subscriber = rospy.Subscriber("emotions", String, self.callback_emotion_subscriber)
        rospy.loginfo("Resubscribed...!")

        rospy.loginfo("Waiting for another command...!")
        rospy.sleep(5)

        self.rate.sleep()    
        return


if __name__ == '__main__':
    try:
        rospy.init_node('ksa_control', anonymous=True)
        KSA_CONTROL()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass