#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String,Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class KSA_CONTROL:

    def __init__(self):

        # Create a publisher for each joint in the upper body of the robot
        self.pub_head_pan_response = rospy.Publisher('/robotis_op2/head_pan_position/command', Float64, queue_size=10)
        self.pub_head_tilt_response = rospy.Publisher('/robotis_op2/head_tilt_position/command', Float64, queue_size=10)
        self.pub_r_sho_roll_response = rospy.Publisher('/robotis_op2/r_sho_roll_position/command', Float64, queue_size=10)
        self.pub_r_sho_pitch_response = rospy.Publisher('/robotis_op2/r_sho_pitch_position/command', Float64, queue_size=10)
        self.pub_r_el_response = rospy.Publisher('/robotis_op2/r_el_position/command', Float64, queue_size=10)
        self.pub_l_sho_roll_response = rospy.Publisher('/robotis_op2/l_sho_roll_position/command', Float64, queue_size=10)
        self.pub_l_sho_pitch_response = rospy.Publisher('/robotis_op2/l_sho_pitch_position/command', Float64, queue_size=10)
        self.pub_l_el_response = rospy.Publisher('/robotis_op2/l_el_position/command', Float64, queue_size=10)
        self.pub_x_velocity_response = rospy.Publisher('/robotis_op2/cmd_vel', Twist, queue_size=10)

        # Create a subscriber to get the emotions that the robot sees right now.
        self.emotion_subscriber = rospy.Subscriber("emotions", String, self.callback_emotion_subscriber)
        self.rate = rospy.Rate(10) # 10hz


        # The valid emotions and the corresonding motion array which is the value in degrees for each joint in order, Can be changed according to the Motion table as we want.
        # each array means [head_pan, head_tilt, r_sho_roll], r_sho_pitch, r_el, l_sho_roll, l_sho_pitch, l_el
        self.responses_dict = {
            "Angry": [0.0, -1.17, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "Disgusted": [1.17, 0.0, -1.0, 0.0, -1.5, 0.0, 0.0, 0.0],
            "Fearful": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,-1.2],
            "Happy": [0.0, 0.0, 2.5, 0.0, -0.7, -2.5, 0.0, 0.7],
            "Neutral": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,2.0],
            "Sad": [1.17, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "Surprised": [0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.5]}


    # A method that gets the emotion that is recieved on the /emotions topic and returns the coresponding array from the responses_dict.
    def get_emotion(self, emotion):
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
        self.get_motion_array_output = self.get_emotion(self.recieved_emotion)

        # if the recieved emotion was valid or not
        if self.get_motion_array_output == "Emotion not found":
            rospy.loginfo("Received emotion is not valid: %s ... %s", msg.data,self.get_motion_array_output)
            return
        

        # put the output of the get_motion_array_output function in a variable called responses array.
        self.responses_array = self.get_motion_array_output
        

        rospy.loginfo("Proper responses array: %s", self.responses_array)
        
        rospy.loginfo("Publishing the proper response...!")


        # Unsubscribe from the emotions topic until we finish responding to the last emotion recieved.
        self.emotion_subscriber.unregister()


        # Define a ROS msg of type Twist to publish a valocity command to the robot when the response is to walk forward or backward
        self.twist_msg = Twist()
        if self.recieved_emotion == "Fearful" or self.recieved_emotion == "Neutral":
            self.twist_msg.linear.x = self.responses_array[8]
            self.twist_msg.linear.y = 0.0
            self.twist_msg.linear.z = 0.0
            self.twist_msg.angular.x = 0.0
            self.twist_msg.angular.y = 0.0
            self.twist_msg.angular.z = 0.0
            self.pub_x_velocity_response.publish(self.twist_msg)
            # Wait until the robot finishes the action
            rospy.sleep(5)
        else:
            # Publish each value in the responses array to trhe corresponding joint.
            self.pub_head_pan_response.publish(self.responses_array[0])
            self.pub_head_tilt_response.publish(self.responses_array[1])
            self.pub_r_sho_roll_response.publish(self.responses_array[2])
            self.pub_r_sho_pitch_response.publish(self.responses_array[3])
            self.pub_r_el_response.publish(self.responses_array[4])
            self.pub_l_sho_roll_response.publish(self.responses_array[5])
            self.pub_l_sho_pitch_response.publish(self.responses_array[6])
            self.pub_l_el_response.publish(self.responses_array[7])
            # Wait until the robot finishes the action
            rospy.sleep(5)

        # Publish zero radians for all joints to return to make sure the initial position is reached after each ation
        self.pub_head_pan_response.publish(0.0)
        self.pub_head_tilt_response.publish(0.0)
        self.pub_r_sho_roll_response.publish(0.0)
        self.pub_r_sho_pitch_response.publish(0.0)
        self.pub_r_el_response.publish(0.0)
        self.pub_l_sho_roll_response.publish(0.0)
        self.pub_l_sho_pitch_response.publish(0.0)
        self.pub_l_el_response.publish(0.0)
        self.twist_msg.linear.x = 0.0
        self.pub_x_velocity_response.publish(self.twist_msg)

        rospy.loginfo("Done...!")

        # Wait until the robot finishes the action
        rospy.sleep(5)

        # Resubscribe to the emotions topic to recieve a new emotion and respond to it.
        self.emotion_subscriber = rospy.Subscriber("emotions", String, self.callback_emotion_subscriber)

        
        rospy.loginfo("Resubscribed...!")


        self.rate.sleep()    
        return

if __name__ == '__main__':
    try:
        rospy.init_node('ksa_control', anonymous=True)
        KSA_CONTROL()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass