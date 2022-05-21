#!/usr/bin/env python3
import rospy
from std_msgs.msg import String,Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import argparse
import matplotlib.pyplot as plt
import cv2
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Flatten
from tensorflow.keras.layers import Conv2D
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.layers import MaxPooling2D
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import sys
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'



model_path = '/home/dozi/catkin_ws2/src/trial/scripts/model.h5'                                 # Absolute Path to the model.h5 file on your local system. 
                                                                                                #Needs to be modified with the file current path.

cascade_path = '/home/dozi/catkin_ws2/src/trial/scripts/haarcascade_frontalface_default.xml'    # Absolute Path to the haarcascade_frontalface_default.xml file on your local system. 
                                                                                                # Needs to be modified with the file current path.


############ Create the Machine Learning Keras model! #############
model = Sequential()

model.add(Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=(48,48,1)))
model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))

model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))

model.add(Flatten())
model.add(Dense(1024, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(7, activation='softmax'))
###################################################################


############## Defining the ROS Publish object ####################
pub = rospy.Publisher('emotions', String, queue_size=10)
bridge = CvBridge()
###################################################################

######### A Function to Show the image in a small window ##########
def show_image(img):
      cv2.imshow("Image Window", img)
      cv2.waitKey(3)
###################################################################


### The Callback function for the ROS Topic /usb_cam/image_raw ####
def image_callback(msg):

    # Convert your ROS Image message to OpenCV2
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Load the ML Trained Model Weights
    model.load_weights(model_path)

    # prevents openCL usage and unnecessary logging messages
    cv2.ocl.setUseOpenCL(False)

    # dictionary which assigns each label an emotion (alphabetical order)
    emotion_dict = {0: "Angry", 1: "Disgusted", 2: "Fearful", 3: "Happy", 4: "Neutral", 5: "Sad", 6: "Surprised"}

    # Take the converted CV2 image and save it in frame variable
    cap = cv2_img
    frame = cap

    # Prepare the CV2 image for the Recognetion algorthim.
    facecasc = cv2.CascadeClassifier(cascade_path)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Start detecting faces in the CV2 image.
    faces = facecasc.detectMultiScale(gray,scaleFactor=1.3, minNeighbors=5)

    # For each face detected run the following lines.
    for (x, y, w, h) in faces:

        # Prepare a rectangle to draw around the detected face.
        cv2.rectangle(frame, (x, y-50), (x+w, y+h+10), (255, 0, 0), 2)

        # Some post proccesing to the CV2 image.
        roi_gray = gray[y:y + h, x:x + w]
        cropped_img = np.expand_dims(np.expand_dims(cv2.resize(roi_gray, (48, 48)), -1), 0)

        # Get the Trained model predection probilities to the emotions displayed in the image.
        prediction = model.predict(cropped_img)

        # Define the one with the maximum probability.
        maxindex = int(np.argmax(prediction))

        # Add Text to the Image with the detected emotion to the corresponding face. (Post proccessing)
        cv2.putText(frame, emotion_dict[maxindex], (x+20, y-60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            
    
    # Show in a Small window the created image.
    show_image(frame)

    # Publish to /emotions topic the detected emotion of the last detected face in the image.
    pub.publish(emotion_dict[maxindex])


# The Main running ROS Loop Function.
def main():

    # Intiallize the ROS Node.
    rospy.init_node('emotions_ML')

    # Define your image topic
    image_topic = "/usb_cam/image_raw"

    # Set up your subscriber and define its callback function
    rospy.Subscriber(image_topic, Image, image_callback)

    # Loop with a rate if 10 Hz.
    rate = rospy.Rate(10) # 10hz

    # Spin until ctrl + c
    rospy.spin()


# The Main loop.
if __name__ == '__main__':

    try:
        main()
    except rospy.ROSInterruptException:
        pass