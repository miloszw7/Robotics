#!/usr/bin/env python3
import rospy
import actionlib
import math
import time
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import paho.mqtt.client as mqtt

import time
import json

import cv2, cv_bridge
import os
from sensor_msgs.msg import Image  # Add at top if missing

import RealtimeTurtlebotROIClassification

from collections import deque

navigator = None
##
BROKER_IP = "10.148.187.246"
TOPIC = "rangeTopic"

####

lastdistance = 1000

###

### camera
image_received = False
last_image = None
bridge = cv_bridge.CvBridge()

capture_requested = False  # Global flag

image_num = 0

### Arrays for sound and distance to track spikes

sound_array = deque(maxlen=20)
distance_array = deque(maxlen=20)


def detect_spike(data, threshold=20):
    if len(data) < 5:
        return False  # not enough data yet

    avg = sum(list(data)[:-1]) / (len(data) - 1)
    latest = data[-1]

    return abs(latest - avg) > threshold


###


def image_callback(msg):
    global last_image, image_received
    try:
        last_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    image_received = True
    except cv_bridge.CvBridgeError as e:
    print(e)


def capture_image(num):
    global last_image, image_received, image_num
    if image_received and last_image is not None:
        print("Saving captured image...")
    # cv2.imwrite("grabbed_image.jpg", last_image)
    cv2.imwrite(f"gradded_image{num}.jpg", last_image)
    image_num += 1
    image_received = False  # Reset so next image is fresh
    else:
    print("No image available to save.")


####

once_goal = True


def on_message(client, userdata, msg):
    global lastdistance, navigator, distance_array, sound_array

    if navigator is None:
        print("Navigator not initialized yet. Skipping message.")
    return  # Ignore messages until navigator is ready

    try:
        print(f"{msg.topic} {msg.payload}")
    payload_str = msg.payload.decode('utf-8')  # Decode bytes to string
    data = json.loads(payload_str)  # Parse JSON string
    sound = data["sound"]
    lastdistance = data["distance"]  # Extract the "sound" value
    print(f"SOUND: {sound}")
    print(f"DISTANCE: {lastdistance}")

    # Store recent values
    sound_array.append(sound)
    distance_array.append(lastdistance)

    # Check for spike
    # if detect_spike(distance_array, threshold=10):
    #	print("Spike in distance detected — overriding!")
    #   sound_array = deque(maxlen=20)
    #  distance_array = deque(maxlen=20)
    # client.publish("turtlebot/status", "danger")
    # navigator.override_goal(0.9626055955886841, 1.2596771717071533, 0)

    if lastdistance < 10 and once_goal:
        print("OVERRIDING")
        once_goal = False
        client.publish("turtlebot/status", "danger")
        navigator.override_goal(0.9626055955886841, 1.2596771717071533, 0)

    except (json.JSONDecodeError, KeyError, AttributeError) as e:
    print(f"Failed to process message: {e}")


"""
def on_message(client, userdata, msg):
	print(f"{msg.topic} {msg.payload}")
	sound = msg.payload["sound"]
	print(f"SoUND {sound}")
"""


## helper


##

class GoalNavigator:
    def __init__(self):

    # client = mqtt.Client()
    # client.on_message = on_message
    # client.connect("10.148.187.246", 1883, 60)

    # subscribe.callback(on_message, "rangeTopic", hostname = "10.148.187.246")
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)

    rospy.init_node('movement_publisher', anonymous=False)

    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    self.client.wait_for_server()

    rospy.Subscriber('/odom', Odometry, self.odom_callback)
    self.is_moving = False

    rospy.wait_for_service('/move_base/clear_costmaps')
    self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

    self.current_yaw = 0.0

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Postions - TOP Left pose:
    # pose:
    # position:
    #  x: 0.9277703166007996
    # y: -0.15571722388267517
    # z: 0.0
    # top top right
    # position:
    # x: 2.1603753566741943
    # y: 1.2811906337738037
    # z: 0.0
    # MIdle LANE
    #  position:
    # x: 1.756173849105835
    # y: 0.48169469833374023
    # z: 0.0

    #

    self.goals = [
        (0.9277703166007996, -0.15571722388267517, 360),  # Bottom RIght
        (2.1603753566741943, 1.2811906337738037, 360)  # Top TOp right corner
        # (1.756173849105835,0.48169469833374023,0.0), #Middle lane
        # (0.9626055955886841, 1.2596771717071533, 0.0), #TOP left main
        #  (0.9277703166007996, -0.15571722388267517,0)  #Bottom RIght
    ]
    self.current_goal_index = 0

    self.send_next_goal()

    self.count2 = 0

    def odom_callback(self, msg):
        linear_velocity = msg.twist.twist.linear.x

    angular_velocity = msg.twist.twist.angular.z
    self.is_moving = abs(linear_velocity) > 0.02 or abs(angular_velocity) > 0.02
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (_, _, yaw) = euler_from_quaternion(orientation_list)
    self.current_yaw = yaw  # Store current yaw in radians

    def send_goal(self, x, y):
        """Send a goal to move_base (only position)."""

    global once
    self.clear_costmaps()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo(f"Sending position goal: x={x}, y={y}")
    self.client.send_goal(goal)
    self.client.wait_for_result()

    if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(" Position reached! Rotating to correct orientation...")
        print(f"CAalling Camer")

        return True
    else:
        rospy.logwarn(" Failed to reach position!")
        return False

    def rotate_for_duration(self, duration_seconds=10):
        global client

    rospy.loginfo(f"Rotating for {duration_seconds} seconds")

    rotation_speed = 0.3  # Angular speed in radians/sec
    twist = Twist()
    twist.angular.z = rotation_speed  # Set counterclockwise rotation

    rate = rospy.Rate(10)  # 10 Hz loop rate
    start_time = rospy.Time.now()

    photo_num = 0
    last_photo_time = -2  # So the first one happens at 0

    while not rospy.is_shutdown():
        elapsed_time = (rospy.Time.now() - start_time).to_sec()
        if elapsed_time >= duration_seconds:
            break

        # Take photo every 2 seconds
        if elapsed_time - last_photo_time >= 2.0:
            rospy.loginfo(f"Capturing photo #{photo_num}")
        capture_image(photo_num)
        photo_num += 1
        last_photo_time = elapsed_time
        # client.publish("turtlebot/status", "rotating")

        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Rotating... {elapsed_time:.1f} / {duration_seconds} seconds")
        rate.sleep()

    # Stop the robot after the duration
    twist.angular.z = 0
    self.cmd_vel_pub.publish(twist)

    rospy.loginfo("Rotation complete.")
    capture_image(photo_num)

    intrude = False
    for i in range(photo_num + 1):
        prediction = RealtimeTurtlebotROIClassification.classify_Image(i)
    # if prediction == "pink":
    #	intrude = True
    #	client.publish("turtlebot/status", "intruder")
    # if not intrude:
    #	client.publish("turtlebot/status", "safe")
    if self.count2 == 1:
        client.publish(client.publish("turtlebot/status", "danger"))
    else:
        client.publish("turtlebot/status", "safe")

    self.count2 += 1

    def send_next_goal(self):
        """Send the next goal when the robot stops moving."""

    if self.current_goal_index >= len(self.goals):
        self.current_goal_index = 0
    if self.current_goal_index < len(self.goals):
        x, y, yaw = self.goals[self.current_goal_index]

        if self.send_goal(x, y):
        # self.rotate_to_yaw(yaw)
        self.rotate_for_duration()

        self.current_goal_index += 1
        rospy.Timer(rospy.Duration(1), self.wait_until_stop, oneshot=True)
    else:
        rospy.loginfo(" All goals reached!")

    def wait_until_stop(self, event):
        """Wait until the robot stops before sending the next goal."""

    if not self.is_moving:
        rospy.loginfo("Robot stopped. Sending next goal...")
        self.send_next_goal()
    else:
        rospy.Timer(rospy.Duration(1), self.wait_until_stop, oneshot=True)

    def override_goal(self, x, y, yaw):
        """Send an immediate override goal."""

    self.client.cancel_all_goals()
    self.clear_costmaps()
    rospy.loginfo(f"OVERRIDE: Navigating to override goal at x={x}, y={y}")
    self.send_goal(x, y)
    self.rotate_for_duration(yaw)


def main():
    global navigator, client
    try:
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(BROKER_IP, 1883, 60)
    client.subscribe(TOPIC)
    client.loop_start()
    client.publish("turtlebot/status", "startig")

    if navigator == None:
        print(f"NOt NONE")
    # rospy.spin()
    except rospy.ROSInterruptException:
    pass


if __name__ == "__main__":
    main()
    navigator = GoalNavigator()

    rospy.spin()

import os
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
from cv_bridge import CvBridgeError
import numpy as np
import pickle
import pandas as pd
import json
from time import sleep

image_received = False
last_image = None
bridge = cv_bridge.CvBridge()


def load_model(filename):
    # Loads a trained model from a file
    with open(filename, 'rb') as file:
        return pickle.load(file)


def extract_central_roi(image, roi_size=144):
    # Extracts a central 144x144 ROI from the image
    h, w, _ = image.shape
    roi_x = (w // 2) - (roi_size // 2)
    roi_y = (h // 2) - (roi_size // 2)
    roi = image[roi_y:roi_y + roi_size, roi_x:roi_x + roi_size]
    return roi


def extract_color_features(image):
    # Extracts mean and std of RGB and HSV channels from the ROI image
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    features = {}
    for i, color in enumerate(['R', 'G', 'B']):
        features[f'mean_{color}'] = np.mean(image_rgb[:, :, i])
    features[f'std_{color}'] = np.std(image_rgb[:, :, i])

    for i, color in enumerate(['H', 'S', 'V']):
        features[f'mean_{color}'] = np.mean(image_hsv[:, :, i])
    features[f'std_{color}'] = np.std(image_hsv[:, :, i])

    return features


def classify_postit(svm_model_file, image_path):
    # Loads SVM model, extracts features from a grabbed image, and predicts class.
    # Load pre-trained model
    model = load_model(svm_model_file)

    # Load and process image
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Image not found.")
    return None

    roi = extract_central_roi(image)
    features = extract_color_features(roi)

    # Convert features to model input format as a DataFrame to match training format
    feature_df = pd.DataFrame([features])

    # Predict class
    predicted_class = model.predict(feature_df)[0]
    print(f"Predicted Post-it Note Colour: {predicted_class}")
    return predicted_class


def classify_Image(num):
    image_path = f"gradded_image{num}.jpg"
    predicted_class = classify_postit("svm_model.pkl", image_path)
    if predicted_class:
        print(f"Published classification result {predicted_class}")

    return predicted_class
