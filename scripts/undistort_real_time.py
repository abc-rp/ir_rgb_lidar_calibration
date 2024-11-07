#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Load calibration file
def load_calibration(calibration_file):
    print(f"Loading calibration from {calibration_file}")
    calib_data = {}
    with open(calibration_file, "r") as file:
        for line in file:
            if ':' in line:
                key, value = line.strip().split(':', 1)
                calib_data[key.strip()] = value.strip()
    # Parse K and D
    K_values = [float(x) for x in calib_data['K'].split()]
    D_values = [float(x) for x in calib_data['D'].split()]
    K = np.array(K_values).reshape((3, 3))
    D = np.array(D_values).reshape((len(D_values), 1))
    print(f"Calibration loaded: K={K}, D={D}")
    return K, D

# Undistort image using fisheye calibration
def undistort_fisheye(image, K, D):
    h, w = image.shape[:2]
    DIM = (w, h)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), K, DIM, cv2.CV_16SC2
    )
    undistorted_image = cv2.remap(
        image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
    )
    return undistorted_image

# Image callback function for compressed image
def image_callback(msg, args):
    K, D, bridge, undistorted_pub, camera_name = args
    print(f"Compressed image received for {camera_name}. Decoding...")

    try:
        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Undistort the image
        undistorted_image = undistort_fisheye(cv_image, K, D)

        # Convert undistorted image back to ROS message and publish
        undistorted_msg = bridge.cv2_to_compressed_imgmsg(undistorted_image)
        undistorted_msg.header = msg.header  # Preserve the header
        undistorted_pub.publish(undistorted_msg)
        print(f"Published undistorted image for {camera_name}.")

    except Exception as e:
        print(f"Error in image callback for {camera_name}: {e}")

def main():
    rospy.init_node("image_undistorter", anonymous=True)

    # Use ROS parameters or defaults
    left_camera_topic = rospy.get_param("~left_camera_topic", "/left_camera/image_color/compressed")
    right_camera_topic = rospy.get_param("~right_camera_topic", "/right_camera/image_color/compressed")

    undistorted_left_topic = rospy.get_param("~undistorted_left_topic", "/left_camera/image_color/undistorted/compressed")
    undistorted_right_topic = rospy.get_param("~undistorted_right_topic", "/right_camera/image_color/undistorted/compressed")

    # Set calibration file paths directly
    left_calibration_file = "/var/home/dan/catkin_ws/src/ir_rgb_lidar_calibration/data/camera_info/BESS2/left_camera.txt"
    right_calibration_file = "/var/home/dan/catkin_ws/src/ir_rgb_lidar_calibration/data/camera_info/BESS2/right_camera.txt"

    # Load calibration parameters for both cameras
    left_K, left_D = load_calibration(left_calibration_file)
    right_K, right_D = load_calibration(right_calibration_file)

    # Bridge for converting between ROS and OpenCV images
    bridge = CvBridge()

    # Publishers for the undistorted images
    undistorted_left_pub = rospy.Publisher(undistorted_left_topic, CompressedImage, queue_size=10)
    undistorted_right_pub = rospy.Publisher(undistorted_right_topic, CompressedImage, queue_size=10)

    # Subscribe to both camera topics
    rospy.Subscriber(left_camera_topic, CompressedImage, image_callback, 
                     (left_K, left_D, bridge, undistorted_left_pub, "left_camera"))
    rospy.Subscriber(right_camera_topic, CompressedImage, image_callback, 
                     (right_K, right_D, bridge, undistorted_right_pub, "right_camera"))

    print(f"Subscribed to {left_camera_topic} and {right_camera_topic}.")
    print(f"Publishing undistorted images to {undistorted_left_topic} and {undistorted_right_topic}.")

    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()
