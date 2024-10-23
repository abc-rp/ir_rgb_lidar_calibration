import rospy
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Load calibration file
def load_calibration(calibration_file):
    print(f"Loading calibration from {calibration_file}")
    with open(calibration_file, "r") as file:
        calib_data = yaml.safe_load(file)
        
    K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
    D = np.array(calib_data['distortion_coefficients']['data']).reshape(1, 4)
    DIM = (calib_data['image_width'], calib_data['image_height'])
    
    print(f"Calibration loaded: K={K}, D={D}, DIM={DIM}")
    return K, D, DIM

# Undistort image using fisheye calibration
def undistort_fisheye(image, K, D, DIM):
    h, w = image.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_image

# Image callback function for compressed image
def image_callback(msg, args):
    K, D, DIM, bridge, undistorted_pub, camera_name = args
    print(f"Compressed image received for {camera_name}. Decoding...")

    try:
        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Undistort the image
        undistorted_image = undistort_fisheye(cv_image, K, D, DIM)

        # Convert undistorted image back to ROS message and publish
        undistorted_msg = bridge.cv2_to_compressed_imgmsg(undistorted_image)
        undistorted_msg.header = msg.header  # Preserve the header
        undistorted_pub.publish(undistorted_msg)
        print(f"Published undistorted image for {camera_name}.")

    except Exception as e:
        print(f"Error in image callback for {camera_name}: {e}")

def main():
    rospy.init_node("image_undistorter", anonymous=True)

    # File paths and parameters for both cameras
    left_camera_topic = "/left_camera/image_color/compressed"  # Adjust based on your actual topic
    right_camera_topic = "/right_camera/image_color/compressed"  # Adjust based on your actual topic

    undistorted_left_topic = "/left_camera/image_color/undistorted/compressed"
    undistorted_right_topic = "/right_camera/image_color/undistorted/compressed"

    left_calibration_file = "/var/home/dan/catkin_ws/src/lvt2calib/data/rgb-thermal-calibration/left_camera/left_ost.yaml"
    right_calibration_file = "/var/home/dan/catkin_ws/src/lvt2calib/data/rgb-thermal-calibration/right_camera/right_ost.yaml"

    # Load calibration parameters for both cameras
    left_K, left_D, left_DIM = load_calibration(left_calibration_file)
    right_K, right_D, right_DIM = load_calibration(right_calibration_file)

    # Bridge for converting between ROS and OpenCV images
    bridge = CvBridge()

    # Publishers for the undistorted images
    undistorted_left_pub = rospy.Publisher(undistorted_left_topic, CompressedImage, queue_size=10)
    undistorted_right_pub = rospy.Publisher(undistorted_right_topic, CompressedImage, queue_size=10)

    # Subscribe to both camera topics
    rospy.Subscriber(left_camera_topic, CompressedImage, image_callback, 
                     (left_K, left_D, left_DIM, bridge, undistorted_left_pub, "left_camera"))
    rospy.Subscriber(right_camera_topic, CompressedImage, image_callback, 
                     (right_K, right_D, right_DIM, bridge, undistorted_right_pub, "right_camera"))

    print(f"Subscribed to {left_camera_topic} and {right_camera_topic}.")
    print(f"Publishing undistorted images to {undistorted_left_topic} and {undistorted_right_topic}.")

    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()

