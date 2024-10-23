import os
import cv2
import rosbag
import yaml
import numpy as np
from tqdm import tqdm  # Progress bar
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def load_calibration(calibration_file):
    print(f"Loading calibration from {calibration_file}")
    with open(calibration_file, "r") as file:
        calib_data = yaml.safe_load(file)
        
    K = np.array(calib_data['camera_matrix']['data']).reshape(3, 3)
    D = np.array(calib_data['distortion_coefficients']['data']).reshape(1, 4)
    DIM = (calib_data['image_width'], calib_data['image_height'])
    
    print(f"Calibration loaded: K={K}, D={D}, DIM={DIM}")
    return K, D, DIM

def undistort_fisheye(image, K, D, DIM):
    print("Undistorting image...")
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_image = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_image

def save_comparison(original_image, undistorted_image, output_path, bag_name, camera_name):
    # Resize images to the same height for better comparison
    h = min(original_image.shape[0], undistorted_image.shape[0])
    original_resized = cv_image_resize(original_image, height=h)
    undistorted_resized = cv_image_resize(undistorted_image, height=h)
    
    comparison_image = np.hstack((original_resized, undistorted_resized))
    
    output_filename = os.path.join(output_path, f"comparison_{camera_name}_{bag_name}.png")
    cv2.imwrite(output_filename, comparison_image)
    print(f"Saved comparison image {output_filename}")

def cv_image_resize(image, height=None, width=None):
    h, w = image.shape[:2]
    if height is not None:
        aspect_ratio = height / float(h)
        dim = (int(w * aspect_ratio), height)
    elif width is not None:
        aspect_ratio = width / float(w)
        dim = (width, int(h * aspect_ratio))
    else:
        return image
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

def process_bag(bag_file, left_camera_topic, right_camera_topic, left_calibration_file, right_calibration_file, output_dir, compressed=False):
    left_K, left_D, left_dimensions = load_calibration(left_calibration_file)
    right_K, right_D, right_dimensions = load_calibration(right_calibration_file)
    
    bag = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()

    output_bag_file = os.path.join(output_dir, os.path.basename(bag_file))
    comparison_output_dir = os.path.join(output_dir, "comparisons")
    
    if not os.path.exists(comparison_output_dir):
        os.makedirs(comparison_output_dir)

    total_messages = bag.get_message_count(topic_filters=[left_camera_topic, right_camera_topic])
     # Flag to save only one comparison image
    left_comparison_saved = False  
    right_comparison_saved = False  

    with rosbag.Bag(output_bag_file, 'w') as out_bag:
        print(f"Processing bag: {bag_file}")
        for topic, msg, t in tqdm(bag.read_messages(), total=total_messages, desc=f"Processing {bag_file}"):
            if topic == left_camera_topic or topic == right_camera_topic:
                is_left_camera = topic == left_camera_topic
                K, D, dimensions = (left_K, left_D, left_dimensions) if is_left_camera else (right_K, right_D, right_dimensions)
                camera_name = "left_camera" if is_left_camera else "right_camera"
                comparison_saved = left_comparison_saved if is_left_camera else right_comparison_saved

                if compressed:
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    undistorted_image = undistort_fisheye(cv_image, K, D, dimensions)

                    if not comparison_saved:
                        save_comparison(cv_image, undistorted_image, comparison_output_dir, os.path.basename(bag_file), camera_name)
                        if is_left_camera:
                            left_comparison_saved = True
                        else:
                            right_comparison_saved = True

                    success, encoded_image = cv2.imencode('.jpg', undistorted_image)
                    if success:
                        undistorted_cv_image = encoded_image.tobytes()
                        compressed_image_msg = CompressedImage()
                        compressed_image_msg.data = undistorted_cv_image
                        compressed_image_msg.format = msg.format  
                        compressed_image_msg.header = msg.header  
                        
                        print(f"Writing undistorted image to {camera_name} at time {t.to_sec()}")
                        out_bag.write(topic, compressed_image_msg, t)
                    else:
                        print(f"Failed to encode undistorted image for {camera_name} at time {t.to_sec()}")
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                    undistorted_image = undistort_fisheye(cv_image, K, D, dimensions)

                    if not comparison_saved:
                        save_comparison(cv_image, undistorted_image, comparison_output_dir, os.path.basename(bag_file), camera_name)
                        if is_left_camera:
                            left_comparison_saved = True
                        else:
                            right_comparison_saved = True

                    undistorted_msg = bridge.cv2_to_imgmsg(undistorted_image, encoding="bgr8")
                    undistorted_msg.header = msg.header 
                    print(f"Writing undistorted image to {camera_name} at time {t.to_sec()}")

                    out_bag.write(topic, undistorted_msg, t)
            else:
                out_bag.write(topic, msg, t)
    
    print(f"Processed and saved undistorted images for {bag_file}. Bag saved at {output_bag_file}")

def process_bag_directory(bag_dir, left_camera_topic, right_camera_topic, left_calibration_file, right_calibration_file, output_dir, compressed=False):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    for bag_file in os.listdir(bag_dir):
        if bag_file.endswith(".bag"):
            bag_path = os.path.join(bag_dir, bag_file)
            process_bag(bag_path, left_camera_topic, right_camera_topic, left_calibration_file, right_calibration_file, output_dir, compressed=compressed)

if __name__ == "__main__":
    # Hardcoded file paths and parameters
    bag_dir = "/var/home/dan/catkin_ws/src/lvt2calib/data/rgb-thermal-calibration/right_camera"
    left_camera_topic = "/left_camera/image_color/compressed" 
    right_camera_topic = "/right_camera/image_color/compressed" 
    left_calibration_file = "/var/home/dan/catkin_ws/src/lvt2calib/data/rgb-thermal-calibration/left_camera/left_ost.yaml"
    right_calibration_file = "/var/home/dan/catkin_ws/src/lvt2calib/data/rgb-thermal-calibration/right_camera/right_ost.yaml"
    output_dir = "/var/home/dan/catkin_ws/src/lvt2calib/data/rgb-thermal-calibration/right_camera_rectified"
    compressed = True  # Set to True if you're using compressed images, else False
    
    process_bag_directory(bag_dir, left_camera_topic, right_camera_topic, left_calibration_file, right_calibration_file, output_dir, compressed=compressed)
