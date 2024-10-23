import os
import cv2
import rosbag
import numpy as np
from tqdm import tqdm  # Progress bar
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import argparse  # For command-line arguments

# Normalize the image data to the range [0, 255]
def normalize_image(image):
    try:
        normalized_image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
        return normalized_image.astype(np.uint8)
    except Exception as e:
        print(f"Error normalizing image: {e}")
        return image

# Apply a color map to a thermal image
def apply_colormap(image):
    try:
        colored_image = cv2.applyColorMap(image, cv2.COLORMAP_BONE)  # Using COLORMAP_BONE as requested
        return colored_image
    except Exception as e:
        print(f"Error applying color map: {e}")
        return image

def save_comparison_image(original_image, output_image, output_path, bag_name, camera_name):
    # Ensure both images are 3-channel for hstack
    if len(original_image.shape) == 2:
        original_image_color = cv2.cvtColor(original_image, cv2.COLOR_GRAY2BGR)
    else:
        original_image_color = original_image

    if len(output_image.shape) == 2:
        output_image_color = cv2.cvtColor(output_image, cv2.COLOR_GRAY2BGR)
    else:
        output_image_color = output_image

    comparison_image = np.hstack((original_image_color, output_image_color))

    output_filename = os.path.join(output_path, f"comparison_{camera_name}_{bag_name}.png")
    cv2.imwrite(output_filename, comparison_image)
    print(f"Saved comparison image {output_filename}")

def process_bag(bag_file, left_ir_topic, right_ir_topic, output_dir, compressed=False, threshold=None):
    bag = rosbag.Bag(bag_file, 'r')
    bridge = CvBridge()

    output_bag_file = os.path.join(output_dir, os.path.basename(bag_file))
    comparison_output_dir = os.path.join(output_dir, "comparison_images")
    
    if not os.path.exists(comparison_output_dir):
        os.makedirs(comparison_output_dir)

    total_messages = bag.get_message_count(topic_filters=[left_ir_topic, right_ir_topic])
    left_comparison_saved = False  
    right_comparison_saved = False  

    with rosbag.Bag(output_bag_file, 'w') as out_bag:
        print(f"Processing bag: {bag_file}")
        for topic, msg, t in tqdm(bag.read_messages(), total=total_messages, desc=f"Processing {bag_file}"):
            if topic == left_ir_topic or topic == right_ir_topic:
                is_left_camera = topic == left_ir_topic
                camera_name = "left_ir_camera" if is_left_camera else "right_ir_camera"
                comparison_saved = left_comparison_saved if is_left_camera else right_comparison_saved

                if compressed:
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)  # Thermal images are grayscale

                    # Normalize the image
                    normalized_image = normalize_image(cv_image)

                    if threshold is not None:
                        # Ensure threshold is within [0, 255]
                        threshold = np.clip(threshold, 0, 255)
                        # Apply thresholding
                        _, thresholded_image = cv2.threshold(normalized_image, threshold, 255, cv2.THRESH_TOZERO)
                        # Output image is thresholded image
                        output_image = thresholded_image
                    else:
                        # Apply colormap to normalized image
                        output_image = apply_colormap(normalized_image)

                    # Save the comparison image (original vs. output_image)
                    if not comparison_saved:
                        save_comparison_image(cv_image, output_image, comparison_output_dir, os.path.basename(bag_file), camera_name)
                        if is_left_camera:
                            left_comparison_saved = True
                        else:
                            right_comparison_saved = True

                    # Encode and write the image
                    if len(output_image.shape) == 2:  # Grayscale image
                        success, encoded_image = cv2.imencode('.jpg', output_image)
                    else:
                        success, encoded_image = cv2.imencode('.jpg', output_image)

                    if success:
                        undistorted_cv_image = encoded_image.tobytes()
                        compressed_image_msg = CompressedImage()
                        compressed_image_msg.data = undistorted_cv_image
                        compressed_image_msg.format = msg.format  
                        compressed_image_msg.header = msg.header  
                        
                        print(f"Writing processed image to {camera_name} at time {t.to_sec()}")
                        out_bag.write(topic, compressed_image_msg, t)
                    else:
                        print(f"Failed to encode processed image for {camera_name} at time {t.to_sec()}")
                else:
                    # For uncompressed images
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

                    # Normalize the image
                    normalized_image = normalize_image(cv_image)

                    if threshold is not None:
                        # Ensure threshold is within [0, 255]
                        threshold = np.clip(threshold, 0, 255)
                        # Apply thresholding
                        _, thresholded_image = cv2.threshold(normalized_image, threshold, 255, cv2.THRESH_TOZERO)
                        output_image = thresholded_image
                    else:
                        # Apply colormap to normalized image
                        output_image = apply_colormap(normalized_image)

                    # Save the comparison image
                    if not comparison_saved:
                        save_comparison_image(cv_image, output_image, comparison_output_dir, os.path.basename(bag_file), camera_name)
                        if is_left_camera:
                            left_comparison_saved = True
                        else:
                            right_comparison_saved = True

                    # Convert back to ROS Image message
                    if len(output_image.shape) == 2:  # Grayscale image
                        undistorted_msg = bridge.cv2_to_imgmsg(output_image, encoding="mono8")
                    else:
                        undistorted_msg = bridge.cv2_to_imgmsg(output_image, encoding="bgr8")
                    undistorted_msg.header = msg.header 
                    print(f"Writing processed image to {camera_name} at time {t.to_sec()}")

                    out_bag.write(topic, undistorted_msg, t)
            else:
                out_bag.write(topic, msg, t)
        
        print(f"Processed and saved images for {bag_file}. Bag saved at {output_bag_file}")

def process_bag_directory(bag_dir, left_ir_topic, right_ir_topic, output_dir, compressed=False, threshold=None):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    for bag_file in os.listdir(bag_dir):
        if bag_file.endswith(".bag"):
            bag_path = os.path.join(bag_dir, bag_file)
            process_bag(bag_path, left_ir_topic, right_ir_topic, output_dir, compressed=compressed, threshold=threshold)

if __name__ == "__main__":
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="Process ROS bag files to apply colormap or threshold to thermal images.")
    parser.add_argument('--bag_dir', required=True, help='Directory containing ROS bag files to process.')
    parser.add_argument('--output_dir', required=True, help='Directory to save processed ROS bag files.')
    parser.add_argument('--left_ir_topic', default='/left_ir_camera/image_raw/compressed', help='Left IR camera topic.')
    parser.add_argument('--right_ir_topic', default='/right_ir_camera/image_raw/compressed', help='Right IR camera topic.')
    parser.add_argument('--compressed', action='store_true', help='Set this flag if images are compressed.')
    parser.add_argument('--threshold', type=float, default=None, help='Threshold value for image processing. If not set, colormap is applied.')

    args = parser.parse_args()

    process_bag_directory(
        bag_dir=args.bag_dir,
        left_ir_topic=args.left_ir_topic,
        right_ir_topic=args.right_ir_topic,
        output_dir=args.output_dir,
        compressed=args.compressed,
        threshold=args.threshold
    )
