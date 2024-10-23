import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

# Normalize the image data to the range [0, 255]
def normalize_image(image):
    try:
        # Normalize the image to span the full grayscale range [0, 255]
        normalized_image = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)
        return normalized_image
    except Exception as e:
        print(f"Error normalizing image: {e}")
        return image  # Return original image in case of failure

# Apply a color map to a thermal image
def apply_colormap(image):
    try:
        # Apply a colormap (e.g., COLORMAP_BONE) to the grayscale thermal image
        colored_image = cv2.applyColorMap(image, cv2.COLORMAP_BONE)
        return colored_image
    except Exception as e:
        print(f"Error applying color map: {e}")
        return image  # Return original image in case of failure

# Image callback function for compressed thermal image
def thermal_image_callback(msg, args):
    bridge, colormap_pub, camera_name = args
    print(f"Compressed thermal image received for {camera_name}. Decoding...")

    try:
        # Fetch the latest threshold value dynamically
        threshold_param = rospy.get_param("/thermal_image_colormap_dual/threshold", None)
        print(f"Current threshold parameter value: {threshold_param}")

        # Determine the threshold value
        if threshold_param is None or str(threshold_param).lower() == 'none':
            threshold = None
        else:
            try:
                threshold = float(threshold_param)
            except ValueError:
                print(f"Invalid threshold value: {threshold_param}. Using None.")
                threshold = None

        # Convert compressed image message to OpenCV image
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)  # Decoding as grayscale image

        # Normalize the thermal image to [0, 255]
        normalized_image = normalize_image(cv_image)

        # Apply thresholding if threshold is not None
        if threshold is not None:
            # Ensure threshold is within [0, 255]
            threshold = np.clip(threshold, 0, 255)
            # Apply threshold: set pixels below threshold to zero
            _, thresholded_image = cv2.threshold(normalized_image, threshold, 255, cv2.THRESH_TOZERO)
            # Do not apply colormap; use the normalized and thresholded image
            output_image = thresholded_image
        else:
            # Apply the color map to the normalized image
            output_image = apply_colormap(normalized_image)

        # Ensure the image is uint8
        output_image = output_image.astype(np.uint8)

        # Convert the image back to ROS message and publish
        colored_image_msg = bridge.cv2_to_compressed_imgmsg(output_image)
        colored_image_msg.header = msg.header  # Preserve the header
        colormap_pub.publish(colored_image_msg)
        print(f"Published thermal image for {camera_name}.")

    except Exception as e:
        print(f"Error in thermal image callback for {camera_name}: {e}")

def main():
    rospy.init_node("thermal_image_colormap_dual", anonymous=True)

    # Topics for the left and right IR cameras
    left_thermal_camera_topic = "/left_ir_camera/image_raw/compressed"
    right_thermal_camera_topic = "/right_ir_camera/image_raw/compressed"

    # Topics for publishing color-mapped images
    left_colored_thermal_topic = "/left_ir_camera/image_colormap/compressed"
    right_colored_thermal_topic = "/right_ir_camera/image_colormap/compressed"

    # Bridge for converting between ROS and OpenCV images
    bridge = CvBridge()

    # Publishers for the thermal images
    left_colormap_pub = rospy.Publisher(left_colored_thermal_topic, CompressedImage, queue_size=10)
    right_colormap_pub = rospy.Publisher(right_colored_thermal_topic, CompressedImage, queue_size=10)

    # Subscribe to both the left and right thermal camera topics
    rospy.Subscriber(left_thermal_camera_topic, CompressedImage, thermal_image_callback,
                     (bridge, left_colormap_pub, "left_ir_camera"))
    rospy.Subscriber(right_thermal_camera_topic, CompressedImage, thermal_image_callback,
                     (bridge, right_colormap_pub, "right_ir_camera"))

    print(f"Subscribed to {left_thermal_camera_topic} and {right_thermal_camera_topic}.")
    print(f"Publishing thermal images to {left_colored_thermal_topic} and {right_colored_thermal_topic}.")

    # Keep the node alive
    rospy.spin()

if __name__ == "__main__":
    main()
