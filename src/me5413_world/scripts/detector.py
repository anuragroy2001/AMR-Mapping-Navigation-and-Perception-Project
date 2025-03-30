#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import String
from vision_msgs.msg import Detection2D
import os
import message_filters

template_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "3.png")
# template_path = "me5413_world/scripts/3.png"
firsttrack_list = []
firsttrack_list.append((0, 0, 106, 137))


class Detector(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_sub = message_filters.Subscriber("/front/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber('/front/depth/image_raw', Image)
        self.template_sub = rospy.Subscriber("/rviz_panel/goal_name", String, self.template_callback)

        self.ats = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.01)
        self.ats.registerCallback(self.synced_images_callback) 

        # Template for matching, obtained from the first frame.
        self.template = None
        self.template_coords = firsttrack_list[0]  # Initial tracking coordinates (x, y, w, h)
        self.template_path = template_path
        self.done = False

        self.depth_image = None

        # Initialize a publisher for sending msg
        self.matrix_pub = rospy.Publisher("/me5413/student_matrix", String, queue_size=10)
        self.detected_pub = rospy.Publisher("/me5413/detected", Detection2D, queue_size=10)
        self.current_depth_pub = rospy.Publisher("/me5413/current_depth", Image, queue_size=10)
        
        # Add debug publisher
        self.debug_pub = rospy.Publisher("/me5413/debug", String, queue_size=10)
        # And publish a message every 5 seconds to verify the node is running
        rospy.Timer(rospy.Duration(5), self.debug_timer_callback)

    def template_callback(self, data):
        print(f"Received message on /rviz_panel/goal_name: '{data.data}'")
        if "box" not in data.data:
            self.template = None
            print("No 'box' in message, template set to None")
            return

        try:
            # Read the template img from the path
            print(f"Attempting to load template from: {self.template_path}")
            template_img = cv2.imread(self.template_path)
            if template_img is None:
                rospy.logerr("Failed to load template image from path: {}".format(self.template_path))
                print("OpenCV returned None when loading the image")
            else:
                rospy.loginfo("Template image loaded successfully from path: {}".format(self.template_path))
                print(f"Template image shape: {template_img.shape}")
                x, y, w, h = self.template_coords
                self.template = template_img[y:y+h, x:x+w]
                print(f"Cropped template shape: {self.template.shape}")

        except Exception as e:
            rospy.logerr("Error loading template image: {}".format(e))
            print(f"Exception details: {str(e)}")

    def synced_images_callback(self, rgb_data, depth_data):
        print(f"Received synced images: RGB shape={rgb_data.height}x{rgb_data.width}, encoding={rgb_data.encoding}")
        
        if self.template is None:
            print("Template is None, publishing empty detection")
            detection = Detection2D()
            detection.bbox.size_x = 0
            detection.bbox.size_y = 0
            detection.bbox.center.x = 0
            detection.bbox.center.y = 0
            detection.source_img = rgb_data
            self.detected_pub.publish(detection)
            return
        
        try:
            print("Converting ROS image to OpenCV image")
            cv_image = self.bridge.imgmsg_to_cv2(rgb_data, "bgr8")  # Changed to "bgr8" for consistency
            print(f"OpenCV image shape: {cv_image.shape}")
            current_depth = depth_data
            self.detect(cv_image, current_depth)
        except CvBridgeError as e:
            print(f"CvBridge error: {e}")

    def detect(self, image, current_depth):
        print("Starting detection...")
        
        image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        processed_image = image_gray
        template_gray = cv2.cvtColor(self.template, cv2.COLOR_BGR2GRAY)
        processed_template = template_gray
        
        print(f"Image dimensions: {processed_image.shape}")
        print(f"Template dimensions: {processed_template.shape}")

        original_height, original_width = processed_template.shape[:2]
        scales = [0.5, 0.75, 1.0, 1.25, 1.5]
        max_match_val = -1
        best_scale = None
        best_max_loc = None

        for scale in scales:
            new_width = int(original_width * scale)
            new_height = int(original_height * scale)
            
            resized_template = cv2.resize(processed_template, (new_width, new_height),
                                          interpolation=cv2.INTER_AREA if scale < 1.0 else cv2.INTER_CUBIC)
            
            result = cv2.matchTemplate(processed_image, resized_template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)
            
            print(f"Scale {scale}: match value = {max_val}, location = {max_loc}")

            if max_val > max_match_val:
                max_match_val = max_val
                best_scale = scale
                best_max_loc = max_loc
        
        print(f"Best match: value={max_match_val}, scale={best_scale}, location={best_max_loc}")

        # Only the match with score > 0.2 is considered as a valid detection
        if max_match_val > 0.75:
            print(f"DETECTION SUCCESSFUL with confidence {max_match_val}")
            x_d, y_d = best_max_loc
            width = int(original_width * best_scale)
            height = int(original_height * best_scale)
            
            # Draw bounding box for visualization
            vis_image = image.copy()
            cv2.rectangle(vis_image, (x_d, y_d), (x_d + width, y_d + height), (0, 255, 0), 2)
            cv2.putText(vis_image, f"Score: {max_match_val:.2f}", (x_d, y_d-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Save debug image to disk to verify detection
            debug_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "debug_detection.jpg")
            cv2.imwrite(debug_path, vis_image)
            print(f"Saved debug image to {debug_path}")
        else:
            print(f"No detection. Best confidence was only {max_match_val}")
            x_d, y_d, width, height = 0, 0, 0, 0

        print(f"Publishing detection: x={x_d}, y={y_d}, w={width}, h={height}")
        self.publish_detection(x_d, y_d, width, height, image, current_depth)

    def publish_detection(self, x, y, width, height, img, current_depth):
        print(f"Creating Detection2D message with bbox: center=({x + width//2}, {y + height//2}), size=({width}, {height})")
        detection = Detection2D()
        detection.bbox.size_x = width
        detection.bbox.size_y = height
        detection.bbox.center.x = x + width // 2
        detection.bbox.center.y = y + height // 2
        
        try:
            print("Converting image for detection message")
            ros_img = self.bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            print(f"CvBridge error in publish_detection: {e}")
        
        detection.source_img = ros_img
        
        print("Publishing to /me5413/detected")
        self.detected_pub.publish(detection)
        print("Publishing to /me5413/current_depth")
        self.current_depth_pub.publish(current_depth)
        print("Publishing to /me5413/student_matrix")
        self.matrix_pub.publish("A0285282X")
        print("All messages published successfully")

    def debug_timer_callback(self, event):
        self.debug_pub.publish("Detector node is running")
        print("Debug timer fired, node is running")


def main():
    rospy.init_node('detector', anonymous=True)
    det = Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
