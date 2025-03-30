#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D

class ObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.detection_sub = rospy.Subscriber("/me5413/detected", Detection2D, self.callback)
        self.window_initialized = False

    def callback(self, data):
        try:
            # Convert image to BGR format (most common for OpenCV)
            cv_image = self.bridge.imgmsg_to_cv2(data.source_img, "bgr8")
            
            # Draw bounding box
            x = int(data.bbox.center.x)
            y = int(data.bbox.center.y)
            w = int(data.bbox.size_x)
            h = int(data.bbox.size_y)
            cv2.rectangle(cv_image, (x-w//2, y-h//2), (x+w//2, y+h//2), (0, 255, 0), 2)

            # Create named window first (helps with some OpenCV backends)
            if not self.window_initialized:
                cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
                self.window_initialized = True
            
            # Display image
            cv2.imshow("Image window", cv_image)
            cv2.waitKey(1)  # Small delay (1ms) to allow GUI updates

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

def main():
    rospy.init_node('object_detector', anonymous=True)
    od = ObjectDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
