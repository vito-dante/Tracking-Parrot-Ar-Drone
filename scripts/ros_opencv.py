# To convert the picture type ROS to OpenCV
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

# ROS IMAGE converted to OpenCV
def to_opencv(ros_image):
    try:  # 'bgr8' | desired_encoding = "passthrough"
        return bridge.imgmsg_to_cv2(ros_image, 'bgr8')
    except CvBridgeError as e:
        raise Exception("failure in conversion OpenCV image: {}".format(e))