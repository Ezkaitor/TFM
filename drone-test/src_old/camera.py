import rospy
from sensor_msgs.msg import Image, CameraInfo

from cv_bridge import CvBridge
import cv2 as cv
import numpy as np



def get_cam(msg):
    print("entering camera visualization")
    bridge = CvBridge()
    rgb_img = bridge.imgmsg_to_cv2(msg)
    thr_img = cv.cvtColor(rgb_img, cv.COLOR_BGR2GRAY)
    _, thr_img = cv.threshold(rgb_img, 100, 255, cv.THRESH_BINARY)
    cv.imshow("image", rgb_img)
    cv.imshow("image_bw", thr_img)
    cv.waitKey(0)
    cv.destroyAllWindows()

def main():
    print("Test has started")

    rospy.init_node("camera")
    rgb_sub = rospy.Subscriber("/iris/camera/image_raw", Image, get_cam)

    rospy.spin()

if __name__ == "__main__":
    main()
