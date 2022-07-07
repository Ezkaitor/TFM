import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import yaml

rospy.init_node("image_publisher")

with open('test/image.yaml', 'r') as image_file:
    image = yaml.safe_load(image_file)

image_tp = Image()
image_tp.header = Header()
image_tp.data = image["data"]
image_tp.encoding = "rgb8"
image_tp.step = 800 * 3
image_tp.width = 800
image_tp.height = 800

cam_info_tp = CameraInfo()
cam_info_tp.header = Header()
cam_info_tp.width = 800
cam_info_tp.height = 800

cam_info_pub = rospy.Publisher("iris/camera/camera_info", CameraInfo, queue_size=1)
image_pub = rospy.Publisher("iris/camera/image_raw", Image, queue_size=1)

rate = rospy.Rate(3)

while not rospy.is_shutdown():
    image_pub.publish(image_tp)
    cam_info_pub.publish(cam_info_tp)
    rate.sleep()



