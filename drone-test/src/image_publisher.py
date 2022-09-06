from PIL import Image
from pathlib import Path

import rospy
from sensor_msgs.msg import Image

rate = rospy.Rate(2)

image_dict = {
    "single": True,
    "left": False,
    "center": False,
    "right": False,
}

rospy.init_node("image_publisher")

image_path = Path.home() / "repos/TFM/drone-test/images"
if image_dict["single"]:
    single_image = Image.open(image_path/"single.png")
    single_pub = rospy.Publisher("iris/camera/image_raw", Image, queue_size=1)
if image_dict["center"]:
    center_image = Image.open(image_path/"center.png")
    center_pub = rospy.Publisher("iris/camera_center/image_raw", Image, queue_size=1)
if image_dict["left"]:
    left_pub = rospy.Publisher("iris/camera_left/image_raw", Image, queue_size=1)
    if not image_dict["center"]:
        left_image = Image.open(image_path/"left_30.png")
    elif image_dict["center"]:
        left_image = Image.open(image_path/"left_45.png")
if image_dict["right"]:
    right_pub = rospy.Publisher("iris/camera_right/image_raw", Image, queue_size=1)
    if not image_dict["center"]:
        right_image = Image.open(image_path/"right_30.png")
    elif image_dict["center"]:
        right_image = Image.open(image_path/"right_45.png")

while not rospy.is_shutdown():

    if image_dict["single"]:
        single_image = Image.open(image_path/"single.png")
        single_pub = rospy.Publisher("iris/camera/image_raw", Image, queue_size=1)
    if image_dict["center"]:
        center_image = Image.open(image_path/"center.png")
        center_pub = rospy.Publisher("iris/camera_center/image_raw", Image, queue_size=1)
    if image_dict["left"]:
        left_pub = rospy.Publisher("iris/camera_left/image_raw", Image, queue_size=1)
        if not image_dict["center"]:
            left_image = Image.open(image_path/"left_30.png")
        elif image_dict["center"]:
            left_image = Image.open(image_path/"left_45.png")
    if image_dict["right"]:
        right_pub = rospy.Publisher("iris/camera_right/image_raw", Image, queue_size=1)
        if not image_dict["center"]:
            right_image = Image.open(image_path/"right_30.png")
        elif image_dict["center"]:
            right_image = Image.open(image_path/"right_45.png")
    rate.sleep()