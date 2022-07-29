import rospy

import numpy as np
import cv2
#from cv_bridge import CvBridge

from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo

from std_msgs.msg import Header


class ImageOccupation:
    def __init__(self):
        self.image_size = [0,0]

        self.camera_info_tp = rospy.Subscriber('iris/camera/camera_info', CameraInfo, self.get_camera_info)
        self.front_camera_tp = rospy.Subscriber('iris/camera/image_raw', Image, self.get_image)
        #self.bounding_boxes_tp = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.bounding_boxes_tp = rospy.Subscriber('mobilenet_ros/bounding_boxes', BoundingBoxes, self.boxes)

        self.occupation_map_tp = rospy.Publisher('iris/occupation_map', Image, queue_size=1)

        self.truth_map = {  "CamInfo": False,
                            "BBoxes": False,
                            }

    def get_camera_info(self, msg):
        self.image_size[0] = msg.width
        self.image_size[1] = msg.height
        self.truth_map["CamInfo"] = True

    def boxes(self, msg):
        if self.truth_map["CamInfo"]:
            self.occupation_map = np.zeros(self.image_size, dtype=bool)
            # TODO: play with probabilities
            # TODO: different with classes
            for box in msg.bounding_boxes:
                try:
                    self.occupation_map[box.ymin:box.ymax, box.xmin:box.xmax] = True
                except IndexError:
                    rospy.WARN("Bounding box out of the frame, check sizes.")
            
            truth_image = Image()
            truth_image.header = Header()
            truth_image.width = self.image_size[0]
            truth_image.height = self.image_size[1]
            truth_image.encoding = "mono8"
            truth_image.data = self.occupation_map.tobytes()
            self.occupation_map_tp.publish(truth_image)

        self.truth_map["BBoxes"] = True
        #print("checkpoint")

    def get_image(self, msg):
        if self.truth_map["CamInfo"] and self.truth_map["BBoxes"]:
            
            rgb_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:,:,::-1]
            combined = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
            
            combined[self.occupation_map] = 255 # NOTE: uint8 image
            #cv2.imshow("image", combined)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()


rospy.init_node("bounding_box")

img_occ = ImageOccupation()

rospy.spin()