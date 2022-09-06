#!/usr/bin/python3

import rospy

import numpy as np
import cv2
import math
#from cv_bridge import CvBridge

from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo
from mavros_msgs.msg import PositionTarget

from std_msgs.msg import Header

rospy.init_node("bounding_box")

def get_minimum_distance(point, box) -> float:
    
    # Smallest distance to one of the edges INSIDE
    if point.x < box.xmax and point.x > box.xmin and point.y < box.ymax and point.y > box.ymin:
        if abs(point.x - box.xmax) < abs(point.x - box.xmin) and abs(point.x - box.xmax) < abs(point.y - box.ymin) and abs(point.x - box.xmax) < abs(point.y - box.ymax):
            return point.x - box.xmax
        elif abs(point.x - box.xmax) > abs(point.x - box.xmin) and abs(point.x - box.xmax) < abs(point.y - box.ymin) and abs(point.x - box.xmax) < abs(point.y - box.ymax):
            return box.xmin - point.x
        elif abs(point.y - box.ymax) < abs(point.y - box.ymin) and abs(point.y - box.ymax) < abs(point.x - box.xmin) and abs(point.y - box.ymax) < abs(point.x - box.xmax):
            return point.y - box.ymax
        elif abs(point.y - box.ymax) > abs(point.y - box.ymin) and abs(point.y - box.ymax) < abs(point.x - box.xmin) and abs(point.y - box.ymax) < abs(point.x - box.xmax):
            return box.ymin - point.y
        
    # Smallest distance to one of the corners
    elif point.x > box.xmax and point.y > box.ymax: # Top-right corner
        return math.sqrt((point.x - box.xmax)**2 + (point.y - box.ymax)**2)
    elif point.x > box.xmax and point.y < box.ymin: # Bottom-right corner
        return math.sqrt((point.x - box.xmax)**2 + (point.y - box.ymin)**2)
    elif point.x < box.xmin and point.y > box.ymax: # Top-left corner
        return math.sqrt((point.x - box.xmin)**2 + (point.y - box.ymax)**2)
    elif point.x < box.xmin and point.y < box.ymin: # Bottom-left corner
        return math.sqrt((point.x - box.xmin)**2 + (point.y - box.ymin)**2)
    
    # Smallest distance to one of the edges OUTSIDE
    elif point.x > box.xmax and point.y < box.ymax and point.y > box.ymin: # Right edge
        return point.x - box.xmax
    elif point.x < box.xmax and point.x > box.xmin and point.y > box.ymax: # Top edge
        return point.y - box.ymax
    elif point.x < box.xmax and point.x > box.xmin and point.y < box.ymin: # Bottom edge
        return box.ymin - point.y
    elif point.x < box.xmin and point.y < box.ymax and point.y > box.ymin: # Left edge
        return box.xmin - point.x


class ImageOccupation:
    def __init__(self):
        self.image_size = [0,0]

        self.camera_info_tp = rospy.Subscriber('iris/camera/camera_info', CameraInfo, self.get_camera_info)
        #self.front_camera_tp = rospy.Subscriber('iris/camera/image_raw', Image, self.get_image)
        #self.bounding_boxes_tp = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.bounding_boxes_tp = rospy.Subscriber('mobilenet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.goal_point_sub = rospy.Subscriber("mavros/setpoint_raw/local", PositionTarget, self.goal_cb)

        self.occupation_map_tp = rospy.Publisher('iris/occupation_map', Image, queue_size=1)

        self.truth_map = {  "CamInfo": False,
                            "BBoxes": False,
                            "AllCameras": False,
                            "Goal": False
                            }
        
        self.cameras = {}
        self.set_goal = False

    def goal_cb(self, goal):
        self.goal_point = [goal.position.x, goal.position.y, goal.position.x]
        self.truth_map["Goal"] = True

    def get_camera_info(self, msg):
        self.image_size[0] = msg.width
        self.image_size[1] = msg.height
        self.truth_map["CamInfo"] = True

    
    def boxes(self, msg):
        
        # Check if all cameras are working, depending on de layout
        if not self.truth_map["AllCameras"]:
            self.cameras[msg.header.frame_id] = None
            if "single" in self.cameras.keys():
                self.truth_map["AllCameras"] = True
            elif "left" in self.cameras.keys() and "right" in self.cameras.keys():
                self.truth_map["AllCameras"] = True

        if self.truth_map["CamInfo"] and self.truth_map["AllCameras"] and self.truth_map["Goal"]:
            
            ### Algorithm avoid bounding boxes ###
            centres = []
            distances = []
            for box in msg.bounding_boxes:
                #centres.append((box.ymax - box.ymin, box.xmax - box.xmin))
                distances.append(get_minimum_distance(self.goal_point, box))
            ######################################

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


def main():
    

    img_occ = ImageOccupation()

    rospy.spin()

if __name__ == "__main__":
    main()