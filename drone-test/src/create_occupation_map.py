#!/usr/bin/python3

import rospy

import numpy as np
import cv2
import math
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from mavros_msgs.msg import PositionTarget

from std_msgs.msg import Header

#from voronoi import generate_voronoi
from safe_space import create_safe_zone

rospy.init_node("bounding_box")

FOV = 80


def closest_from_boolmap(node, bool_map):
    nodes = np.vstack(np.where(bool_map)).transpose()
    deltas = nodes - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return nodes[np.argmin(dist_2)]

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

class ImageOccupation:
    def __init__(self):
        self.image_size = [0,0]

        self.camera_info_tp = rospy.Subscriber('iris/camera/camera_info', CameraInfo, self.get_camera_info)
        #self.front_camera_tp = rospy.Subscriber('iris/camera/image_raw', Image, self.get_image)
        #self.bounding_boxes_tp = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.bounding_boxes_tp = rospy.Subscriber('mobilenet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.goal_point_sub = rospy.Subscriber("mavros/setpoint_raw/local", PositionTarget, self.goal_cb)
        self.current_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.get_current_pose)

        self.occupation_map_tp = rospy.Publisher('iris/occupation_map', Image, queue_size=1)

        self.truth_map = {  "CamInfo": False,
                            "BBoxes": False,
                            "AllCameras": False,
                            "Current": False,
                            "Goal": False,
                            }
        
        self.cameras = {}
    
    def get_current_pose(self, msg):
        pose = msg.pose
        self.current_position = (pose.position.x, pose.position.y, pose.position.z)
        self.current_orientation = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,  pose.orientation.w)
        self.truth_map["Current"] = True

    def goal_cb(self, goal):
        self.goal_point = [goal.position.x, goal.position.y, goal.position.z]
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

        if self.truth_map["CamInfo"] and self.truth_map["AllCameras"]:
            
            ### Algorithm to create a occupation map ###
            self.cameras[msg.header.frame_id] = np.zeros(self.image_size, dtype=bool)
            # TODO: play with probabilities
            # TODO: different with classes
            for box in msg.bounding_boxes:
                try:
                    self.cameras[msg.header.frame_id][box.ymin:box.ymax, box.xmin:box.xmax] = True
                except IndexError:
                    rospy.WARN("Bounding box out of the frame, check sizes.")
            
            truth_image = Image()
            truth_image.header = Header()
            truth_image.width = self.image_size[0]
            truth_image.height = self.image_size[1]
            truth_image.encoding = "mono8"
            truth_image.data = self.cameras[msg.header.frame_id].tobytes() # TODO: Make a bigger image combined
            
            occupation_image = (self.cameras[msg.header.frame_id] * 255).astype(np.uint8)
            
            if self.truth_map["Goal"] and self.truth_map["Current"]: # TODO: set to false when goal reached
                
                # HACK: set borders as obstacles for a better voronoi
                # TODO: check what happenswith multiple cameras
                #occupation_image[0,:] = 255
                #occupation_image[-1,:] = 255
                #occupation_image[:, 0] = 255
                #occupation_image[:, -1] = 255

                full, gradient = create_safe_zone(occupation_image, 50)
                #gradient = (gradient * 255).astype(np.uint8)

                # Goal point in image
                goal_x = (self.image_size[0]/2)*(self.goal_point[1]-self.current_position[0])/(self.goal_point[0]*math.sin(math.radians(FOV/2))/math.sin(math.radians(90))) # NOTE: now it is x, change to focusing drone
                goal_y = (self.image_size[1]/2)*(self.goal_point[2]-self.current_position[2])/(self.goal_point[0]*math.sin(math.radians(FOV/2))/math.sin(math.radians(90))) # NOTE: now it is x, change to focusing drone

                goal_x = int(goal_x+self.image_size[0]/2)
                goal_y = int(goal_y+self.image_size[1]/2)

                if full[goal_y,goal_x]:
                    closest_goal = closest_from_boolmap([goal_y, goal_x], gradient)

                # Check if goal in sight line
                if goal_x > self.image_size[0]:
                    pass # Goal out of sight in x axis
                if goal_y > self.image_size[1]:
                    pass # Goal out of sight in y axis
                
                # Showing purposes
                occupation_image = cv2.cvtColor(occupation_image, cv2.COLOR_GRAY2BGR)
                occupation_image = cv2.circle(occupation_image, (goal_x, goal_y), radius=5, color=(0, 0, 255), thickness=-1)
                occupation_image = cv2.circle(occupation_image, (closest_goal[1], closest_goal[0]), radius=5, color=(255, 0, 0), thickness=-1)
                occupation_image[gradient] = (0,255,0)

                #cv2.imshow("Gradient", gradient)
            cv2.imshow("Occupation", occupation_image)
            cv2.waitKey(0)
            #self.occupation_map_tp.publish(truth_image)
            ###########################################

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