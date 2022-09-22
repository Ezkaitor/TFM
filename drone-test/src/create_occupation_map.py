#!/usr/bin/python3

import time

import rospy

import numpy as np
import cv2
import math
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image, CameraInfo
from mavros_msgs.msg import PositionTarget

from numpy_ros import to_numpy

from std_msgs.msg import Header

#from voronoi import generate_voronoi
from safe_space import create_safe_zone
from transformations import euler_from_quaternion


from PIL import Image

rospy.init_node("bounding_box")


VISUALIZE = True



def closest_from_boolmap(node, bool_map):
    nodes = np.vstack(np.where(bool_map)).transpose()
    deltas = nodes - node
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    closest_node = nodes[np.argmin(dist_2)]
    return closest_node, math.sqrt((closest_node[0]-node[0])**2+(closest_node[1]-node[1])**2)



class ImageOccupation:
    def __init__(self):
        self.image_size = [0,0]
        self.FOVx = 0
        self.FOVy = 0

        self.camera_info_tp = rospy.Subscriber('iris/camera_left/camera_info', CameraInfo, self.get_camera_info)
        self.camera_info_tp = rospy.Subscriber('iris/camera/camera_info', CameraInfo, self.get_camera_info)
        
        #self.bounding_boxes_tp = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.bounding_boxes_tp = rospy.Subscriber('mobilenet_ros/bounding_boxes', BoundingBoxes, self.boxes)
        self.goal_point_sub = rospy.Subscriber("mavros/setpoint_position/local", PoseStamped, self.goal_cb)
        self.current_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.get_current_pose)

        self.new_target_pub = rospy.Publisher('mavros/setpoint_position/temp', PoseStamped, queue_size=2)

        self.truth_map = {  "CamInfo": False,
                            "BBoxes": False,
                            "AllCameras": False,
                            "Current": False,
                            "Goal": False,
                            }
        self.truth_map2 = {
                            "Processed": True,
                            "Saved": True,
                            "Empty": False,
                            }
        
        self.new_goal = None
        self.no_detect = time.time()
        self.camera_maps = {}
        self.camera_angles = {}
        self.last_occupation_image = None
        self.detected = time.time()
        self.proc_count = -1
    
    def get_current_pose(self, msg):
        pose = msg.pose
        self.current_position = pose.position
        self.current_orientation = np.array(euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,  pose.orientation.w))
        self.current_orientation[2] -= math.pi
        if self.current_orientation[2] < -math.pi: self.current_orientation[2] = self.current_orientation[2] + 2*math.pi
        #self.current_orientation = [orientation*(180.0/math.pi) for orientation in self.current_orientation]
        self.truth_map["Current"] = True

    def goal_cb(self, goal):
        self.goal_point = goal.pose.position
        self.truth_map["Goal"] = True

    def get_camera_info(self, msg):
        #NOTE: Is assumed that all cameras are exactly identical
        self.image_size[0] = msg.width
        self.image_size[1] = msg.height

        self.FOVx = round(2*math.atan(msg.width/(2*msg.K[0])) * 180/math.pi)
        self.FOVy = round(2*math.atan(msg.height/(2*msg.K[4])) * 180/math.pi)

        self.pixel_per_degree = self.image_size[0]/self.FOVx

        self.truth_map["CamInfo"] = True

    def boxes(self, msg):
        
        if not self.truth_map2["Processed"]: return

        if not self.truth_map2["Saved"]:
            return
        self.truth_map2["Saved"] = False
        
        cam_name, cam_angle = msg.header.frame_id.split("_")
        self.camera_maps[cam_name] = np.zeros(self.image_size, dtype=bool)
        if int(cam_angle) > 180: self.camera_angles[cam_name] = int(cam_angle)-360
        else: self.camera_angles[cam_name] = int(cam_angle)

        ### Algorithm to create a occupation map ###
        
        # TODO: play with probabilities
        # TODO: different with classes
        for box in msg.bounding_boxes:
            try:
                self.camera_maps[cam_name][box.ymin:box.ymax, box.xmin:box.xmax] = True
            except IndexError:
                rospy.WARN("Bounding box out of the frame, check sizes.")
        
        # Check if all cameras are working, depending on de layout
        if not self.truth_map["AllCameras"]:
            if "single" in self.camera_maps.keys():
                self.truth_map["AllCameras"] = True
            elif "left" in self.camera_maps.keys() and "right" in self.camera_maps.keys():
                self.truth_map["AllCameras"] = True
            self.truth_map2["Saved"] = True
            return

        #if self.camera_maps["left"] == self.camera_maps["center"]: rospy.loginfo("Same image")
        #if self.camera_maps["right"] == self.camera_maps["center"]: rospy.loginfo("Same image")

        if self.truth_map["CamInfo"]:
            min_angle = min(self.camera_angles.values())-self.FOVx//2
            max_angle = max(self.camera_angles.values()) + self.FOVx//2
            self.occupation_image = np.zeros((self.image_size[1], round((max_angle-min_angle)*self.pixel_per_degree)), dtype=bool)

            for name, camera in self.camera_maps.items():
                min_value = int((self.camera_angles[name]-self.FOVx//2)*self.pixel_per_degree + self.occupation_image.shape[1]//2)
                self.occupation_image[:, min_value:min_value+self.image_size[0]] += camera

            #temp = np.copy(self.occupation_image)

            #if self.last_occupation_image is not None:
            #    self.occupation_image += self.last_occupation_image
            #self.last_occupation_image = temp
            #rospy.loginfo(f"Camera updated {time.time()}")
            self.truth_map["BBoxes"] = True
        self.truth_map2["Saved"] = True
    
    def send_empty(self):
        if not self.truth_map2["Empty"]: # and (time.time()-self.no_detect > 0.1):
            new_target = PoseStamped()
            new_target.header.frame_id = "none"
            self.new_target_pub.publish(new_target)
            #self.truth_map["Goal"] = False
            rospy.loginfo("Published Empty...")
            self.truth_map2["Empty"] = True
            self.no_detect = time.time()

            self.new_goal = None
    
    def show(self):
        if self.truth_map["BBoxes"] and VISUALIZE and all(flag for flag in self.truth_map.values()) and np.any(self.occupation_image):
            self.occupation_image_show = cv2.circle(self.occupation_image_show, (self.cam_goal_x, self.cam_goal_y), radius=5, color=(0, 0, 255), thickness=-1)
            self.occupation_image_show = cv2.resize(self.occupation_image_show, (1100, 400))
            cv2.imshow("Occupation", self.occupation_image_show)
            cv2.waitKey(1)

    def check(self):

        if self.truth_map["BBoxes"] and VISUALIZE:
            # Showing purposes
            self.occupation_image_show = (self.occupation_image*255).astype(np.uint8)
            self.occupation_image_show = cv2.cvtColor(self.occupation_image_show, cv2.COLOR_GRAY2BGR)
        #self.goal_point = Point(60, 50, 1.5)
        #self.truth_map["Goal"] = True
        #if False:
        self.truth_map2["Processed"] = False
        if all(flag for flag in self.truth_map.values()) and np.any(self.occupation_image): # TODO: set to false when goal reached
                                                            # All cameras is not necessary, just 1

            if self.new_goal is not None and self.new_goal==self.goal_point:
                self.goal_point = self.main_goal

            # Angle between two vectors
            module_yaw = math.atan2(self.goal_point.y-self.current_position.y, self.goal_point.x-self.current_position.x) 
            #module_pitch = math.atan2(self.goal_point.z-self.current_position.z, self.goal_point.x-self.current_position.x)
            # Angle between plane xy and vector
            dir_vector = to_numpy(self.goal_point) - to_numpy(self.current_position)
            xy_normal = np.array([0,0,1])
            module_pitch = math.asin(np.dot(dir_vector, xy_normal) / (math.sqrt(np.dot(dir_vector, dir_vector)) * math.sqrt(np.dot(xy_normal, xy_normal))))
            
            # Calculate goal coordinates in image
            self.cam_goal_x = int((self.current_orientation[2] - module_yaw) * (180.0/math.pi)*self.pixel_per_degree + self.occupation_image.shape[1]/2)
            self.cam_goal_y = int((self.current_orientation[1] - module_pitch) * (180.0/math.pi)*self.pixel_per_degree + self.occupation_image.shape[0]/2)

            if self.cam_goal_x >= self.occupation_image.shape[1] or self.cam_goal_x < 0 or self.cam_goal_y >= self.occupation_image.shape[0] or self.cam_goal_x < 0:
                self.truth_map2["Processed"] = True
                return False
            
            ## NEW ##
            # Angle between two vectors
            if self.new_goal is not None:
                module_yaw_new = math.atan2(self.new_goal.y-self.current_position.y, self.new_goal.x-self.current_position.x) 
                # Angle between plane xy and vector
                dir_vector_new = to_numpy(self.new_goal) - to_numpy(self.current_position)
                xy_normal = np.array([0,0,1])
                module_pitch_new = math.asin(np.dot(dir_vector_new, xy_normal) / (math.sqrt(np.dot(dir_vector_new, dir_vector_new)) * math.sqrt(np.dot(xy_normal, xy_normal))))

                # Calculate goal coordinates in image
                self.cam_goal_x_new = int((self.current_orientation[2] - module_yaw_new) * (180.0/math.pi)*self.pixel_per_degree + self.occupation_image.shape[1]/2)
                self.cam_goal_y_new = int((self.current_orientation[1] - module_pitch_new) * (180.0/math.pi)*self.pixel_per_degree + self.occupation_image.shape[0]/2)

            
            
            
            self.proc_count += 1
            #print(f"Pre process {self.proc_count}: {time.time()-self.detected}")
            self.truth_map2["Processed"] = True
            full, gradient = create_safe_zone(self.occupation_image, 150)
            

            #rospy.loginfo(f"Trying to {time.time()}")

            if full[self.cam_goal_y, self.cam_goal_x]:
                if self.new_goal is not None:
                    if self.cam_goal_x_new <= self.occupation_image.shape[1] and self.cam_goal_x_new > 0 and self.cam_goal_y_new <= self.occupation_image.shape[0] and self.cam_goal_x_new > 0:
                        if not full[self.cam_goal_y_new, self.cam_goal_x_new]:
                            return True
                
                self.main_goal = self.goal_point
                
                xy_module = 15#math.sqrt(np.dot(dir_vector[:2], dir_vector[:2]))

                min_height = 0.5
                
                delta_pitch_rev = math.atan(xy_module/(min_height-self.current_position.z))
                cam_cut = int(self.occupation_image.shape[0]/2 - delta_pitch_rev*self.pixel_per_degree)
                
                
                gradient[cam_cut:, :] = False # Remove part of the floor
                if not np.any(gradient):
                    
                    return True

                #print(f"Pre calculation {self.proc_count}: {time.time()-self.detected}")
                self.detected = time.time()
                closest_goal, dist = closest_from_boolmap([self.cam_goal_y, self.cam_goal_x], gradient)

                if self.new_goal is not None:
                    _, dist_new = closest_from_boolmap([self.cam_goal_y_new, self.cam_goal_x_new], gradient)
                    if dist_new < dist:
                        
                        return True

                

                delta_yaw = (closest_goal[1] - self.occupation_image.shape[1]/2)/self.pixel_per_degree
                delta_pitch = (self.occupation_image.shape[0]/2 - closest_goal[0])/self.pixel_per_degree
                
                new_goal_x = (xy_module/math.sin(math.radians(180-90-abs(delta_yaw))))*math.sin(math.radians(delta_yaw))
                if (delta_pitch < 10 and delta_pitch > 0) or (delta_pitch > -10 and delta_pitch < 0): new_goal_y = 0
                else:
                    new_goal_y = (xy_module/math.sin(math.radians(180-90-abs(delta_pitch))))*math.sin(math.radians(delta_pitch))
                
                #self.goal_point[0] = self.current_position[0] - 4
                #new_goal_x = (closest_goal[1] - self.image_size[1]/2) * (self.goal_point.x*math.sin(math.radians(self.FOVx/2))/math.sin(math.radians(270))) / (self.image_size[0]/2) + self.current_position.x
                #new_goal_y = (closest_goal[0] - self.image_size[1]/2) * (self.goal_point.x*math.sin(math.radians(self.FOVx/2))/math.sin(math.radians(90))) / (self.image_size[0]/2) + self.current_position[2]
                
                new_target = PoseStamped()
                new_target.header.frame_id = "temp_target"
                new_target.header.stamp = rospy.Time.now()
                
                if self.current_position.x - self.goal_point.x > 0: x_module = 5
                else: x_module = -5
                if self.current_position.y - self.goal_point.y > 0: x_module = 5
                else: y_module = -5
                new_target.pose.position.x = self.goal_point.x + new_goal_x*math.sin(module_yaw) #self.goal_point.x + new_goal_x*math.sin(module_yaw)
                new_target.pose.position.y = self.goal_point.y - new_goal_x*math.cos(module_yaw)#self.goal_point.y - new_goal_x*math.cos(module_yaw)
                new_target.pose.position.z = self.current_position.z + new_goal_y

                self.new_goal = new_target.pose.position

                print(f"Pre publish {self.proc_count}: {time.time()-self.detected}")
                self.detected = time.time()
                self.new_target_pub.publish(new_target)
                self.truth_map2["Empty"] = False

                rospy.loginfo("Publishing new target")
            
                
                if VISUALIZE:
                    self.occupation_image_show = cv2.circle(self.occupation_image_show, (closest_goal[1], closest_goal[0]), radius=5, color=(255, 0, 0), thickness=-1)
                    self.occupation_image_show[gradient] = (0,255,0)
                    self.occupation_image_show = cv2.circle(self.occupation_image_show, (self.cam_goal_x, self.cam_goal_y), radius=5, color=(0, 0, 255), thickness=-1)
                    self.occupation_image_show = cv2.resize(self.occupation_image_show, (1100, 400))
                    cv2.imshow("Occupation", self.occupation_image_show)
                    cv2.waitKey(1)

                    #cv2.imshow("Gradient", gradient)
                    
                
                
                self.no_detect = time.time()
                #self.occupation_image = np.zeros((self.occupation_image.shape), dtype=bool)
                #self.truth_map["AllCameras"] = False
                
                return True
        
        if self.truth_map["BBoxes"] and VISUALIZE: 
            self.occupation_image_show = (self.occupation_image*255).astype(np.uint8)
            self.occupation_image_show = cv2.resize(self.occupation_image_show, (1100, 400))
            cv2.imshow("Occupation", self.occupation_image_show)
            cv2.waitKey(1)
            #self.occupation_image = np.zeros((self.occupation_image.shape), dtype=bool)
            self.truth_map2["Processed"] = True
        
        
        return False

        ###########################################


def main():
    
    rate = rospy.Rate(20)
    img_occ = ImageOccupation()
    while not rospy.is_shutdown():
        success = img_occ.check()
        img_occ.truth_map2["Processed"] = True
        img_occ.show()
        if not success: img_occ.send_empty()
        rate.sleep()

if __name__ == "__main__":
    main()