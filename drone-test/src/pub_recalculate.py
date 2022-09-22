#!/usr/bin/python3

from sympy import Point
import rospy
import numpy as np

from std_msgs.msg import Bool
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import Image

goal_point = None
set_goal = False
def generate_skeleton(occupancy_map: np.ndarray) -> np.ndarray:
    skeleton = None
    return skeleton

def goal_cb(goal):
    global goal_point
    global set_goal
    goal_point = [goal.position.x, goal.position.y, goal.position.x]
    set_goal = True

def occupation_cb(image):
    occupation = np.frombuffer(image.data, dtype=bool).reshape((800,800))

    
    if not occupation.any(): return

    if set_goal:
        skeleton = generate_skeleton(occupation)
        # TODO: check the closest point to skeleton
        camera_centre = 400
        obstacle_first_line = np.where(occupation==True)[0][0]
        obstacle_centre = (np.where(occupation[obstacle_first_line])[0][-1] + np.where(occupation[obstacle_first_line])[0][0])//2
        if abs(obstacle_centre-camera_centre) < 50: 
            publish_recalculate()

def publish_recalculate():
    rospy.loginfo("Publishing recalculate...")

    loop_freq = 1
    timeout= 3
    rate = rospy.Rate(loop_freq)

    for i in range(timeout*loop_freq):
        recalculate_pub.publish(True)
        rate.sleep()

rospy.init_node("recalculate_node")

occupation_tp = rospy.Subscriber("iris/occupation_map", Image, occupation_cb)
goal_point_sub = rospy.Subscriber("mavros/setpoint_raw/local", PositionTarget, goal_cb)
recalculate_pub = rospy.Publisher("mavros/recalculate", Bool, queue_size=1)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
    rate.sleep()



