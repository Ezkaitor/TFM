import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import time
from pathlib import Path


rospy.init_node("trajectories")


drone_pos_points = []
drone_altitude = []
start_time = time.time()
reached = True

save_path = Path(__file__).parent / "trajectories"

def get_current_pose(position):
    global drone_altitude
    global drone_pos_points
    global start_time

    if not reached:
        drone_pos_points.append((position.pose.position.x, position.pose.position.y))
        drone_altitude.append((round(time.time()-start_time, 2),position.pose.position.z))

def reset_positions(goal):
    global reached
    global drone_altitude
    global drone_pos_points
    global start_time
    if reached:
        drone_altitude = []
        drone_pos_points = []
        start_time = time.time()

def create_map(msg):
    global reached
    global drone_altitude
    global drone_pos_points
    reached = msg.data
    if reached:
        x = [pos[0] for pos in drone_pos_points]
        y = [pos[1] for pos in drone_pos_points]
        z = [alt[1] for alt in drone_altitude]
        timestamp = [alt[0] for alt in drone_altitude]

        fig, (ax1, ax2) = plt.subplots(nrows=2)

        ax1.plot(x,y,color='green')
        ax1.set_title("Desplazamiento en el plano XY")
        ax1.set_xlabel("X")
        ax1.set_ylabel("Y")
        ax1.set_xlim((min(x+y)-1, max(x+y)+1))
        ax1.set_ylim((min(x+y)-1, max(x+y)+1))

        ax2.plot(timestamp,z,color='blue')
        ax2.set_title("Altura con respecto del tiempo")
        ax2.set_xlabel("Tiempo")
        ax2.set_ylabel("Altura")

        plt.tight_layout()

        plt.savefig(save_path/"map.png")

        plt.close()

        rospy.loginfo("Map Generated")
    


goal_point_sub = rospy.Subscriber("mavros/setpoint_position/local", PoseStamped, reset_positions)
current_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, get_current_pose)
position_reached_sub = rospy.Subscriber("mavros/setpoint_position/reached", Bool, create_map)


while not rospy.is_shutdown():
    rospy.spin()