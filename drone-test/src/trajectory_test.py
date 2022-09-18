import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String

from sys import argv

import time
from pathlib import Path


rospy.init_node("trajectories")


drone_pos_points = []
drone_altitude = []
start_time = time.time()
reached = True
available = True

test = argv[1]

save_path = Path(__file__).parent / "trajectories"

obstacle_props = {"facecolor":"grey", "fill":True}
obstacles = [
    Rectangle((18, 11.8), 1, 6, facecolor="grey", fill=True),
    Rectangle((13.5, -2.3), 1, 4, facecolor="grey", fill=True),
    Rectangle((19.5, -7), 1, 4, facecolor="grey", fill=True),
    Rectangle((26.5, -1.5), 1, 6, facecolor="grey", fill=True),
    Rectangle((44, 23.5), 4, 1, facecolor="grey", fill=True),
    Rectangle((35.8, 30.5), 4, 1, facecolor="grey", fill=True),
    Rectangle((45, 39), 4, 1, facecolor="grey", fill=True),
]

def get_response(resp):
    global available
    
    #msg, state = resp.data.split(" ")
    if resp.data == "Is Alive": return
    available = True

def get_current_pose(position):
    global drone_altitude
    global drone_pos_points
    global start_time

    if not available:
        drone_pos_points.append((position.pose.position.x, position.pose.position.y))
        drone_altitude.append((round(time.time()-start_time, 2),position.pose.position.z))
        

def create_map():
    global drone_altitude
    global drone_pos_points
    global obstacles
    global test

    #test = 2
    size = [800, 800]
    fov = 80
    speed = 2
    cameras = 1
    angles = [0]

    x = [pos[0] for pos in drone_pos_points]
    y = [pos[1] for pos in drone_pos_points]
    z = [alt[1] for alt in drone_altitude]
    timestamp = [alt[0] for alt in drone_altitude]

    fig1, ax1 = plt.subplots(nrows=1)
    fig2, ax2 = plt.subplots(nrows=1)

    ax1.scatter(x[0], y[0], color="red")
    ax1.scatter(x[-1], y[-1], color="blue")
    ax1.plot(x,y,color='green')
    ax1.set_title("Desplazamiento en el plano XY")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_xlim((-6, 58))
    ax1.set_ylim((-12, 55))

    for rectangle in obstacles:
        ax1.add_patch(rectangle)

    ax2.plot(timestamp,z,color='blue')
    ax2.set_title("Altura con respecto del tiempo")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylabel("Altura")

    plt.tight_layout()

    name = f"{test}-{size[0]}x{size[1]}_cams{cameras}"
    for angle in angles: name += "_" + str(angle)
    name += "_fov_" + str(fov)
    
    fig1.savefig(save_path/(name+"_map.png"))
    fig2.savefig(save_path/(name+"_alt.png"))

    plt.close()

    rospy.loginfo("Map Generated")


current_position_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, get_current_pose)


msg_pub = rospy.Publisher("api/message", String, queue_size=1)
state_sub = rospy.Subscriber("api/response", String, get_response)

rate = rospy.Rate(2)
count = 0
point_count = 0
while not rospy.is_shutdown():
    
    points = [(48, 0, 1.5), (45,45,1.5)]

    if available and count > 1:
        #try:
        if point_count >= 2: break
        point = points[point_count]
        #except:
        #    break
        msg = f"reach {point[0]} {point[1]} {point[2]}"
        msg_pub.publish(msg)
        available = False
        point_count += 1
    
    #msg_pub.publish(msg + str(count))
    count += 1
    #rospy.loginfo("Running...")
    rate.sleep()

create_map()
msg = f"reach 0 0 1.5"
msg_pub.publish(msg)