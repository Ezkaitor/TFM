from argparse import ArgumentParser, SUPPRESS
from multiprocessing.sharedctypes import Value

from subprocess import Popen
from tkinter.messagebox import NO
from drone import Drone

import roslaunch

# TODO: add an execute routine to these modules
#import mobilenet_ros
#import yolo_ros
#import bounding_box
#import pub_recalculate

import rospy
from std_msgs.msg import String

from pymavlink import mavutil

process_recalculate = None
process_bounding_box = None
process_algorithm = None

msg = ""

def build_argparser():
    parser = ArgumentParser(add_help=False)
    args = parser.add_argument_group('Options')
    args.add_argument('-h', '--help', action='help', default=SUPPRESS, help='Show this help message and exit.')
    args.add_argument("-a", "--avoidance", default=True, help="Optional. Use avoidance.")
    args.add_argument("-al", "--algorithm", default="mobilenet", choices=["yolo", "mobilenet"], help="Optional. Selects the algorithm for avoidance.")
    
    return parser.parse_args()

def get_msg(message):
    global msg
    msg = message.data

def launch_avoidance(algorithm='mobilenet'):
    global process_algorithm
    global process_recalculate
    global process_bounding_box

    rosrun_args = ["rosrun", "drone-test"]
    
    process_recalculate = Popen(args=rosrun_args + ["pub_recalculate.py"])
    process_bounding_box = Popen(args=rosrun_args + ["bounding_box.py"])

    if algorithm == "mobilenet":
        process_algorithm = Popen(args=rosrun_args + ["mobilenet_ros.py"])

rospy.init_node("drone_instance")
msg_sub = rospy.Subscriber("api/message", String, get_msg)
resp_pub = rospy.Publisher("api/response", String, queue_size=1)

options = build_argparser()

avoidance = options.avoidance
avoidance = False
if avoidance:
    launch_avoidance(options.algorithm)
    

drone = Drone()

debug = False
if not debug:
    drone.setUp()
    # make sure the simulation is ready to start the mission
    #drone.wait_for_topics(60)

    #drone.wait_for_mav_type(10)
    #drone.tearDown()


rate = rospy.Rate(20)
msg = ""
success = None
offboard = False
while not rospy.is_shutdown():
    
    ##if not msg: continue

    if "takeoff" in msg:
        msg, altitude = msg.split(" ")
        success = drone.takeoff(altitude=int(altitude), timeout=15)
        
    elif msg == "land":
        print("Preparing to land...")
        success = drone.land(timeout=15)
        # check
        pass
    elif msg == "base":
        print("Returning to base...")
        success = drone.return_to_base(timeout=15)
        pass
    elif "reach" in msg:
        if not offboard:
            drone.set_mode_srv(0, "OFFBOARD")
            offboard = True
        # TODO: think which possibilities could be
        msg_splitted = msg.split(" ")
        if len(msg_splitted) < 2:
            print(f"Provided altitude {msg_splitted[1]} is not valid.")
            continue
        point_str = msg_splitted[1:] # TODO: stablish a split
        point = [0,0,0]
        for idx, coord in enumerate(point_str):
            try:
                point[idx] = float(coord)
            except ValueError:
                print(f"Provided coord {coord} is not an float.")

        success = drone.reach_local_position(point)
    elif "avoidance" in msg:
        # check
        msg_splitted = msg.split(" ")
        if msg_splitted[1].lower() in ["true", "1"]: avoidance = True
        elif msg_splitted[1].lower() in ["false", "0"]: avoidance = False
        else:
            raise ValueError("Value not valid. Try True or False")
        if avoidance:
            launch_avoidance()
        else:
            if process_algorithm is not None: process_algorithm.terminate()
            if process_bounding_box is not None: process_bounding_box.terminate()
            if process_recalculate is not None: process_recalculate.terminate()
        
        drone.set_avoidance(bool(msg_splitted[1]))
        success = True

    if success is not None:
        resp = msg.split(" ")[0]
        resp_pub.publish(f"{resp} {success}")
        rospy.loginfo(f"Responding: {resp} {success}")
        success = None
        msg = ""

    resp_pub.publish("Is Alive")

    rate.sleep()


