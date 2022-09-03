#!/usr/bin/python3
from multiprocessing.sharedctypes import Value

from requests import check_compatibility
import rospy
from std_msgs.msg import String

import time

print("""
##########################################\n
######## Welcome to the drone api #######\n
##########################################
""")


available = True
drone_comm = False
check_alive = time.time()
responses = {
    "takeoff": "Drone has takeoff.",
    "land": "Drone has succesfully landed.",
    "base": "Drone succesfully returned to base.",
    "avoidance": "Avoidance successfully changed."
}
def get_response(resp):
    global available
    global drone_comm
    global check_alive
    if resp.data == "Is Alive":
        drone_comm = True
        check_alive = time.time()
        return
    msg, state = resp.data.split(" ")
    if bool(state) == True:
        print(responses[msg])
    else:
        print(f"State of the drone is not optimal for {msg}.")
    available = True

rospy.init_node("drone_api_node")
msg_pub = rospy.Publisher("api/message", String, queue_size=1)
state_sub = rospy.Subscriber("api/response", String, get_response)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if time.time() - check_alive > 5:
        drone_comm = False
        available = True
    if not drone_comm:
        print("Drone communication is not active.")
        while not drone_comm:
            pass
        print("Drone communication has been stablished!")
    if available:
        print(">> ", end='')
        msg = input()
        if "takeoff" in msg:
            msg_splitted = msg.split(" ")
            if len(msg_splitted) < 2:
                print(f"Provided altitude {msg_splitted[1]} is not valid.")
                continue
            try:
                int(msg_splitted[1])
            except ValueError:
                print(f"Provided altitude {msg_splitted[1]} is not an integer.")
                continue
            
            print("Preparing for takeoff...")
            
            # Check if drone is available for takeoff
            pass
        elif msg == "land":
            print("Preparing to land...")
            # check
            pass
        elif msg == "base":
            print("Returning to base...")
            # check
            pass
        elif "reach" in msg:
            # TODO: think which possibilities could be
            msg_splitted = msg.split(" ")
            if len(msg_splitted) < 2:
                print(f"Provided altitude {msg_splitted[1]} is not valid.")
                continue
            point = [1,2,3] # TODO: stablish a split
            for coord in point:
                try:
                    int(coord)
                except ValueError:
                    print(f"Provided coord {coord} is not an integer.")
        elif "avoidance" in msg:
            # check
            pass
        elif msg == "exit": break

        elif not msg:
            continue
        else:
            print("WARNING! Provided command has not been found.")
            print("Available commands: takeoff, land, base, reach, avoidance, exit")
        
        msg_pub.publish(msg)
        available = False
    rate.sleep()
print("Exited Successfully!")
print("Come back early :)")