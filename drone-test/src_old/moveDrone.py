import rospy
from geometry_msgs.msg import Twist

import time

class MyDrone:
    def __init__(self):

        self._is_running=True

        rospy.init_node("drone_node")
        rospy.loginfo("Drone has been initialized.")

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        
    
    def set_vel(self, vx=0, vz=0):
        vel_msg = Twist()
        vel_msg.linear.x = vx
        vel_msg.linear.y = 0
        vel_msg.linear.z = vz
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0

        self.vel_pub.publish(vel_msg)

if __name__ == "__main__":
    drone = MyDrone()
    drone.set_vel(vx=0.2)
    print("Setting vel")
    rospy.spin()