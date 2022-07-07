import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode



current_state = State

def state_cb(msg):
    current_state = msg

def main(*argc, **argv):
    rospy.init_node("offb_node")

    rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)

    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    for i in range(100, 0, -1):
        local_pos_pub.publish(pose)
        rate.sleep()
    
    offb_set_mode = SetMode
    offb_set_mode._request_class.custom_mode = "OFFBOARD"

    arm_cmd = CommandBool
    arm_cmd._request_class.value = True

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and rospy.Time.now() - last_request > rospy.Duration(5.0):
        #    if set_mode_client.call(offb_set_mode) and offb_set_mode._response_class.mode_sent:
            rospy.loginfo("Offboard enabled")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and rospy.Time.now() - last_request > rospy.Duration(5.0):
                if arming_client.call(arm_cmd) and arm_cmd._response_class.success:
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
    
    return 

if __name__ == "__main__":
    main()
    rospy.spin()