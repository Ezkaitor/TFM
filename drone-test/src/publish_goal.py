
import rospy
from mavros_msgs.msg import PositionTarget


rospy.init_node("goal_publisher")

goal_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=1)

raw_msg = PositionTarget()

mask = 4088
raw_msg.header.frame_id = "home"
raw_msg.header.stamp = rospy.Time.now()
raw_msg.coordinate_frame = 1
raw_msg.type_mask  = mask
raw_msg.position.x = -5
raw_msg.position.y = 0
raw_msg.position.z = 2

rate = rospy.Rate(2)
while not rospy.is_shutdown():
    rospy.loginfo("Publishing target...")
    goal_pub.publish(raw_msg)
    rate.sleep()


