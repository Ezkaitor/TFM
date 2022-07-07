import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

rospy.init_node("offbrd",anonymous=True)
rate=rospy.Rate(2)
setpoint_pub=rospy.Publisher("/mavros/setpoint_position/local",PoseStamped,queue_size=10)
arming_s=rospy.ServiceProxy("/mavros/cmd/arming",CommandBool)
set_mode=rospy.ServiceProxy("/mavros/set_mode",SetMode)
setpt=PoseStamped()
setpt.header.stamp = rospy.Time.now()
setpt.pose.position.z=5
for i in range (0,10):
    setpoint_pub.publish(setpt)
    rate.sleep()
set_mode(0,"OFFBOARD")
arming_s(True)

while (rospy.is_shutdown()==False):
    setpoint_pub.publish(setpt)
    rate.sleep()