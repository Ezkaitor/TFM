#from re import A
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from mavros import mavlink
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, \
                            WaypointList, Mavlink, Waypoint, WaypointReached, PositionTarget
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu
from six.moves import xrange
from threading import Thread


class Drone():

    def __init__(self, *args):
        super(Drone, self).__init__(*args)

    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.imu_data = Imu()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state', 'imu'
            ]
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/param/set', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            raise ValueError("failed to connect to services")
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_param_srv = rospy.ServiceProxy('mavros/param/set', ParamSet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.imu_data_sub = rospy.Subscriber('mavros/imu/data',
                                               Imu,
                                               self.imu_data_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)

        self.mission_item_reached = -1  # first mission item is 0
        #self.mission_name = ""

        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        self.global_target = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        

        # need to simulate heartbeat to prevent datalink loss detection
        self.hb_mav_msg = mavutil.mavlink.MAVLink_heartbeat_message(
            mavutil.mavlink.MAV_TYPE_GCS, 0, 0, 0, 0, 0)
        self.hb_mav_msg.pack(mavutil.mavlink.MAVLink('', 2, 1))
        self.hb_ros_msg = mavlink.convert_to_rosmsg(self.hb_mav_msg)
        self.hb_thread = Thread(target=self.send_heartbeat, args=())
        self.hb_thread.daemon = True
        self.hb_thread.start()
    
    def send_heartbeat(self):
        rate = rospy.Rate(2)  # Hz
        while not rospy.is_shutdown():
            self.mavlink_pub.publish(self.hb_ros_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def tearDown(self):
        self.log_topic_vars()

    #
    # Callback functions
    #
    
    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def imu_data_callback(self, data):
        self.imu_data = data

        if not self.sub_topics_ready['imu']:
            self.sub_topics_ready['imu'] = True

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)


    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)


    def set_param(self, param_id, param_value, timeout):
        """param: PX4 param string, ParamValue, timeout(int): seconds"""
        if param_value.integer != 0:
            value = param_value.integer
        else:
            value = param_value.real
        rospy.loginfo("setting PX4 parameter: {0} with value {1}".
        format(param_id, value))
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        param_set = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.set_param_srv(param_id, param_value)
                if res.success:
                    rospy.loginfo("param {0} set to {1} | seconds: {2} of {3}".
                    format(param_id, value, i / loop_freq, timeout))
                break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)


    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def wait_for_takeoff_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for takeoff state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)    

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")

    def reach_position(self, position):
        long, lat, alt = position
        #gb_pos = GlobalPositionTarget(longitude=long, latitude= lat, altitude=alt)
        p = Point(x=long, y=lat, z=alt)
        raw_msg = PositionTarget()

        mask = 4088
        raw_msg.header.frame_id = "home"
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.coordinate_frame = 1
        raw_msg.type_mask  = mask
        raw_msg.position.x = long
        raw_msg.position.y = lat
        raw_msg.position.z = alt
        self.point = raw_msg
        #raw_msg.velocity.x = vx
        #raw_msg.velocity.y = vy
        #raw_msg.yaw        = yaw
        rate = rospy.Rate(2)  # Hz
        #while not rospy.is_shutdown():
        for i in range(10):
            self.global_target.publish(raw_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    drone = Drone()
    drone.setUp()
    rospy.spin()