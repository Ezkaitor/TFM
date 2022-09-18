#from re import A
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Point
from mavros import mavlink
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList, Mavlink, GlobalPositionTarget, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, \
                            WaypointPush
from pymavlink import mavutil
#from sensor_msgs.msg import NavSatFix, Imu
from threading import Thread

from transformations import quaternion_from_euler


class Drone():

    def __init__(self, *args):
        super(Drone, self).__init__(*args)
        self.flying = False
        self.temp_target = None

    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.global_position = PoseStamped()
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
            rospy.wait_for_service('mavros/cmd/land', service_timeout)
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
        self.set_land_srv = rospy.ServiceProxy('mavros/cmd/land',
                                                 CommandTOL)
        self.set_takeoff_srv = rospy.ServiceProxy('mavros/cmd/takeoff',
                                                 CommandTOL)
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
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)
        self.temp_target_sub = rospy.Subscriber('mavros/setpoint_position/temp', PoseStamped, self.get_temp_target)

        self.mission_item_reached = -1  # first mission item is 0
        #self.mission_name = ""

        self.mavlink_pub = rospy.Publisher('mavlink/to', Mavlink, queue_size=1)
        #self.local_target = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.local_target = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
        
        self.global_target = rospy.Publisher('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
        self.reached_goal = rospy.Publisher('mavros/setpoint_position/reached', Bool, queue_size=1)
        

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
        for i in range(timeout * loop_freq):
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
    
    def set_takeoff(self, altitude, timeout):
        """take off: current longitude and latitude, x altitude, timeout(int): seconds"""
        rospy.loginfo("setting FCU takeoff")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        takeoff_set = False
        for i in range(timeout * loop_freq):
            self.reach_local_position((self.local_position.pose.position.x, self.local_position.pose.position.y, altitude))
            takeoff_set = True
            rospy.loginfo("set takeoff success | seconds: {0} of {1}".format(
                i / loop_freq, timeout))
            
            break  
            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

    def set_land(self, timeout):
        """land: current longitude and latitude, 0 altitude, timeout(int): seconds"""
        rospy.loginfo("setting FCU land")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        land_set = False
        for i in range(timeout * loop_freq):
            if self.extended_state.landed_state == self.extended_state.LANDED_STATE_ON_GROUND:
                land_set = True
                rospy.loginfo("set land success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                self.set_arm(False, 5)
                break
            else:
                try:
                    land_cmd = CommandTOL()
                    #land_cmd.latitude = current_latitude
                    #land_cmd.longitude = current_longitude
                    land_cmd.altitude = 0.0
                    res = self.set_land_srv(0,0,0,0,0)
                    if not res.success:
                        rospy.logerr("failed to send land command")
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
        for i in range(timeout * loop_freq):
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

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in range(timeout * loop_freq):
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
        for i in range(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                return True

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)
        return False

    def wait_for_takeoff_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for takeoff state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in range(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                return True

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e) 
        return False   

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in range(timeout * loop_freq):
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
        for i in range(timeout * loop_freq):
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


    def set_avoidance(self, avoidance):
        self.avoidance = avoidance

    ### Drone Control ###
    def takeoff(self, altitude:int=5, timeout:float=15.0):
        landed = self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                10, -1)
        if landed:
            if not self.state.armed:
                self.set_arm(True, 5)
                # TODO: set mode offboard?
            self.takeoff_position = [self.local_position.pose.position.x, self.local_position.pose.position.y, altitude]
            self.set_mode_srv(0, "OFFBOARD")
            success = self.reach_local_position(self.takeoff_position)
            
            return success
        else:
            return False
    
    def land(self, timeout:float=15.0):
        res = self.set_land_srv(0,0,0,0,0)
        self.set_arm(False, 5)
        return res.success
    
    def return_to_base(self, timeot=15.0):
        home = [self.home_position.position.x, self.home_position.position.y, 2]
        success = self.reach_local_position(home)
        success *= self.land()
        return success

    def reach_global_position(self, position):
        # DEPRECATED
        long, lat, alt = position
        #gb_pos = GlobalPositionTarget(longitude=long, latitude= lat, altitude=alt)
        p = Point(x=long, y=lat, z=alt)
        raw_msg = GlobalPositionTarget()

        mask = 4088
        raw_msg.header.frame_id = "target"
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.coordinate_frame = 1
        raw_msg.type_mask  = mask
        raw_msg.longitude = long
        raw_msg.latitude = lat
        raw_msg.altitude = self.altitude.amsl - alt

        
        rate = rospy.Rate(2)  # Hz
        #while not rospy.is_shutdown():
        for i in range(10):
            self.global_target.publish(raw_msg)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
    

    def calculate_velocities(self, position_target):

        maxVel = 1
        minVel = 0.4
        position_target.velocity.x = (position_target.position.x - self.local_position.pose.position.x)/10
        position_target.velocity.y = (position_target.position.y - self.local_position.pose.position.y)/10
        position_target.velocity.z = (position_target.position.z - self.local_position.pose.position.z)/10

        if position_target.velocity.x > maxVel : position_target.velocity.x = maxVel
        if position_target.velocity.y > maxVel : position_target.velocity.y = maxVel
        if position_target.velocity.z > maxVel : position_target.velocity.z = maxVel

        if position_target.velocity.x < -maxVel : position_target.velocity.x = -maxVel
        if position_target.velocity.y < -maxVel : position_target.velocity.y = -maxVel
        if position_target.velocity.z < -maxVel : position_target.velocity.z = -maxVel

        if abs(position_target.velocity.x) < 0.02 : position_target.velocity.x = 0
        if abs(position_target.velocity.y) < 0.02 : position_target.velocity.y = 0
        if abs(position_target.velocity.z) < 0.02 : position_target.velocity.z = 0

        if position_target.velocity.x > -minVel and position_target.velocity.x < 0 : position_target.velocity.x = -minVel
        if position_target.velocity.y > -minVel and position_target.velocity.y < 0 : position_target.velocity.y = -minVel
        if position_target.velocity.z > -minVel and position_target.velocity.z < 0 : position_target.velocity.z = -minVel

        if position_target.velocity.x < minVel and position_target.velocity.x > 0 : position_target.velocity.x = minVel
        if position_target.velocity.y < minVel and position_target.velocity.y > 0 : position_target.velocity.y = minVel
        if position_target.velocity.z < minVel and position_target.velocity.z > 0 : position_target.velocity.z = minVel

        return position_target
    
    def get_temp_target(self, target):
        
        if target.header.frame_id == "none":
            self.temp_target = None
        else:
            self.temp_target = target

    def get_orientation(self, target):
        x = self.local_position.pose.position.x - target[0]
        y = self.local_position.pose.position.y - target[1]
        angle = math.atan2(y, x)
        return quaternion_from_euler(0, 0, angle)

    def reach_local_position(self, target):
        
        self.reached_goal.publish(False)

        self.target_position = PoseStamped()

        self.target_position.header.frame_id = "target"
        self.target_position.header.stamp = rospy.Time.now()
        
        
        self.target_position.pose.position.x = target[0]
        self.target_position.pose.position.y = target[1]
        self.target_position.pose.position.z = target[2]
        
        rate = rospy.Rate(1)  # Hz
        self.point_reached = False
        last_target = None

        while not rospy.is_shutdown():
            
            #while not self.point_reached:

            if self.temp_target is not None:
                reach_target =  self.temp_target
            else:
                reach_target = self.target_position
            
            if (abs(self.local_position.pose.position.x - target[0]) + abs(self.local_position.pose.position.y - target[1])) > 0.5:
                qx, qy, qz, qw = self.get_orientation(target)
                reach_target.pose.orientation.x = qx
                reach_target.pose.orientation.y = qy
                reach_target.pose.orientation.z = qz
                reach_target.pose.orientation.w = qw
            
            self.local_target.publish(reach_target)
            if reach_target != last_target:
                print("Moving to:", reach_target.pose.position)
                last_target = reach_target

            
            if abs(self.local_position.pose.position.x - self.target_position.pose.position.x) < 0.5 and \
                abs(self.local_position.pose.position.y - self.target_position.pose.position.y) < 0.5 and \
                abs(self.local_position.pose.position.z - self.target_position.pose.position.z) < 0.5:
                self.point_reached = True
                rospy.loginfo("Local Position x: {0}, y: {1}, z: {2} reached.".format( \
                                self.target_position.pose.position.x, self.target_position.pose.position.y, self.target_position.pose.position.z))
                self.reached_goal.publish(True)
                return True
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

if __name__ == "__main__":
    raise ValueError("You, stupid piece of crap!")