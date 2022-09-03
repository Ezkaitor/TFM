from drone import Drone
import rospy
from pymavlink import mavutil
import math
from six.moves import xrange

def distance_to_wp(self, lat, lon, alt):
    """alt(amsl): meters"""
    R = 6371000  # metres
    rlat1 = math.radians(lat)
    rlat2 = math.radians(self.global_position.latitude)

    rlat_d = math.radians(self.global_position.latitude - lat)
    rlon_d = math.radians(self.global_position.longitude - lon)

    a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
            math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    d = R * c
    alt_d = abs(alt - self.altitude.amsl)

    rospy.logdebug("d: {0}, alt_d: {1}".format(d, alt_d))
    return d, alt_d

def reach_position(self, lat, lon, alt, timeout, index):
    """alt(amsl): meters, timeout(int): seconds"""
    rospy.loginfo(
        "trying to reach waypoint | lat: {0:.9f}, lon: {1:.9f}, alt: {2:.2f}, index: {3}".
        format(lat, lon, alt, index))
    best_pos_xy_d = None
    best_pos_z_d = None
    reached = False
    mission_length = 2

    # does it reach the position in 'timeout' seconds?
    loop_freq = 2  # Hz
    rate = rospy.Rate(loop_freq)
    for i in xrange(timeout * loop_freq):
        pos_xy_d, pos_z_d = distance_to_wp(self, lat, lon, alt)

        # remember best distances
        if not best_pos_xy_d or best_pos_xy_d > pos_xy_d:
            best_pos_xy_d = pos_xy_d
        if not best_pos_z_d or best_pos_z_d > pos_z_d:
            best_pos_z_d = pos_z_d

        # FCU advanced to the next mission item, or finished mission
        reached = (
            # advanced to next wp
            (index < 3)
            # end of mission
            or (index == (mission_length - 1) and
                self.mission_item_reached == index))

        if reached:
            rospy.loginfo(
                "position reached | pos_xy_d: {0:.2f}, pos_z_d: {1:.2f}, index: {2} | seconds: {3} of {4}".
                format(pos_xy_d, pos_z_d, index, i / loop_freq, timeout))
            break
        elif i == 0 or ((i / loop_freq) % 10) == 0:
            # log distance first iteration and every 10 sec
            rospy.loginfo(
                "current distance to waypoint | pos_xy_d: {0:.2f}, pos_z_d: {1:.2f}, index: {2}".
                format(pos_xy_d, pos_z_d, index))

        try:
            rate.sleep()
        except rospy.ROSException as e:
            print(e)


def main():
    drone = Drone()
    drone.setUp()
    # make sure the simulation is ready to start the mission
    drone.wait_for_topics(60)
    drone.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                10, -1)
    drone.wait_for_mav_type(10)
    drone.tearDown()
    
    #drone.set_mode("OFFBOARD", 5) # Not working

    

    #drone.set_mode("AUTO.TAKEOFF", 5) # Only Auto modes are set up

    #drone.wait_for_takeoff_state(mavutil.mavlink.MAV_LANDED_STATE_IN_AIR, 10, -1)

    #drone.set_mode("MANUAL", 5)

    def publish_point(*args, **kwargs):
        drone.global_target.publish(drone.point)
    alt = 5
    alt += drone.altitude.amsl - drone.altitude.relative
    rospy.loginfo(drone.state)
    drone.set_arm(True, 5)
    drone.set_mode_srv(0, "AUTO.TAKEOFF")
    drone.reach_position((56.39773941040039, 8.5455904006958, alt))
    drone.set_mode_srv(0, "OFFBOARD")
    setvel_timer = rospy.Timer(rospy.Duration(0.05), publish_point)
    #drone.set_mode("OFFBOARD", 5) # Not working
    #drone.set_arm(True, 5)
    #reach_position(drone, 47.39773941040039, 8.5455904006958, alt, 60, 0)
    #drone.wait_for_vtol_state(mavutil.mavlink.MAV_VTOL_STATE_FW, 60, 0)

    alt = 10
    alt += drone.altitude.amsl - drone.altitude.relative

    #drone.reach_position((47.39773941040039, 25.5455904006958, alt))

    #reach_position(drone, 47.39773941040039, 8.54559326171875, alt, 60, 1)
    #drone.wait_for_vtol_state(0, 60, 0)



if __name__ == "__main__":
    rospy.init_node("test_node", anonymous=True)
    main()
    rospy.spin()