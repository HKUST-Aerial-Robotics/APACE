import rospy

from mavros_msgs.msg import AttitudeTarget


if __name__ == "__main__":
    rospy.init_node("airsim_full_thrust", anonymous=True)
    pub = rospy.Publisher(
        "/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        cmd = AttitudeTarget()
        cmd.type_mask = 0
        cmd.thrust = 1.0
        pub.publish(cmd)
        rate.sleep()
