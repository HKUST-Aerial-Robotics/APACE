#!/usr/bin/env python2

from __future__ import print_function
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_srvs.srv import Empty

import argparse
import rospy
import sys
import math
import numpy as np
import tf.transformations

current_pose = Odometry()


def pose_callback(pose):
    global current_pose
    current_pose = pose
    # print("Current pose: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(
    #     current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z))

def send(cmd, pub):
    pub.publish(cmd)

def getSegCoeff(T0, T1, b):
    coeff = np.zeros(6)
    A = np.zeros((6, 6))

    A[0, :] = [T0**5, T0**4, T0**3, T0**2, T0, 1]
    A[1, :] = [T1**5, T1**4, T1**3, T1**2, T1, 1]
    A[2, :] = [5*T0**4, 4*T0**3, 3*T0**2, 2*T0, 1, 0]
    A[3, :] = [5*T1**4, 4*T1**3, 3*T1**2, 2*T1, 1, 0]
    A[4, :] = [20*T0**3, 12*T0**2, 6*T0, 2, 0, 0]
    A[5, :] = [20*T1**3, 12*T1**2, 6*T1, 2, 0, 0]

    coeff = np.linalg.solve(A, b)
    return coeff

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='A easy command tool for sending command to swarm drone')
    parser.add_argument('command_type', metavar='command_type', choices=[
                        "takeoff", "landing", "emland", "flyto", "preset_poly", "vel", "arm", "disarm", "joy_control", "circle", "circle_yaw", "figure_eight", "figure_eight_yaw", "full_thrust", "sweep", "csv"], help="Type of command to send")
    parser.add_argument("-c", "--center", nargs=3, type=float,
                        help="center for circle", default=[0, 0, 1])
    parser.add_argument("-r", "--radius", type=float,
                        help="radius for circle", default=0.5)
    parser.add_argument("-t", "--cycle", type=float,
                        help="time for flyto or cycle for circle or cycle for sweep", default=30)
    parser.add_argument("--fmin", type=float,
                        help="min freq for sweep", default=0.1)
    parser.add_argument("--fmax", type=float,
                        help="max freq for sweep", default=5)
    parser.add_argument("--count", type=int,
                        help="sweep count number for sweep", default=3)
    parser.add_argument("-x", "--axis", type=int,
                        help="axis for sweep", default=0)
    parser.add_argument("-A", "--amp", type=float,
                        help="amp for sweep", default=1.0)
    parser.add_argument("-p", "--path", type=str, help="Path", default="")
    parser.add_argument("params", nargs="*", type=float,
                        help="parameters for command")
    args = parser.parse_args()

    print("Will send command {} with params {}".format(
        args.command_type, args.params))

    try:
        rospy.get_master().getPid()
    except:
        print("roscore is offline, exit")
        sys.exit(-1)

    rospy.init_node('airsim_trajectory_server', anonymous=True)

    pose_sub = rospy.Subscriber(
        "/airsim_node/drone_1/odom_local_enu", Odometry, pose_callback)
    # pose_sub = rospy.Subscriber(
    #     "/vins_estimator/imu_propagate", Odometry, pose_callback)
    pub = rospy.Publisher("/position_cmd", PositionCommand, queue_size=1)

    freq = 50
    rate = rospy.Rate(freq)

    i = 0
    j = 0
    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        if connections > 0:
            break

        # Debug info
        message = "Waiting for ROS connection" + "." * i
        sys.stdout.write("{}\r".format(message))
        sys.stdout.write("\033[K")
        j += 1
        if j % (freq / 2) == 0:
            i = (i+1) % 4

        rate.sleep()

    cmd = PositionCommand()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "drone_1"
    cmd.kx = [5.7, 5.7, 6.2]
    cmd.kv = [3.4, 3.4, 4.0]

    if args.command_type == "takeoff":
        if len(args.params) < 1:
            rospy.logwarn("No height specs, will fly to default height 1.0m")
            height = 1.0
        else:
            rospy.logwarn("Takeoff to height {}m".format(args.params[0]))
            height = args.params[0]

        cmd.position.x = 0
        cmd.position.y = 0
        cmd.position.z = height
        cmd.velocity.x = 0
        cmd.velocity.y = 0
        cmd.velocity.z = 0
        cmd.acceleration.x = 0
        cmd.acceleration.y = 0
        cmd.acceleration.z = 0
        cmd.jerk.x = 0
        cmd.jerk.y = 0
        cmd.jerk.z = 0
        cmd.yaw = 0

        send(cmd, pub)

    elif args.command_type == "flyto":
        if len(args.params) < 3:
            rospy.logerr("Must give xyz when using flyto")
            sys.exit(-1)

        # Define the maximum speed and acceleration
        max_speed = 5.0   # m/s
        max_acceleration = 1.5  # m/s^2

        current_pos = Point()
        current_pos.x = current_pose.pose.pose.position.x
        current_pos.y = current_pose.pose.pose.position.y
        current_pos.z = current_pose.pose.pose.position.z
        q = current_pose.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        print("Yaw: {} degrees".format(yaw * 180 / 3.14159))
        
        target_pos = Point()
        target_pos.x = args.params[0]
        target_pos.y = args.params[1]
        target_pos.z = args.params[2]

        rospy.logwarn("Fly to {} from {}".format([target_pos.x, target_pos.y, target_pos.z], [
                      current_pos.x, current_pos.y, current_pos.z]))

        target_yaw = 0.0
        if len(args.params) == 4:
            target_yaw = args.params[3]

        # Calculate the distance and time to the target position
        distance = math.sqrt((target_pos.x - current_pos.x)**2 +
                             (target_pos.y - current_pos.y)**2 +
                             (target_pos.z - current_pos.z)**2)

        t0, t1, t2, t3 = 0.0, 0.0, 0.0, 0.0

        cutoff_dist = max_speed * max_speed / max_acceleration
        if (distance < cutoff_dist):
            t3 = 2 * math.sqrt(distance / max_acceleration)
            t1 = t3 / 2
            t2 = t3 / 2
        else:
            t1 = max_speed / max_acceleration
            t2 = distance / max_speed
            t3 = distance / max_speed + max_speed / max_acceleration

        accel_distance = 0.5 * max_acceleration * t1 ** 2
        const_distance = max_speed * (t2 - t1)
        decel_distance = 0.5 * max_acceleration * (t3 - t2) ** 2

        # Initialize the time and distance counters
        t = 0
        d = 0

        while not rospy.is_shutdown():
            try:
                if t <= t1:
                    # Acceleration phase
                    v = max_acceleration * t
                    x = 0.5 * max_acceleration * t**2
                    a = max_acceleration
                elif t <= t2:
                    # Constant speed phase
                    v = max_speed
                    x = accel_distance + (t - t1) * max_speed
                    a = 0
                elif t <= t3:
                    # Deceleration phase
                    v = max_speed - max_acceleration * (t - t2)
                    x = accel_distance + const_distance + max_speed * \
                        (t-t2) - 0.5 * max_acceleration * (t - t2) ** 2
                    a = -max_acceleration
                else:
                    # Finished
                    v = 0
                    x = distance
                    a = 0

                cmd.position.x = current_pos.x + \
                    (x / distance) * (target_pos.x - current_pos.x)
                cmd.position.y = current_pos.y + \
                    (x / distance) * (target_pos.y - current_pos.y)
                cmd.position.z = current_pos.z + \
                    (x / distance) * (target_pos.z - current_pos.z)
                cmd.velocity.x = (v / distance) * \
                    (target_pos.x - current_pos.x)
                cmd.velocity.y = (v / distance) * \
                    (target_pos.y - current_pos.y)
                cmd.velocity.z = (v / distance) * \
                    (target_pos.z - current_pos.z)
                cmd.acceleration.x = (a / distance) * \
                    (target_pos.x - current_pos.x)
                cmd.acceleration.y = (a / distance) * \
                    (target_pos.y - current_pos.y)
                cmd.acceleration.z = (a / distance) * \
                    (target_pos.z - current_pos.z)
                cmd.jerk.x = 0
                cmd.jerk.y = 0
                cmd.jerk.z = 0
                cmd.yaw = target_yaw

                t += 1.0 / freq
                rospy.loginfo("{:3.2f} p {:3.2f} {:3.2f} {:3.2f} yaw {:3.2f} v {:3.2f} {:3.2f} {:3.2f} a {:3.2f} {:3.2f} {:3.2f}".format(
                    t, cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw, cmd.velocity.x, cmd.velocity.y, cmd.velocity.z, cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z))
                send(cmd, pub)
                rate.sleep()

                if (t > t3 + 5.0):
                    exit(0)
            except KeyboardInterrupt:
                exit(0)

        # T = args.cycle
        # t = 0

        # while not rospy.is_shutdown():
        #     try:
        #         if (t > T):
        #             cmd.position.x = target_pos.x
        #             cmd.position.y = target_pos.y
        #             cmd.position.z = target_pos.z
        #             cmd.velocity.x = 0
        #             cmd.velocity.y = 0
        #             cmd.velocity.z = 0
        #             cmd.acceleration.x = 0
        #             cmd.acceleration.y = 0
        #             cmd.acceleration.z = 0
        #             cmd.jerk.x = 0
        #             cmd.jerk.y = 0
        #             cmd.jerk.z = 0
        #             cmd.yaw = target_yaw
        #             # break
        #         else:
        #             cmd.position.x = current_pos.x + \
        #                 (target_pos.x - current_pos.x) * t / T
        #             cmd.position.y = current_pos.y + \
        #                 (target_pos.y - current_pos.y) * t / T
        #             cmd.position.z = current_pos.z + \
        #                 (target_pos.z - current_pos.z) * t / T
        #             cmd.velocity.x = (target_pos.x - current_pos.x) / T
        #             cmd.velocity.y = (target_pos.y - current_pos.y) / T
        #             cmd.velocity.z = (target_pos.z - current_pos.z) / T
        #             cmd.acceleration.x = 0
        #             cmd.acceleration.y = 0
        #             cmd.acceleration.z = 0
        #             cmd.jerk.x = 0
        #             cmd.jerk.y = 0
        #             cmd.jerk.z = 0
        #             cmd.yaw = target_yaw

        #             t += 1.0 / freq

        #         send(cmd, pub)
        #         rate.sleep()
        #     except KeyboardInterrupt:
        #         exit(0)

    elif args.command_type == "preset_poly":
        current_pos = Point()
        current_pos.x = current_pose.pose.pose.position.x
        current_pos.y = current_pose.pose.pose.position.y
        current_pos.z = current_pose.pose.pose.position.z
        q = current_pose.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        print("Yaw: {} degrees".format(yaw * 180 / 3.14159))

        dt = 0.02
        T = np.array([2.0, 1.0, 1.0, 2.0])
        duration = np.sum(T)
        
        y_all = np.array([0.0, 5.0, 10.0, 15.0, 20.0])
        vy_all = np.array([0.0, 5.0, 5.0, 5.0, 0.0])
        ay_all = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        x_all = np.array([0.0, 0.0, 0.0])
        vx_all = np.array([0.0, 0.0, 0.0])
        ax_all = np.array([0.0, 0.0, 0.0])
        
        # x_all = np.array([0.0, -2.0, 0.0])
        # vx_all = np.array([0.0, 0.0, 0.0])
        # ax_all = np.array([0.0, 1.5, 0.0])

        x_all += current_pos.x
        y_all += current_pos.y
        
        # Call the estimation evaluation service
        start_eval = rospy.ServiceProxy('/services/start_eval', Empty)
        response = start_eval()

        t = 0

        while not rospy.is_shutdown():
            try:
                if (t < T[0]):
                    by = np.array([y_all[0], y_all[1], vy_all[0], vy_all[1], ay_all[0], ay_all[1]])
                    y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(0, T[0], by))
                    vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(0, T[0], by))
                    ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(0, T[0], by))
                elif (t < T[0] + T[1]):
                    by = np.array([y_all[1], y_all[2], vy_all[1], vy_all[2], ay_all[1], ay_all[2]])
                    y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0], T[0] + T[1], by))
                    vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0], T[0] + T[1], by))
                    ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0], T[0] + T[1], by))
                elif (t < T[0] + T[1] + T[2]):
                    by = np.array([y_all[2], y_all[3], vy_all[2], vy_all[3], ay_all[2], ay_all[3]])
                    y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2], by))
                    vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2], by))
                    ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2], by))
                elif (t < T[0] + T[1] + T[2] + T[3]):
                    by = np.array([y_all[3], y_all[4], vy_all[3], vy_all[4], ay_all[3], ay_all[4]])
                    y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0] + T[1] + T[2], T[0] + T[1] + T[2] + T[3], by))
                    vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0] + T[1] + T[2], T[0] + T[1] + T[2] + T[3], by))
                    ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0] + T[1] + T[2], T[0] + T[1] + T[2] + T[3], by))
                
                if (t < T[0] + T[1]):
                    bx = np.array([x_all[0], x_all[1], vx_all[0], vx_all[1], ax_all[0], ax_all[1]])
                    x = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(0, T[0] + T[1], bx))
                    vx = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(0, T[0] + T[1], bx))
                    ax = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(0, T[0] + T[1], bx))
                elif (t < T[0] + T[1] + T[2] + T[3]):
                    bx = np.array([x_all[1], x_all[2], vx_all[1], vx_all[2], ax_all[1], ax_all[2]])
                    x = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2] + T[3], bx))
                    vx = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2] + T[3], bx))
                    ax = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2] + T[3], bx))
                else:
                    x = x_all[-1]
                    y = y_all[-1]
                    vx = vx_all[-1]
                    vy = vy_all[-1]
                    ax = ax_all[-1]
                    ay = ay_all[-1]
                
                end_yaw = -0.78
                # end_yaw = 0.0

                cmd.position.x = x
                cmd.position.y = y
                cmd.position.z = current_pos.z
                cmd.velocity.x = vx
                cmd.velocity.y = vy
                cmd.velocity.z = 0
                cmd.acceleration.x = ax
                cmd.acceleration.y = ay
                cmd.acceleration.z = 0
                cmd.jerk.x = 0
                cmd.jerk.y = 0
                cmd.jerk.z = 0
                cmd.yaw = (end_yaw - yaw) * t / duration
                
                t += 1.0 / freq
                rospy.loginfo("{:3.2f} p {:3.2f} {:3.2f} {:3.2f} yaw {:3.2f} v {:3.2f} {:3.2f} {:3.2f} a {:3.2f} {:3.2f} {:3.2f}".format(
                    t, cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw, cmd.velocity.x, cmd.velocity.y, cmd.velocity.z, cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z))
                send(cmd, pub)
                rate.sleep()

                if (t > T[0] + T[1] + T[2] + T[3]):
                    # Call the estimation evaluation service
                    end_eval = rospy.ServiceProxy('/services/end_eval', Empty)
                    response = end_eval()
                    exit(0)
            except KeyboardInterrupt:
                exit(0)

    elif args.command_type == "circle" or args.command_type == "circle_yaw":
        print("Will draw circle @ origin {} {} {}, r {} T {}".format(
            args.center[0],
            args.center[1],
            args.center[2],
            args.radius,
            args.cycle
        ))

        ox = args.center[0]
        oy = args.center[1]
        oz = args.center[2]
        r = args.radius
        T = args.cycle

        current_pos = Point()
        current_pos.x = current_pose.pose.pose.position.x
        current_pos.y = current_pose.pose.pose.position.y
        current_pos.z = current_pose.pose.pose.position.z

        target_pos = Point()
        target_pos.x = ox + math.sin(0)*r
        target_pos.y = oy + math.cos(0)*r
        target_pos.z = oz

        flyto_speed = 1.0

        t = 0
        t_flyto = 0
        dist_flyto = math.sqrt(
            (target_pos.x - current_pos.x)**2 +
            (target_pos.y - current_pos.y)**2 +
            (target_pos.z - current_pos.z)**2
        )
        T_flyto = dist_flyto / flyto_speed
        yaw = 0
        while not rospy.is_shutdown():
            try:
                # Fly to start position
                if (t_flyto < T_flyto):
                    rospy.loginfo("Fly to start position")
                    cmd.position.x = current_pos.x + \
                        (target_pos.x - current_pos.x) * t_flyto / T_flyto
                    cmd.position.y = current_pos.y + \
                        (target_pos.y - current_pos.y) * t_flyto / T_flyto
                    cmd.position.z = current_pos.z + \
                        (target_pos.z - current_pos.z) * t_flyto / T_flyto
                    cmd.velocity.x = (target_pos.x - current_pos.x) / T_flyto
                    cmd.velocity.y = (target_pos.y - current_pos.y) / T_flyto
                    cmd.velocity.z = (target_pos.z - current_pos.z) / T_flyto
                    cmd.acceleration.x = 0
                    cmd.acceleration.y = 0
                    cmd.acceleration.z = 0
                    cmd.jerk.x = 0
                    cmd.jerk.y = 0
                    cmd.jerk.z = 0
                    cmd.yaw = yaw

                    t_flyto += 1.0 / freq
                else:
                    cmd.position.x = ox + math.sin(t*math.pi*2/T)*r
                    cmd.position.y = oy + math.cos(t*math.pi*2/T)*r
                    cmd.position.z = oz
                    cmd.velocity.x = math.cos(t*math.pi*2/T) * r * math.pi*2/T
                    cmd.velocity.y = -math.sin(t*math.pi*2/T) * r * math.pi*2/T
                    cmd.velocity.z = 0.0
                    cmd.acceleration.x = - \
                        math.sin(t*math.pi*2/T) * r * math.pi*2/T * math.pi*2/T
                    cmd.acceleration.y = - \
                        math.cos(t*math.pi*2/T) * r * math.pi*2/T * math.pi*2/T
                    cmd.acceleration.z = 0.0
                    if args.command_type == "circle_yaw":
                        cmd.yaw = t*math.pi*2/T
                    t += 1 / freq

                    # rospy.loginfo("{:3.2f} xyz {:3.2f} {:3.2f} {:3.2f} Y {:3.2f} ff {:3.2f} {:3.2f} {:3.2f} {:3.2f}".format(
                    #     t, x, y, oz, yaw, vx, vy, ax, ay))
                rospy.loginfo("{:3.2f} p {:3.2f} {:3.2f} {:3.2f} yaw {:3.2f} v {:3.2f} {:3.2f} {:3.2f} a {:3.2f} {:3.2f} {:3.2f}".format(
                    t, cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw, cmd.velocity.x, cmd.velocity.y, cmd.velocity.z, cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z))
                send(cmd, pub)
                rate.sleep()

            except KeyboardInterrupt:
                exit(0)

    elif args.command_type == "figure_eight" or args.command_type == "figure_eight_yaw":
        print("Will fly figure eight @ origin {} {} {}, r {} T {}".format(
            args.center[0],
            args.center[1],
            args.center[2],
            args.radius,
            args.cycle
        ))

        ox = args.center[0]
        oy = args.center[1]
        oz = args.center[2]
        r = args.radius
        T = args.cycle

        current_pos = Point()
        current_pos.x = current_pose.pose.pose.position.x
        current_pos.y = current_pose.pose.pose.position.y
        current_pos.z = current_pose.pose.pose.position.z

        target_pos = Point()
        target_pos.x = ox
        target_pos.y = oy
        target_pos.z = oz

        flyto_speed = 1.0

        t = 0
        t_flyto = 0
        dist_flyto = math.sqrt(
            (target_pos.x - current_pos.x)**2 +
            (target_pos.y - current_pos.y)**2 +
            (target_pos.z - current_pos.z)**2
        )
        T_flyto = dist_flyto / flyto_speed
        yaw = 0
        while not rospy.is_shutdown():
            try:
                # Fly to start position
                if (t_flyto < T_flyto):
                    rospy.loginfo("Fly to start position")
                    cmd.position.x = current_pos.x + \
                        (target_pos.x - current_pos.x) * t_flyto / T_flyto
                    cmd.position.y = current_pos.y + \
                        (target_pos.y - current_pos.y) * t_flyto / T_flyto
                    cmd.position.z = current_pos.z + \
                        (target_pos.z - current_pos.z) * t_flyto / T_flyto
                    cmd.velocity.x = (target_pos.x - current_pos.x) / T_flyto
                    cmd.velocity.y = (target_pos.y - current_pos.y) / T_flyto
                    cmd.velocity.z = (target_pos.z - current_pos.z) / T_flyto
                    cmd.acceleration.x = 0
                    cmd.acceleration.y = 0
                    cmd.acceleration.z = 0
                    cmd.jerk.x = 0
                    cmd.jerk.y = 0
                    cmd.jerk.z = 0
                    cmd.yaw = yaw

                    t_flyto += 1.0 / freq
                else:
                    x = ox + math.sin(t*math.pi/T)*r
                    y = oy + math.cos(t*math.pi/T)*r * math.sin(t*math.pi/T)
                    z = oz
                    cmd.position.x = x
                    cmd.position.y = y
                    cmd.position.z = z
                    cmd.velocity.x = math.cos(t*math.pi/T) * r * math.pi/T
                    cmd.velocity.y = math.sin(
                        t*math.pi/T) * r * math.pi/T * math.cos(t*math.pi/T)
                    cmd.velocity.z = 0.0
                    cmd.acceleration.x = - \
                        math.sin(t*math.pi/T) * r * math.pi/T * math.pi/T
                    cmd.acceleration.y = math.cos(
                        t*math.pi/T) * r * math.pi/T * math.sin(t*math.pi/T) * math.pi/T
                    cmd.acceleration.z = 0.0
                    if args.command_type == "figure_eight_yaw":
                        cmd.yaw = math.atan2(math.cos(t*math.pi/T) * r * math.pi/T * math.sin(
                            t*math.pi/T), math.cos(t*math.pi/T) * r * math.pi/T)
                    t += 1 / freq

                rospy.loginfo("{:3.2f} p {:3.2f} {:3.2f} {:3.2f} yaw {:3.2f} v {:3.2f} {:3.2f} {:3.2f} a {:3.2f} {:3.2f} {:3.2f}".format(
                    t, cmd.position.x, cmd.position.y, cmd.position.z, cmd.yaw, cmd.velocity.x, cmd.velocity.y, cmd.velocity.z, cmd.acceleration.x, cmd.acceleration.y, cmd.acceleration.z))
                send(cmd, pub)
                rate.sleep()

            except KeyboardInterrupt:
                exit(0)
