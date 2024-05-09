import rospy
from geometry_msgs.msg import Point, Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.interpolate import BSpline, splprep
import numpy as np

# Define the center position and radius of the circle
center = [1, 1, 1] # x, y, z
radius = 2

# Define the number of control points and the degree of the b-spline
n_control_points = 10
degree = 3

# Define the time range of the trajectory
t_min = 0
t_max = 5
t_range = np.linspace(t_min, t_max, int((t_max-t_min)/0.02))

# Generate the control points for the b-spline
theta_range = np.linspace(0, 2*np.pi, n_control_points)
x = center[0] + radius*np.cos(theta_range)
y = center[1] + radius*np.sin(theta_range)
z = center[2]*np.ones_like(theta_range)

# Generate the b-spline
tck, u = splprep([x, y, z], k=degree, s=0, per=True)

# Set the boundary conditions for the b-spline
tck[1] = np.concatenate(([0, 0, 0], tck[1][1:-1], [1, 1, 1]))

bspline = BSpline(*tck)

# Generate the trajectory points
positions = []
velocities = []
accelerations = []
jerks = []

for t in t_range:
    position = Point()
    velocity = Vector3()
    acceleration = Vector3()
    jerk = Vector3()
    
    pos = bspline(t)
    vel = bspline.derivative(nu=1)(t)
    acc = bspline.derivative(nu=2)(t)
    jrk = bspline.derivative(nu=3)(t)
    
    position.x, position.y, position.z = pos
    velocity.x, velocity.y, velocity.z = vel
    acceleration.x, acceleration.y, acceleration.z = acc
    jerk.x, jerk.y, jerk.z = jrk
    
    positions.append(position)
    velocities.append(velocity)
    accelerations.append(acceleration)
    jerks.append(jerk)

# Publish the trajectory
rospy.init_node('bspline_trajectory')
pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

trajectory = JointTrajectory()
trajectory.header.stamp = rospy.Time.now()
trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

for t in t_range:
    point = JointTrajectoryPoint()
    point.positions = [positions[t].x, positions[t].y, positions[t].z, 0, 0, 0]
    point.velocities = [velocities[t].x, velocities[t].y, velocities[t].z, 0, 0, 0]
    point.accelerations = [accelerations[t].x, accelerations[t].y, accelerations[t].z, 0, 0, 0]
    point.effort = [0, 0, 0, 0, 0, 0]
    point.time_from_start = rospy.Duration.from_sec(t)
    trajectory.points.append(point)
    print("point.positions: ", point.positions)
    print("point.velocities: ", point.velocities)
    print("point.accelerations: ", point.accelerations)
    # pub.publish(trajectory)
    rospy.sleep(0.02)