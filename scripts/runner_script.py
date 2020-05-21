#!/usr/bin/env python
from mushr_control.msg import XYHV, XYHVPath
from mushr_control.srv import FollowPath
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
import tf.transformations

import numpy as np
import pickle
from scipy import signal
import rospy


def saw():
    t = np.linspace(0, 20, 100)
    saw = signal.sawtooth(0.5 * np.pi * t)
    configs = [[x, y, 0] for (x, y) in zip(t, saw)]
    return configs

def wave():
    t = np.linspace(0, 20, 100)
    y = np.sin(t)
    theta = np.cos(t)
    configs = [[x, y, _theta] for (x, y, _theta) in zip(t, y, theta)]
    return configs


def circle():
    waypoint_sep = 0.1
    radius = 2.5
    center = [0, radius]
    num_points = int((2 * radius * np.pi) / waypoint_sep)
    thetas = np.linspace(-1 * np.pi / 2, 2 * np.pi - (np.pi / 2), num_points)
    poses = [[radius * np.cos(theta) + center[0], radius * np.sin(theta) + center[1], theta + (np.pi / 2)] for theta in thetas]
    return poses


def left_turn():
    waypoint_sep = 0.1
    turn_radius = 1.5
    straight_len = 10.0
    turn_center = [straight_len, turn_radius]
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]
    num_turn_points = int((turn_radius * np.pi * 0.5) / waypoint_sep)
    thetas = np.linspace(-1 * np.pi / 2, 0, num_turn_points)
    turn_poses = [[turn_radius * np.cos(theta) + turn_center[0], turn_radius * np.sin(theta) + turn_center[1], theta + (np.pi / 2)] for theta in thetas]
    poses = straight_poses + turn_poses
    return poses


def right_turn():
    waypoint_sep = 0.1
    turn_radius = 1.5
    straight_len = 10.0
    turn_center = [straight_len, -turn_radius]
    straight_xs = np.linspace(0, straight_len, int(straight_len / waypoint_sep))
    straight_poses = [[x, 0, 0] for x in straight_xs]
    num_turn_points = int((turn_radius * np.pi * 0.5) / waypoint_sep)
    thetas = np.linspace(1 * np.pi / 2, 0, num_turn_points)
    turn_poses = [[turn_radius * np.cos(theta) + turn_center[0], turn_radius * np.sin(theta) + turn_center[1], theta - (np.pi / 2)] for theta in thetas]
    poses = straight_poses + turn_poses
    return poses


def cse022_path():
    import os
    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(os.path.join(dir_path, 'cse022_path.pickle'), 'r') as f:
        p = pickle.load(f)
    return p


plans = {'circle': circle, 'left turn': left_turn, 'right turn': right_turn, 'wave': wave, 'cse022 real path': cse022_path}
plan_names = ['circle', 'left turn', 'right turn', 'wave', 'cse022 real path']

import tf

def get_current_pose():
    car_pose_topic = \
            ('/pf/inferred_pose' if rospy.has_param('/controller/use_sim_pose') and int(rospy.get_param('/controller/use_sim_pose')) == 0
            else '/car_pose')
    print("Listening to {} for initial pose".format(car_pose_topic))
    car_pose_msg = rospy.wait_for_message(car_pose_topic, PoseStamped)
    tf_listener = tf.TransformListener()
    msg = tf_listener.transformPose("map", car_pose_msg)
    car_pose = [msg.pose.position.x,
            msg.pose.position.y,
            rosquaternion_to_angle(msg.pose.orientation)]
    return car_pose

def rosquaternion_to_angle(q):
    """Convert a quaternion _message_ into an angle in radians.
    The angle represents the yaw.
    This is not just the z component of the quaternion."""
    x, y, z, w = q.x, q.y, q.z, q.w
    _, _, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
    return yaw

def shift_zero_pose(configs, shift_by):
    shift_by[2] = 0
    shifted_configs = []
    _configs = np.array(configs) + np.array(shift_by)
    for c in _configs:
        #c[2] = (c[2] + 2*np.pi) % (2*np.pi)
        shifted_configs.append(list(c))
    return shifted_configs

def generate_plan():
    print "Which plan would you like to generate? "
    for i, name in enumerate(plan_names):
        print "{} ({})".format(name, i)
    index = int(raw_input("num: "))
    if index >= len(plan_names):
        print "Wrong number. Exiting."
        exit()
    if plan_names[index] == 'cse022 real path':
        return plans[plan_names[index]]()
    return shift_zero_pose(plans[plan_names[index]](), get_current_pose())


if __name__ == '__main__':
    rospy.init_node("controller_runner")
    configs = generate_plan()

    if type(configs) == XYHVPath:
        path = configs
    else:
        h = Header()
        h.stamp = rospy.Time.now()
        desired_speed = 2.0
        ramp_percent = 0.1
        ramp_up = np.linspace(0.0, desired_speed, int(ramp_percent * len(configs)))
        ramp_down = np.linspace(desired_speed, 0.3, int(ramp_percent * len(configs)))
        speeds = np.zeros(len(configs))
        speeds[:] = desired_speed
        speeds[0:len(ramp_up)] = ramp_up
        speeds[-len(ramp_down):] = ramp_down
        path = XYHVPath(h, [XYHV(*[config[0], config[1], config[2], speed]) for config, speed in zip(configs, speeds)])
    print "Sending path..."
    controller = rospy.ServiceProxy("/controller/follow_path", FollowPath())
    success = controller(path)
    print "Controller started."
