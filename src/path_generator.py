#!/usr/bin/env python
from mushr_control.msg import XYHV, XYHVPath
from mushr_control.srv import FollowPath
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import PoseStamped
import tf.transformations

import numpy as np
import pickle
from scipy import signal
import rospy



def goal():
    goal_xy = np.array([5.0, 5.0])
    init_pose = np.array([0.0, 0.0, 0.0])
    # straight line
    waypoint_sep = 0.1
    line_len = np.linalg.norm(goal_xy - init_pose[:2])
    straight_xs = np.linspace(init_pose[0], goal_xy[0], int(line_len / waypoint_sep))
    straight_ys = np.linspace(init_pose[1], goal_xy[1], int(line_len / waypoint_sep))
    # TODO
    thetas = [init_pose[2]] * int(line_len / waypoint_sep)
    poses = np.array([straight_xs, straight_ys, thetas]).transpose()
    print("poses", poses.shape)
    return poses

import tf

def get_current_pose():
    car_pose_topic = \
            ('pf/inferred_pose' if rospy.has_param('controller/use_sim_pose') and int(rospy.get_param('controller/use_sim_pose')) == 0
            else 'car_pose')
    print("Listening to {} for initial pose".format(car_pose_topic))
    car_pose_msg = rospy.wait_for_message(car_pose_topic, PoseStamped)
    tf_listener = tf.TransformListener()
    msg = tf_listener.transformPose("map", car_pose_msg)
    car_pose = [msg.pose.position.x,
            msg.pose.position.y,
            rosquaternion_to_angle(msg.pose.orientation)]
    print("Current pose published", car_pose)
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

def generate_plan(local_coordinates=True):
    if local_coordinates:
        return shift_zero_pose(plans[plan_names[index]](), get_current_pose())
    else:
        return plans[plan_names[index]]()

def send_path(path):
    print ("Sending path...")
    controller = rospy.ServiceProxy("controller/follow_path", FollowPath())
    success = controller(path)
    print ("Controller started")

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


    # if wait_for_signal is on, loop until the signal comes in.
    wait_for_signal = rospy.get_param(rospy.search_param("wait_for_signal"))

    if wait_for_signal:
        msg = rospy.wait_for_message("start_path_following", Bool, timeout=None)
        if msg.data == True:
            send_path(path)
    else:
        send_path(path)