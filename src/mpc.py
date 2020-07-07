import numpy as np
import rospy
import utils
from geometry_msgs.msg import PoseStamped

from controller import BaseController

from nav_msgs.srv import GetMap


class ModelPredictiveController(BaseController):
    def __init__(self):
        super(ModelPredictiveController, self).__init__()
        use_dynamic_obstacles = rospy.search_param("use_dynamic_obstacles")
        if use_dynamic_obstacles:
            dynamic_obstacles = rospy.get_param(rospy.search_param("use_dynamic_obstacles"), False)
            if dynamic_obstacles:
                print("MPC: use dynamic_obstacles")
                dynamic_obstacle_topics = rospy.get_param(rospy.search_param("dynamic_obstacle_topics"))
                self.num_obstacles = len(dynamic_obstacle_topics)
                self.dynamic_obstacles = np.zeros((self.num_obstacles, 2), dtype=np.float)
                for i, topic in enumerate(dynamic_obstacle_topics):
                    topic += "/car_pose"
                    rospy.Subscriber(topic, PoseStamped, self.update_obstacle, (i), queue_size=1)
        else:
            self.dynamic_obstacles = None
            self.num_obstacles = 0
        self.reset_params()

    def update_obstacle(self, msg, index):
        pose = msg.pose
        self.dynamic_obstacles[index] = np.array([pose.position.x, pose.position.y], dtype=np.float)

    def reset_params(self):
        with self.path_lock:
            self.finish_threshold = float(rospy.get_param("mpc/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("mpc/exceed_threshold", 4.0))
            self.waypoint_lookahead = float(rospy.get_param("mpc/waypoint_lookahead", 1.0))

            self.K = int(rospy.get_param("mpc/K", 24)) # Sample K rollouts
            self.T = int(rospy.get_param("mpc/T", 20)) # Each rollout has T steps
            self.speed = 0.5 #np.random.uniform(0.2, 4.0)
            # print("speed", self.speed)
                # float(rospy.get_param("mpc/speed", 1.0)) # speed of car in
                                                         # sample rollouts

            # self.collision_w = float(rospy.get_param("mpc/collision_w", 1e5))
            self.collision_w = 1e3*np.tile(np.reshape(1e2*np.arange(self.T+1.0, 1, -1), (1, self.T)), (self.K, 1))

            self.error_w = float(rospy.get_param("mpc/error_w", 1.0))

            self.car_length = float(rospy.get_param("mpc/car_length", 0.33))
            self.car_width = float(rospy.get_param("mpc/car_width", 0.15))
            self.wheelbase = float(rospy.get_param("trajgen/wheelbase", 0.33))
            self.min_delta = float(rospy.get_param("trajgen/min_delta", -0.34))
            self.max_delta = float(rospy.get_param("trajgen/max_delta", 0.34))

    def reset_state(self):
        with self.path_lock:
            self.sampled_controls = self.sample_controls()
            self.scaled = np.zeros((self.K * self.T, 3))
            self.bbox_map = np.zeros((self.K * self.T, 2, 4))
            self.perm = np.zeros(self.K * self.T).astype(np.int)
            self.map = self.get_map()
            self.perm_reg = self.load_permissible_region(self.map)
            self.map_x = self.map.info.origin.position.x
            self.map_y = self.map.info.origin.position.y
            self.map_angle = utils.rosquaternion_to_angle(self.map.info.origin.orientation)
            self.map_c = np.cos(self.map_angle)
            self.map_s = np.sin(self.map_angle)

            L = self.car_length
            W = self.car_width

            # Specify specs of bounding box
            self.bbox = np.array([
                [L / 2.0, W / 2.0],
                [L / 2.0, -W / 2.0],
                [-L / 2.0, W / 2.0],
                [-L / 2.0, -W / 2.0]
            ]) / (self.map.info.resolution)


    def get_control(self, pose, index):

        # TODO 3.1: INSERT CODE HERE. Don't modify / delete this line
        #
        # In MPC, you should first roll out K trajectories. The first
        # positions for all K trajectories are at the current position
        # (pose); the control speed for all trajectories should be the
        # one at the reference point; each of the K trajectories may have
        # a different steering angle.
        #
        # Use the kinematic car model to apply controls to the trajectories,
        # then score all rollouts with your cost function,
        # and finally select the one with the lowest cost.

        rollouts = np.zeros((self.K, self.T, 3))
        rollouts[:, 0, :] = np.array(pose) # rollouts start at the current pose
        self.sampled_controls[:, :, 0] = self.path[index, 3] # velocity
        for t in range(1, self.T):
            cur_state = rollouts[:, t - 1]
            cur_ctrl = self.sampled_controls[:, t - 1]
            xdot, ydot, thetadot = self.apply_kinematics(cur_state, cur_ctrl)
            rollouts[:, t, 0] = cur_state[:, 0] + xdot
            rollouts[:, t, 1] = cur_state[:, 1] + ydot
            rollouts[:, t, 2] = cur_state[:, 2] + thetadot

        costs = self.calculate_cost_for_rollouts(rollouts, index)
        min_control = np.argmin(costs) #find the control trajectory with the minimum cost.
        # Return the first control signal from the argmin trajectory.
        return self.sampled_controls[min_control][0]

    def sample_controls(self):
        '''
        sample_controls computes K series of control signals to be
            rolled out on each of the K trajectories on each step.

            Various methods can be used for generating a sufficient set
            of control trajectories, but we suggest stepping over the
            control space of steering angles to create the initial
            set of control trajectories.
        output:
            ctrls - a (K x T x 2) vector which lists K series of controls.
                    each series contains T steps. A control at each step
                    contains [speed, steering_angle]. Note that, if you decide
                    to sweep over the possible steering angles, you can
                    put a dummy speed here and use a different speed when
                    you actually rollout.
        '''
        # TODO 3.2: INSERT CODE HERE. Don't modify / delete this line
        #
        # Create a control library of K series of controls. Each with T
        # timesteps to use when rolling out trajectories.
        #
        # ctrls = np.zeros((self.K, self.T, 2))
        # step_size = (self.max_delta - self.min_delta) / (self.K - 1)

        ctrls = np.zeros((self.K, self.T, 2))
        step_size = (self.max_delta - self.min_delta) / (self.K - 1)
        ctrls[:, :, 0] = self.speed
        for t in range(self.T):
            ctrls[:, t, 1] = np.linspace(self.min_delta, self.max_delta, self.K)
        return ctrls

    def apply_kinematics(self, cur_x, control):
        '''
        apply_kinematics 'steps' forward the pose of the car using
            the kinematic car model for a given set of K controls.
        input:
            cur_x   (K x 3) - current K "poses" of the car
            control (K x 2) - current controls to step forward
        output:
            (x_dot, y_dot, theta_dot) - where each *_dot is a list
                of k deltas computed by the kinematic car model.
        '''
        # TODO 3.3: INSERT CODE HERE. Don't modify / delete this line
        #
        # Use the kinematic car model discussed in class to step
        # forward the pose of the car. We will step all K poses
        # simultaneously, so we recommend using Numpy to compute
        # this operation.
        #
        # ulimately, return a triplet with x_dot, y_dot_, theta_dot
        # where each is a numpy vector of length K
        # dt = 0.1
        # return (x_dot, y_dot, theta_dot)

        dt = 0.1
        speed = control[:, 0]
        steering_angle = control[:, 1]
        x_dot = speed * np.cos(cur_x[:, 2]) * dt
        y_dot = speed * np.sin(cur_x[:, 2]) * dt
        theta_dot = ((speed * np.tan(steering_angle)) / (self.wheelbase)) * dt
        return (x_dot, y_dot, theta_dot)

    def calculate_cost_for_rollouts(self, poses, index):
        '''
        rollouts (K,T,3) - poses of each rollout
        index    (int)   - reference index in path
        '''
        # TODO 3.4: INSERT CODE HERE. Don't modify / delete this line
        #
        # For each of the K given rollouts, calculate a score that
        # considers the distance to the reference waypoint and collisions

        all_poses = poses.copy()
        all_poses.resize(self.K * self.T, 3)
        collisions = self.check_collisions_in_map(all_poses)
        collisions.resize(self.K, self.T)
        collision_cost = np.sum(collisions * self.collision_w, axis=1)
        error_cost = np.linalg.norm(poses[:, self.T - 1, :2] - self.path[index, :2], axis=1) * self.error_w

        return collision_cost + error_cost

    #===============================================================
    # Collision checking and map utilities
    #===============================================================

    def check_collisions_in_map(self, poses):
        '''
        check_collisions_in_map is a collision checker that determines whether a set of K * T poses
            are in collision with occupied pixels on the map.
        input:
            poses (K * T x 3) - poses to check for collisions on
        output:
            collisions - a (K * T x 1) float vector where 1.0 signifies collision and 0.0 signifies
                no collision for the input pose with corresponding index.
        '''

        self.world2map(poses, out=self.scaled)

        bbox = self.bbox
        x = np.tile(bbox[:, 0], (len(poses), 1))
        y = np.tile(bbox[:, 1], (len(poses), 1))

        xs = self.scaled[:, 0]
        ys = self.scaled[:, 1]
        thetas = self.scaled[:, 2]

        c = np.resize(np.cos(thetas), (len(thetas), 1))
        s = np.resize(np.sin(thetas), (len(thetas), 1))

        self.bbox_map[:, 0] = (x * c - y * s) + np.tile(np.resize(xs, (len(xs), 1)), 4)
        self.bbox_map[:, 1] = (x * s + y * c) + np.tile(np.resize(ys, (len(ys), 1)), 4)

        bbox_idx = self.bbox_map.astype(np.int)

        self.perm[:] = 0
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 0], bbox_idx[:, 0, 0]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 1], bbox_idx[:, 0, 1]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 2], bbox_idx[:, 0, 2]])
        self.perm = np.logical_or(self.perm, self.perm_reg[bbox_idx[:, 1, 3], bbox_idx[:, 0, 3]])

        if self.num_obstacles > 0:
            pose_diff = np.tile(poses[:, :2][:, None, :],(1, self.dynamic_obstacles.shape[0], 1)) - np.tile(self.dynamic_obstacles[None, :, :],(poses.shape[0], 1, 1))
            distance = np.linalg.norm(pose_diff, axis=2)
            in_collision = np.any(distance < 1.0, axis=1)
            self.perm = np.logical_or(self.perm, in_collision)

        return self.perm.astype(np.float)

    def get_map(self):
        '''
        get_map is a utility function which fetches a map from the map_server
        output:
            map_msg - a GetMap message returned by the mapserver
        '''
        srv_name = rospy.get_param("static_map", default="/static_map")
        rospy.logdebug("Waiting for map service")
        rospy.wait_for_service(srv_name)
        rospy.logdebug("Map service started")

        map_msg = rospy.ServiceProxy(srv_name, GetMap)().map
        return map_msg

    def load_permissible_region(self, map):
        '''
        load_permissible_region uses map data to compute a 'permissible_region'
            matrix given the map information. In this matrix, 0 is permissible,
            1 is not.
        input:
            map - GetMap message
        output:
            pr - permissible region matrix
        '''
        map_data = np.array(map.data)
        array_255 = map_data.reshape((map.info.height, map.info.width))
        pr = np.zeros_like(array_255, dtype=bool)

        # Numpy array of dimension (map_msg.info.height, map_msg.info.width),
        # With values 0: not permissible, 1: permissible
        pr[array_255 == 0] = 1
        pr = np.logical_not(pr)  # 0 is permissible, 1 is not

        return pr

    def world2map(self, poses, out):
        '''
        world2map is a utility function which converts poses from global
            'world' coordinates (ROS world frame coordinates) to 'map'
            coordinates, that is pixel frame.
        input:
            poses - set of X input poses
            out - output buffer to load converted poses into
        '''
        out[:] = poses
        # translation
        out[:, 0] -= self.map_x
        out[:, 1] -= self.map_y

        # scale
        out[:, :2] *= (1.0 / float(self.map.info.resolution))

        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(out[:, 0])
        out[:, 0] = self.map_c * out[:, 0] - self.map_s * out[:, 1]
        out[:, 1] = self.map_s * temp + self.map_c * out[:, 1]
        out[:, 2] += self.map_angle
