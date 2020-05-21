import numpy as np
import rospy
from controller import BaseController


class PurePursuitController(BaseController):
    def __init__(self, error='CrossTrackError'):
        super(PurePursuitController, self).__init__(error)

    def reset_params(self):
        with self.path_lock:
            self.speed = float(rospy.get_param("/purpursuit/speed", 1.0))
            self.finish_threshold = float(rospy.get_param("/purepursuit/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/purepursuit/exceed_threshold", 4.0))
            # Lookahead distance from current pose to next reference waypoint.
            # Different from waypoint_lookahead used by other controllers,
            # as those are indexes from the reference point.
            self.distance_lookahead = float(rospy.get_param("/purepursuit/distance_lookahead", 0.6))

    def reset_state(self):
        pass

    def get_reference_index(self, pose):
        '''
        purepursuit controller uses a different way to find reference index
        it finds the next reference waypoint that is about distance_lookahead
        away from the current pose
        '''
        # TODO E.P1: INSERT CODE HERE. Don't modify / delete this line
        #
        # Use the pure pursuit lookahead method described in the
        # handout for determining the reference index.
        #
        # Note: this method must be computationally efficient
        # as it is running directly in the tight control loop.
        with self.path_lock:
            pose = np.array(pose)
            diff = self.path[:, :3] - pose
            dist = np.linalg.norm(diff[:, :2], axis=1)
            index = dist.argmin()
            for i in range(index, len(self.path)):
                if dist[i] > self.distance_lookahead:
                    return i
            return len(self.path)-1
        raise NotImplementedError

    def get_control(self, pose, index):
        # TODO E.P2: INSERT CODE HERE. Don't modify / delete this line
        #
        # Use the pure pursuit control method to compute the
        # steering angle. Refer to the hand out and referenced
        # articles for more details about this strategy.
        error = self.get_error(pose, index)
        cte = error[1]
        control = (2 * cte) / (self.distance_lookahead ** 2)
        return [self.path[index, 3], float(control)]
        raise NotImplementedError



