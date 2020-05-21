import numpy as np
import rospy

from controller import BaseController


class PIDController(BaseController):
    def __init__(self, error='CrossTrackError'):
        super(PIDController, self).__init__(error)

    def reset_params(self):
        with self.path_lock:
            self.finish_threshold = float(rospy.get_param("/pid/finish_threshold", 0.2))
            self.exceed_threshold = float(rospy.get_param("/pid/exceed_threshold", 4.0))
            self.waypoint_lookahead = float(rospy.get_param("/pid/waypoint_lookahead", 0.6))

            self.kp = float(rospy.get_param("/pid/kp", 0.15))
            self.kd = float(rospy.get_param("/pid/kd", 0.2))

            self.error = rospy.get_param("/pid/error", "CrossTrackError")

    def reset_state(self):
        pass

    def get_control(self, pose, index):
        # TODO 2.1: INSERT CODE HERE. Don't delete this line.
        #
        # Compute the next control using the PD control strategy.
        # Consult the spec for details on how PD control works.
        error = self.get_error(pose, index)
        self.gain_p = error[1]
        self.gain_d = np.sin(np.arctan(error[1]/error[0]))
        control = self.kp * self.gain_p + self.kd * self.gain_d
        return [self.path[index, 3], float(control)]
        raise NotImplementedError

