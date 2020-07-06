#!/usr/bin/env python

import control_node
import threading
import signal
import rospy

if __name__ == '__main__':
    node = control_node.ControlNode("controller")

    signal.signal(signal.SIGINT, node.shutdown)

    controller = threading.Tread(start=node.start)
    controller.start()
    rospy.logfatal("start contoller")

    while controller.run:
        signal.pause()

    controller.join()
