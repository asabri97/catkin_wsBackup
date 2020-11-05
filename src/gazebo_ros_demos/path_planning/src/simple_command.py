#! /usr/bin/env python
import rospy
from common_utils.node_client import NodeClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class _SimpleCommand(NodeClient):
    def __init__(self):
        self.declare_topics()
        name = 'SimpleCommand'
        NodeClient.__init__(self, name=name, do_spin=False)
        self.publish_joint_1()

    def log_joint_states(self, msg):
        rospy.loginfo(msg)

    def do_nothing(self, msg):
        pass

    def publish_joint_1(self):
        print("Start publishing")
        msg = Float64()
        rate = rospy.Rate(1)
        # to be encapsulated with self.is_shutdown at a later stage
        while not rospy.is_shutdown():
            print(msg)
            msg.data += 0.1
            self.publishers['/rrbot/joint1_position_controller/command'].publish(msg)
            rate.sleep()
        print("End publishing")

    def declare_topics(self):
        self.publishers_declaration = [
            {'topic': '/rrbot/joint1_position_controller/command', 'data_class': Float64}
        ]


if __name__ == '__main__':
    test = _SimpleCommand()

