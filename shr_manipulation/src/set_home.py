#!/usr/bin/env python
# license removed for brevity
import rospy
from interbotix_xs_msgs.msg import JointGroupCommand


if __name__ == '__main__':
    pub = rospy.Publisher('/vx300s/commands/joint_group', JointGroupCommand, queue_size=10)
    rospy.init_node('set_home', anonymous=True)

    rospy.sleep(5)


    cmds = [
        [0, -1.85, 1.55, 0, 0.8, 0],
        [0, -1.85, 1.55, 0, 0, 0],
        [0, -0.715585, 0.383972, 0, 0, 0],
        [0, -0.715585, 0.383972, 0, 1.8326, 0],
    ]

    rate = rospy.Rate(1)
    for cmd in cmds:
        msg = JointGroupCommand()
        msg.name = 'arm'
        msg.cmd = cmd

        pub.publish(msg)
        rate.sleep()

            # [0, -0.715585, 0.383972, 0, 1.8326, 0],
        # [0, -0.715585, 0.383972, 0, 1.8326, 0],