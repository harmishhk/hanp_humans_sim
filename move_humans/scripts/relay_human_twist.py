#!/usr/bin/env python
import rospy
import argparse
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from hanp_msgs.msg import HumanTwist
from hanp_msgs.msg import HumanTwistArray


class RelayHumanTwist(object):

    def __init__(self):
        parser = argparse.ArgumentParser(
            formatter_class=argparse.RawTextHelpFormatter,
            description=(
                'Allows to relay geometry_msgs/Twist data from input_topic to output_topic.\n\n'
                'Usage:\n\trosrun move_humans relay_human_twist <input_topic> <output_topic> <human_id>'
            )
        )
        parser.add_argument(
            'input_topic', help='Input topic of type geometry_msgs/Twist')
        parser.add_argument(
            'output_topic', help='Output topic will be of type hanp_msgs/HumanTwistArray')
        parser.add_argument('human_id', help='Human ID for output_topic')
        argv = rospy.myargv()
        args = parser.parse_args(argv[1:])

        self.human_id = args.human_id
        self.sub = rospy.Subscriber(
            args.input_topic, Twist, self.callback)
        self.pub = rospy.Publisher(
            args.output_topic, HumanTwistArray, queue_size=1)

    def callback(self, m):
        msg = HumanTwistArray()
        msg.header = Header(stamp=rospy.get_rostime())
        msg.twists.append(HumanTwist(id=int(self.human_id), twist=m))
        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('relay_human_twist', anonymous=True)
    app = RelayHumanTwist()
    rospy.spin()
