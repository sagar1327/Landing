#!/usr/bin/env python

import rospy
from rospy.topics import Publisher
from std_msgs.msg import String


class MinimalPublisher(Publisher):
    def __init__(self):
        super().__init__('minimal_publisher', String, queue_size=10)
        rospy.init_node('talker')
        self.callback()

    def callback(self):
        msg = 'Hello World'
        self.publish(msg)
        rospy.loginfo(msg)


def main():
    MinimalPublisher()


if __name__ == '__main__':
    main()


