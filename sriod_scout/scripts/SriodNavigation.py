#!/usr/bin/env python3
import rospy
from sriod_scout.msg import SriodData
from geometry_msgs.msg import Twist


class SriodNavigation():
    def __init__(self):
        rospy.Subscriber('sriod_data_topic', SriodData, self.sriod_data_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def run(self):
        rospy.spin()

    def sriod_data_callback(self, msg):
        ctl_value = msg.ctl
        ctr_value = msg.ctr
        ctu_value = msg.ctu
        ctd_value = msg.ctd

        linear_x = (ctu_value - ctd_value) * 0.0000001
        angular_vel = (ctl_value - ctr_value) * 0.0000001

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_vel

        self.cmd_vel_pub.publish(twist_msg)

if __name__ == "__main__":
    try:
        # Initialize ROS node
        rospy.init_node('sriod_navigation')

        # Instantiate SriodNavigation class
        node = SriodNavigation()

        # Run the node
        node.run()
    except rospy.ROSInterruptException:
        pass
