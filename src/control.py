#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int16
from geometry_msgs.msg import Twist

class Control():
    def __init__(self):
        self.state = 'init'
        self.state_sub = rospy.Subscriber('/state', String, queue_size=1)
        self.center_line_sub = rospy.Subscriber('/line_pose', Int16, self.center_line_detected)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Rate(100)
        while( not rospy.is_shutdown()):
            #switch case
            if self.state == 'follow_line':
                self.follow_line()
            elif self.state == 'stop':
                self.stop()
    
    def follow_line(self, data):
        t=Twist()
        t.angular.z = -data*.35
        t.linear.x = 0.4-abs(t.angular.z)*(.3/.5)
        self.velocity_pub.publish(t)

    def stop(self):
        t=Twist()
        self.velocity_pub.publish(t)
def main():
    rospy.init_node('state_machine', anonymous=True)
    c = Control()


if __name__ == "__main__":
    main()