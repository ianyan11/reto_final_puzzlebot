#!/usr/bin/env python3
"State machine in ros"

import rospy
from std_msgs.msg import String, Empty

class state_machine():
    def __init__(self):
        self.state = 'init'
        self.image_sub = rospy.Subscriber('/horizontal_line', Empty, self.hline_detected)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)
    
    def hline_detected(self):
        self.state = 'hline'

    def updateState(self):
        if self.state == 'init':
            self.state_pub.publish('follow_line')
        elif self.state == 'hline':
            self.state_pub.publish('stop')
        



def main():
    rospy.init_node('state_machine', anonymous=True)
    s = state_machine()
    rospy.spin()


if __name__ == "__main__":
    main()
    