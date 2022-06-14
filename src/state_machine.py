#!/usr/bin/env python3
"State machine in ros"

from time import time
import rospy
from std_msgs.msg import String, Empty

class state_machine():
    def __init__(self):
        self.state = 'init'
        self.image_sub = rospy.Subscriber('/horizontal_line', Empty, self.hline_detected)
        self.state_pub = rospy.Publisher('/state', String, queue_size=1)
        rospy.Rate(40)
        while( not rospy.is_shutdown()):
            self.updateState()

    def hline_detected(self,_):
        self.state = 'hline'
        self.start_time = time()

    def updateState(self):
        if self.state == 'init':
            self.state_pub.publish('follow_line')
        elif self.state == 'hline':
            self.state_pub.publish('stop')
            #measure time
            if(time() - self.start_time > 5):
                self.state = 'init'
                self.start_time = time()

        



def main():
    rospy.init_node('state_machine', anonymous=True)
    s = state_machine()


if __name__ == "__main__":
    main()
    