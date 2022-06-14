#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class Control():
    def __init__(self):
        self.state = 'follow_line'
        self.center_line = 0
        rospy.Subscriber('/state', String, self.state_update)
        rospy.Subscriber('/line_pose', Float32, self.cline_update)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Rate(40)
        while( not rospy.is_shutdown()):
            #switch case
            if self.state == 'follow_line':
                self.follow_line()
            elif self.state == 'stop':
                rospy.loginfo("stop")
                self.stop()
    
    def follow_line(self):
        t=Twist()
        t.angular.z = -self.center_line*.35
        t.linear.x = 0.4-abs(t.angular.z)*(.3/.5)
        self.velocity_pub.publish(t)

    def stop(self):
        t=Twist()
        self.velocity_pub.publish(t)
    
    #Center line update
    def cline_update(self, data: Float32):
        self.center_line = data.data
    #State update
    def state_update(self, data: String):
        self.state = data.data
def main():
    rospy.init_node('state_machine', anonymous=True)
    c = Control()


if __name__ == "__main__":
    main()