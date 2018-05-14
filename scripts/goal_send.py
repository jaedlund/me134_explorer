#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def talker():

    
    ### FINISH THIS LINE TO PUBLISH TO CORRECT TOPIC WITH CORRECT MESSAGE TYPE
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    ######
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.frame_id = "/map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.z = 0
        goal.pose.position.y = 1.0
        goal.pose.position.x = 1.0
        goal.pose.orientation.w = 1.0
        #rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
