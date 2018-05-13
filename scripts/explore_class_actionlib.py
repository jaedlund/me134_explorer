#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import ros_numpy
#from std_msgs.msg import Int8MultiArray
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class ME134_Explorer:
    def __init__(self):
        self.last_goal = None
        self.last_map = None
        self.last_map_metadata = None
        self.last_scan = None
        self.last_pose = None
        self.abort = False
        self.mode = None

        self.goal_queue = []
        # listen to geometry_msgs and nav_msgs topics
        rospy.init_node('me134_explorer', anonymous=False)
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        rospy.Subscriber("map_metadata", MapMetaData, self.mapMetaDataCallback)
        #rospy.Subscriber("scan", LaserScan, self.scanCallback)
        #rospy.Subscriber("move_base/status",GoalStatusArray, self.goalStatusCallback)
        
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.loginfo('explorer online')
        pass

    def CheckIfHaveFirstData(self):
        return self.last_map and self.last_map_metadata #and self.last_pose

    def AddInplaceRotationsToQueue(self):
        import math
        import numpy
        last_x_y_yaw = (1,1,0)
        x,y,yaw = last_x_y_yaw
        for yaw_target in [math.pi/2,math.pi,-math.pi/2,0]:#numpy.linspace(yaw,yaw-2*math.pi,10):
            self.goal_queue.append((x,y,yaw_target))
            pass
        pass
    
    
        
    
    # def poseCallback(self,poseData):
    #     rospy.loginfo(rospy.get_caller_id()+" pose received: {}".format(poseData))
    #     self.last_pose = poseData
    #     print("pose received: {}".format(poseData))
    #     pass

    def mapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_map = occupancyGridData
        self.last_map_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(self.last_map)
        
        pass

    def PlotMap(self):
        assert self.last_map
        import matplotlib.pyplot as plt
        resolution = self.last_map.info.resolution
        zero_position_x = self.last_map.info.pose.position.x
        zero_position_y = self.last_map.info.pose.position.y
        max_position_x = self.last_map.info.width*resolution
        max_position_y = self.last_map.info.height*resolution
        print self.last_map_metadata.info #: 0.0500000007451
        print "map.shape = ",self.last_map_numpy.shape
        plt.imshow(self.last_map_numpy,extent=(left,right,bottom,top)) # working on this here
        plt.show()
        
    def mapMetaDataCallback(self,mapMetaData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_map_metadata = mapMetaData
        pass

    # def scanCallback(self,scanData):
    #     #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
    #     #rospy.loginfo(rospy.get_caller_id()+" scan received.")
    #     self.last_scan = scanData
    #     pass
    
    def PublishGoal(self,x,y,yaw):
        rospy.loginfo(rospy.get_caller_id()+" requesting to move to {}".format((x,y,yaw)))
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()
        pass
        

    def Step(self):
        print "Mode=",self.mode
        if self.CheckIfHaveFirstData():
            self.PlotMap()
            if self.mode is None:
                self.goal_queue.append((0,0,0))
                self.AddInplaceRotationsToQueue()
                self.mode = "Rotating"
                x,y,yaw = self.goal_queue.pop(0)
                self.PublishGoal(x,y,yaw)
                pass
            if self.goal_queue:
                x,y,yaw = self.goal_queue.pop(0)
                self.PublishGoal(x,y,yaw)
                pass
            else:
                return True
            pass
        else:
            print "NoMapScan"
            pass
        if self.abort:
            return True
        return False
    pass

def explorer():
    # send command to turn around, to get an initial map

    brain = ME134_Explorer()
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        done = brain.Step()
        if done:
            break
        rate.sleep()
        pass
    pass

def findGoal(poseData, mapData):
    nextGoal = PoseStamped()
    nextGoal.header.frame_id = "/map"
    nextGoal.header.stamp = rospy.Time.now()
    nextGoal.pose.position.z = 0.0
    nextGoal.pose.position.x = 1.0 #change this
    nextGoal.pose.position.y = 2.0 # change this
    nextGoal.pose.orientation.w = 1.0
    return nextGoal


if __name__ == '__main__':
    try:
        explorer()
    except rospy.ROSInterruptException:
        pass
