#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import tf2_ros
import ros_numpy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#from std_msgs.msg import Int8MultiArray


class ME134_Explorer:
    def __init__(self):
        self.last_goal = None
        self.last_map = None
        self.last_map_metadata = None
        self.last_scan = None # this will be a tuple of (x position, y position, yaw angle)
        self.last_pose = None
        self.abort = False
        self.mode = None

        self.goal_queue = []

        # Subscribe to all the necessary topics to get messages as they come in.
        rospy.init_node('me134_explorer', anonymous=False)
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        #rospy.Subscriber("scan", LaserScan, self.scanCallback)
        #rospy.Subscriber("move_base/status",GoalStatusArray, self.goalStatusCallback)
        rospy.loginfo('Subscribed to map, scan, move_base/status topics')
        
        # Create a transform buffer (buffers messages for up to 10 seconds), and a listener to recieve tf2 messages from it. 
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        #self.tfListener.callback(self.poseCallback) # this line doesn't work

        # all_frames_as_string is useful for debugging, to see what transforms exist. 
        # It needs the sleep call because it takes a moment to start up
        #rospy.Rate(10).sleep()
        #rospy.loginfo('frames:'+ self.tfBuffer.all_frames_as_string())
        
        self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.loginfo('explorer online')
        pass

    def CheckIfHaveFirstData(self):
        return self.last_map and self.last_map_metadata and self.last_pose

    def AddInplaceRotationsToQueue(self):
        import math
        import numpy
        last_x_y_yaw = (1,1,0)
        x,y,yaw = last_x_y_yaw
        for yaw_target in [math.pi/2,math.pi,-math.pi/2,0]:#numpy.linspace(yaw,yaw-2*math.pi,10):
            self.goal_queue.append((x,y,yaw_target))
            pass
        pass
    
    
    def getLastPose(self):
        try:
            # Check the tf buffer for the latest transform
            # tfmsg is of type geometry_msgs/TransformStamped: http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
            tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            # Note: with a 4th argument, lookup_transform blocks until tranform recieved or until timeout
            #tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            
            rospy.loginfo('map to base_link transform: '+ str(tfmsg))
            # parse info from TransformStamped message
            header = tfmsg.header
            translation = tfmsg.transform.translation
            orientation = tfmsg.transform.rotation
            
            # Create PoseStamped message from tfmsg. We are assuming here that map frame is at (0,0)                
            #position = Point(translation.x,translation.y,translation.z)
            #PSmsg = PoseStamped(header, Pose(position, orientation))
            #rospy.loginfo('PoseStamped message: '+str(self.last_pose))

            [pitch,roll,yaw] = tf.euler_from_quaternion(orientation)
            self.last_pose = (translation.x, translation.y, yaw)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return false

         #rospy.loginfo(rospy.get_caller_id()+" pose received: {}".format(poseData))
         #self.last_pose = poseData
         #print("pose received: {}".format(poseData))
        pass        
    

    def mapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_map = occupancyGridData
        self.last_map_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(self.last_map)
        #self.last_map_from_above = numpy.flipud(self.last_map_numpy)
        
        resolution = self.last_map.info.resolution
        left = self.last_map.info.origin.position.x
        right = left + self.last_map.info.width*resolution
        bottom = self.last_map.info.origin.position.y
        top = bottom + self.last_map.info.height*resolution
        self.last_map_extents = (left,right,bottom,top) 

        pass

    def PlotMap(self):
        assert self.last_map
        import matplotlib.pyplot as plt
                
        #plt.imshow(self.last_map_from_above,extent=self.last_map_from_above_extents)
        fig,ax = plt.subplots()
    
        ax.imshow(self.last_map_numpy,extent=self.last_map_extents,origin='lower')
        #x,y,yaw= self.Get_x_y_yaw()
        x,y,yaw= 0,0,0
        ax.plot(x,y,'o')
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

        self.getLastPose()
        if self.CheckIfHaveFirstData():
            self.PlotMap()
            if self.mode is None: # in the beginning
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
