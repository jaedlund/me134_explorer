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
import numpy

class ME134_Explorer:
    def __init__(self):
        self.last_goal = None

        self.last_map = None
        self.last_map_numpy = None
        self.last_map_extents = None
        
        self.last_global_costmap = None
        self.last_global_costmap_numpy = None
        self.last_global_costmap_extents = None

        self.last_pose = None # this will be a tuple of (x position, y position, yaw angle)
        self.abort = False
        self.mode = None

        self.goal_queue = []

        # Subscribe to all the necessary topics to get messages as they come in.
        rospy.init_node('me134_explorer', anonymous=False)
        #rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.poseCallback)
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        rospy.Subscriber("move_base/global_costmap/costmap", OccupancyGrid, self.globalCostMapCallback)
        #rospy.Subscriber("scan", LaserScan, self.scanCallback)
        #rospy.Subscriber("move_base/status",GoalStatusArray, self.goalStatusCallback)
        rospy.loginfo('Subscribed to map, move_base/global_costmap/costmap topics')
        
        # Create a transform buffer (buffers messages for up to 10 seconds), and a listener to recieve tf2 messages from it. 
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        #self.tfListener.callback(self.poseCallback) # this line doesn't work

        # all_frames_as_string is useful for debugging, to see what transforms exist. 
        # It needs the sleep call because it takes a moment to start up
        #rospy.Rate(10).sleep()
        #rospy.loginfo('frames:'+ self.tfBuffer.all_frames_as_string())
        
        #self.pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.loginfo('explorer online')
        self.overlay_colors={1:(0,255,0),}
        #image[m==-1] = (150,150,150)
        #image[m==100] = (0,0,0)
        #image[m==0] = (255,255,255)
        pass

    def CheckIfHaveFirstData(self):
        return self.last_map and self.last_pose and self.last_global_costmap #and self.last_map_metadata 

    def AddInplaceRotationsToQueue(self):
        import math
        import numpy
        x,y,yaw = self.last_pose
        for yaw_target in [math.pi/2,math.pi,-math.pi/2,0]:#numpy.linspace(yaw,yaw-2*math.pi,10):
            self.goal_queue.append((x,y,yaw+yaw_target))
            pass
        pass
    
    
    def getLastPose(self):
        try:
            # Check the tf buffer for the latest transform
            # tfmsg is of type geometry_msgs/TransformStamped: http://docs.ros.org/api/geometry_msgs/html/msg/TransformStamped.html
            tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            # Note: with a 4th argument, lookup_transform blocks until tranform recieved or until timeout
            #tfmsg = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
            
            # parse info from TransformStamped message
            #rospy.loginfo('map to base_link transform: '+ str(tfmsg))
            header = tfmsg.header
            trans = tfmsg.transform.translation
            orient = tfmsg.transform.rotation
            
            # Create PoseStamped message from tfmsg. We are assuming here that map frame is at (0,0)                
            #position = Point(translation.x,translation.y,translation.z)
            #PSmsg = PoseStamped(header, Pose(position, orientation))
            #rospy.loginfo('PoseStamped message: '+str(self.last_pose))

            # Note: apparently the geometry_msgs quaternion is not the same as the tf.transformations quaternion,
            # so we need to do this explicitly rather than simply using euler_from_quaternion(orientation)
            [pitch,roll,yaw] = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
            self.last_pose = (trans.x, trans.y, yaw)
            rospy.loginfo('Pose: '+str(self.last_pose))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False
        return True
    

    def mapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_map = occupancyGridData
        # occupancygrid_to_numpy returns a masked numpy array, we use the .data to remove the mask.
        self.last_map_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancyGridData).data
        #self.last_map_from_above = numpy.flipud(self.last_map_numpy)
        
        resolution = occupancyGridData.info.resolution
        left = occupancyGridData.info.origin.position.x
        right = left + occupancyGridData.info.width*resolution
        bottom = occupancyGridData.info.origin.position.y
        top = bottom + occupancyGridData.info.height*resolution
        self.last_map_extents = (left,right,bottom,top) 

        pass

    def globalCostMapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_global_costmap = occupancyGridData
        # occupancygrid_to_numpy returns a masked numpy array, we use the .data to remove the mask.
        self.last_global_costmap_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancyGridData).data
        #self.last_map_from_above = numpy.flipud(self.last_map_numpy)
        
        resolution = occupancyGridData.info.resolution
        left = occupancyGridData.info.origin.position.x
        right = left + occupancyGridData.info.width*resolution
        bottom = occupancyGridData.info.origin.position.y
        top = bottom + occupancyGridData.info.height*resolution
        self.last_global_costmap_extents = (left,right,bottom,top)
    
        pass
    
    
    
    def PlotMap(self, overlay=None, overlay_colors=None):
        self.PlotGeneralMap(self.last_map_numpy,self.last_map_extents,overlay=overlay,overlay_colors=overlay_colors)
        pass
    
    def PlotGeneralMap(self,map_data,map_extents,overlay=None,overlay_colors=None):
        assert self.last_map
        import matplotlib.pyplot as plt
                
        #plt.imshow(self.last_map_from_above,extent=self.last_map_from_above_extents)
        fig,ax = plt.subplots()
        m = map_data
        image = numpy.zeros((m.shape[0],m.shape[1],3),dtype=numpy.uint8)
        print set(m.flatten().tolist())
        image[m==-1] = (150,150,150)
        image[m==100] = (0,0,0)
        image[m==0] = (255,255,255)
        if overlay is not None:
            assert overlay_colors
            for value,color in overlay_colors.iteritems():
                image[overlay==value] = color
                pass
            pass
        ax.imshow(image,extent=map_extents,origin='lower')
        ax.autoscale(tight=True)
        #x,y,yaw= self.Get_x_y_yaw()
        x,y,yaw= self.last_pose
        ax.plot(x,y,'o') # TODO: plot robot pointing direction 
        plt.show()
    
    
    
    def PlotCostMap(self,overlay=None,overlay_colors=None):
        assert self.last_map
        import matplotlib.pyplot as plt
                
        #plt.imshow(self.last_map_from_above,extent=self.last_map_from_above_extents)
        fig,ax = plt.subplots()
        m = self.last_global_costmap_numpy
        m_extents = self.last_global_costmap_extents
        #image = numpy.zeros((self.last_map_numpy.shape[0],self.last_map_numpy.shape[1],3),dtype=numpy.uint8)
        #print set(m.flatten().tolist())
        #image[m==-1] = (150,150,150)
        #image[m==100] = (0,0,0)
        #image[m==0] = (255,255,255)
        #if overlay is not None:
        #    assert overlay_colors
        #    for value,color in overlay_colors.iteritems():
        #        image[overlay==value] = color
        #        pass
        #    pass
        ax.imshow(m,extent=m_extents,origin='lower')
        ax.autoscale(tight=True)
        #x,y,yaw= self.Get_x_y_yaw()
        x,y,yaw= 0,0,0
        ax.plot(x,y,'o') # TODO: 
        plt.show()
        1/0

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

    def FindFrontier(self,m):
        free_space_indices = numpy.where(m==0)
        print "free_space_indices=",free_space_indices
        # Brute force way
        frontier_indices = []
        for (a,b) in zip(*free_space_indices):
            for off_a in (-1,0,1):
                for off_b in (-1,0,1):
                    if off_a == 0 and off_b == 0:
                        continue
                    a_new,b_new = a+off_a,b+off_b
                    if 0<=a_new<m.shape[0] and 0<=b_new<m.shape[1]:
                        if m[a_new,b_new] == -1:
                            frontier_indices.append((a,b))
                            pass
                        pass
                    else:
                        print "WARNING: free space at map edge"
                        pass
                    pass
                pass
            pass
        return frontier_indices

    def ProcessMap(self):
        self.frontier_indices = self.FindFrontier(self.last_map_numpy)
        self.overlay = numpy.zeros(self.last_map_numpy.shape)
        for (a,b) in self.frontier_indices:
            self.overlay[a,b]=1
            pass
        

    def Step(self):
        print "Mode=",self.mode

        self.getLastPose()
        if self.CheckIfHaveFirstData():
            self.ProcessMap()
            self.PlotCostMap()
            self.PlotMap(overlay=self.overlay,overlay_colors=self.overlay_colors)
            if self.mode is None: # in the beginning
                #self.goal_queue.append((0,0,0))
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
