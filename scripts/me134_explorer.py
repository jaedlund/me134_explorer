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
import math
import matplotlib.pyplot as plt


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
        self.next_mode = None

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
        self.overlay_colors={1:(0,255,0),} # frontier color green
        pass

    def CheckIfHaveFirstData(self):
        return self.last_map and self.last_pose and self.last_global_costmap #and self.last_map_metadata 

    # This works, but you might find a better way
    def AddInplaceRotationsToQueue(self):
        x,y,yaw = self.last_pose
        for yaw_target in [math.pi/2,math.pi,-math.pi/2,0]:
            self.goal_queue.append((x,y,yaw_target))
            pass
        pass

    def AddMoveForwardToQueue(self,distance):
        x,y,yaw = self.last_pose
        x_new = x + distance*math.cos(yaw)
        y_new = y + distance*math.sin(yaw)
        self.goal_queue.append((x_new,y_new,yaw))
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
        self.last_map_extents = (left,right,bottom,top) # for plotting

        pass

    def globalCostMapCallback(self,occupancyGridData):
        #rospy.loginfo(rospy.get_caller_id()+"map received: {}".format(occupancyGridData))
        #rospy.loginfo(rospy.get_caller_id()+" map received.")
        self.last_global_costmap = occupancyGridData
        # occupancygrid_to_numpy returns a masked numpy array, we use the .data to remove the mask.
        self.last_global_costmap_numpy = ros_numpy.occupancy_grid.occupancygrid_to_numpy(occupancyGridData)
        #self.last_map_from_above = numpy.flipud(self.last_map_numpy)
        
        resolution = occupancyGridData.info.resolution
        left = occupancyGridData.info.origin.position.x
        right = left + occupancyGridData.info.width*resolution
        bottom = occupancyGridData.info.origin.position.y
        top = bottom + occupancyGridData.info.height*resolution
        self.last_global_costmap_extents = (left,right,bottom,top)
    
        pass
        
    def PlotMap(self,overlay=None,overlay_colors=None):
        assert self.last_map
        map_data = self.last_map_numpy
        map_extents = self.last_map_extents
        timestamp = self.last_map.info.map_load_time
        
        fig,ax = plt.subplots()
        m = map_data
        image = numpy.zeros((m.shape[0],m.shape[1],3),dtype=numpy.uint8)
        # useful for debugging. Because numbers besides -1,0,or 100 won't show up on the map.
        #print set(m.flatten().tolist()) 
        image[m==-1] = (150,150,150)
        image[m==100] = (0,0,0)
        image[m==0] = (255,255,255)
        # you can set your own overlay colors
        if overlay is not None:
            assert overlay_colors
            for value,color in overlay_colors.iteritems():
                image[overlay==value] = color
                pass
            pass
        ax.imshow(image,extent=map_extents,origin='lower',interpolation='none')
        ax.autoscale(tight=True)
        ax.set_xlabel("x position (m)")
        ax.set_ylabel("y position (m)")
        ax.set_title("map")
        x,y,yaw= self.last_pose
        ax.plot(x,y,'o', label="robot") # TODO: plot robot pointing direction 
        if self.goal_queue: # If non-empty goal queue, plot first goal in the queue
            x,y,yaw= self.goal_queue[0]
            ax.plot(x,y,'x',label="next goal")
            pass
        fig.suptitle("mode={} Close plots to proceed".format(self.mode))
        ax.legend(loc='best', fancybox=True, framealpha=0.5)

        return fig

    # plots the most recent costmap (from subscribed topic)
    def PlotCostMap(self):
        assert self.last_global_costmap
                
        #plt.imshow(self.last_map_from_above,extent=self.last_map_from_above_extents)
        fig,ax = plt.subplots()
        m = self.last_global_costmap_numpy
        m_extents = self.last_global_costmap_extents
        ax.imshow(m,extent=m_extents,origin='lower',cmap='Oranges',interpolation='none')
        ax.autoscale(tight=True)
        ax.set_xlabel("x position (m)")
        ax.set_ylabel("y position (m)")
        ax.set_title("global cost map (sometimes doesn't update)")
        fig.suptitle("mode={} Close plots to proceed".format(self.mode))
        #x,y,yaw= self.Get_x_y_yaw()
        x,y,yaw= self.last_pose
        ax.plot(x,y,'o',label="robot") 
        if self.goal_queue:
            x,y,yaw= self.goal_queue[0]
            ax.plot(x,y,'x',label="next goal")
            pass
        ax.legend(loc='best', fancybox=True, framealpha=0.5)
        return fig

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

    # returns indices of frontiers
    def FindFrontier(self,m):
        free_space_indices = numpy.where(m==0)
        #print "free_space_indices=",free_space_indices
        # Brute force way
        frontier_indices = []
        for (ia,ib) in zip(*free_space_indices): # iterates through all free spaces
            has_unknown=False
            has_free=False
            for off_ia,off_ib in ((-1,0),(1,0),(0,-1),(0,1)): # frontier is a free space with at least one non-diagonal neighbor unknown and one non-diagonal neighbor free
                if has_unknown and has_free:
                    break
                ia_new,ib_new = ia+off_ia,ib+off_ib
                if 0<=ia_new<m.shape[0] and 0<=ib_new<m.shape[1]:
                    if m[ia_new,ib_new] == -1:
                        has_unknown=True
                        pass
                    if m[ia_new,ib_new] == 0:
                        has_free=True
                        pass
                    pass
                else:
                    print "WARNING: free space at map edge"
                    pass
                pass
            if has_unknown and has_free:
                frontier_indices.append((ia,ib))
                pass
            pass
        return frontier_indices

    def ConvertMapIndicesToMeters(self,ia,ib):
        info = self.last_map.info
        resolution = info.resolution
        x = info.origin.position.x + ib*resolution
        y = info.origin.position.y + ia*resolution
        return x,y

    # find the closest point on a frontier that is at least min_required_distance away
    # NOTE: If you are writing your own exploration algorithm, you will probably want to replace this method with your own
    def FindClosestFrontier(self,min_required_distance=None):
        rx,ry,r_yaw = self.last_pose
        min_distance = None
        target_x_y = None
        # Brute force approach: convert all frontier indices to distances
        for ia,ib in self.frontier_indices:
            tx,ty = self.ConvertMapIndicesToMeters(ia,ib)
            distance = math.sqrt((rx-tx)**2+(ry-ty)**2)
            if min_required_distance is not None and distance < min_required_distance:
                print "Skipping point {} with distance {} which might be under the robot".format((tx,ty),distance)
                continue
            if min_distance is None or distance < min_distance:
                min_distance = distance
                target_x_y = (tx,ty)
                pass
            pass
        print "Closest Frontier is at {}. It is {} meters from the robot.".format(target_x_y,min_distance)
        return target_x_y

    
    # builds frontiers and makes overlay (to plot frontiers on top of map)
    def ProcessMap(self):
        self.frontier_indices = self.FindFrontier(self.last_map_numpy)
        self.overlay = numpy.zeros(self.last_map_numpy.shape)
        for (ia,ib) in self.frontier_indices:
            self.overlay[ia,ib]=1 # mark as frontier
            pass
        

    def Step(self):
        print "Mode=",self.mode
        plotEachStep=True
        self.getLastPose()
        if self.CheckIfHaveFirstData():
            self.ProcessMap()
            print "goal_queue={}".format(self.goal_queue)
            if plotEachStep and (self.mode not in ["Rotating","Initial Movement"]): # self.mode - FSM markign current state
                fig1=self.PlotCostMap()
                fig2=self.PlotMap(overlay=self.overlay,overlay_colors=self.overlay_colors)
                plt.show()  # have to close windows for program to continue
                plt.close(fig1) # free figures so as to not run out of memory
                plt.close(fig2)
            if self.goal_queue: # if there's anythign in the goal queue, do it
                x,y,yaw = self.goal_queue.pop(0)
                self.PublishGoal(x,y,yaw)
                pass
            else: # once done with goal queue, move on to the next step, given by self.next_mode
                # Change the following code to run your algorithm
                # *------------------------------------------------*
                self.mode = self.next_mode
                
                if self.mode is None: # in the beginning
                    #self.goal_queue.append((0,0,0))
                    # There is a bug https://answers.ros.org/question/204740/dwa_local_planner-in-place-rotation-first-goal-does-not-work/
                    # That means that the first movement can't be a turn in place
                    # If we reverse a bit first the robot will rotate to clear the costmap
                    # After that, you can move robot in any way you want
                    # TODO: maybe you can find a better way to do this

                    self.AddMoveForwardToQueue(-0.25)
                    # Then return to the starting place
                    self.AddMoveForwardToQueue(0.25)
                    self.mode = "Initial Movement"
                    self.next_mode = "find_frontier"
                    #self.next_mode = "Rotating" # If you still have visible frontiers 
                    pass
                elif self.mode == "find_frontier":
                    self.next_mode = "Rotating"
                    # You should find a position on the map that views a frontier
                    # from a safe distance using the global_cost_map and/or
                    # another algorithm. Warning: global costmap doesn't always update regularly. 
                    # You can probably fix this if you want by subscribing to global_costmap_updates. 
                    # I'm only going to attempt to go to the closest one
                    # (probably not safe) for the demo.
                    target_x_y = self.FindClosestFrontier(min_required_distance=0.1) # otherwise robot will try to go to the  frontier directly under the robot - at least at first, the laser scanner won't scan that!
                    print("WARNING: Unless you change this code it won't drive anywhere. The goal is too close to a wall. You need to find a place to safely view this spot.")
                    
                    if target_x_y is not None:
                        tx,ty=target_x_y
                        # we're setting yaw=0 here. You may want to compute yaw angle, or choose it carefully...
                        self.goal_queue.append((tx,ty,0))
                        pass
                    else:
                        print "No Frontiers found"
                        self.abort = True
                        pass
                elif self.mode == "Rotating":
                    self.AddInplaceRotationsToQueue()
                    self.next_mode = "find_frontier"
                    pass
                else:
                    print "Unknown mode={}".format(self.mode)
                    return True
                # *-------------------------------------------------------------*
                pass
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
