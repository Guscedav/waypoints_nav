#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PoseStamped
from std_msgs.msg import Empty
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time
import os
import roslaunch

# change Pose to the correct frame
def changePose(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()


#Path for saving and retreiving the pose.csv file
output_file_path = rospkg.RosPack().get_path('waypoints_nav')+"/saved_path/pose.csv"
waypoints = []

class Init(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed'])

    def execute(self, userdata):
        rospy.set_param('~nav_dir_topic', 0)
        rospy.set_param('~start_tracking_topic', 0)
        
        return 'completed'

class Operations(State):
    def __init__(self):
        State.__init__(self, outcomes=['getTrack', 'goStart', 'goEnd'])
        self.start_tracking_topic = 0
        self.nav_dir_topic = 0

    def execute(self, userdata):
        # Start thread to listen for get track messages to clear the waypoint queue
        def wait_for_start_tracker():
            """thread worker function"""
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/start_tracker', Empty)
                self.start_tracking_topic = 1
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
                self.start_tracking_topic = 0
        reset_thread = threading.Thread(target=wait_for_start_tracker)
        reset_thread.start()

        # Start thread to listen for get nav messages to clear the waypoint queue
        def wait_for_go_start():
            """thread worker function"""
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/go_start', Empty)

                self.nav_dir_topic = 1
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
                self.nav_dir_topic = 0
        reset_thread = threading.Thread(target=wait_for_go_start)
        reset_thread.start()

        # Start thread to listen for get nav messages to clear the waypoint queue
        def wait_for_go_end():
            """thread worker function"""
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/go_end', Empty)
                
                self.nav_dir_topic = 2
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
                self.nav_dir_topic = 0
        reset_thread = threading.Thread(target=wait_for_go_end)
        reset_thread.start()


        loop_rate = rospy.Rate(5) # 5 Hz Rate
        while not rospy.is_shutdown():
            if self.start_tracking_topic == 1: #MAYBE have to ADD THREAD TO LISTEN TO THE MSGS
                return 'getTrack'
            elif self.nav_dir_topic == 1:
                return 'goStart'
            elif self.nav_dir_topic == 2:
                return 'goEnd'
            else:
                loop_rate.sleep()
        return 'completed'

class GetTrack(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed'])
        # Subscribe to pose message to get new waypoints
        self.addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        # Start thread to listen for reset messages to clear the waypoint queue
        def wait_for_path_reset():
            """thread worker function"""
            global waypoints
            while not rospy.is_shutdown():
                data = rospy.wait_for_message('/path_reset', Empty)
                rospy.loginfo('Received path RESET message')
                self.initialize_path_queue()
                rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                               # for three seconds and wait_for_message() in a
                               # loop will see it again.
        reset_thread = threading.Thread(target=wait_for_path_reset)
        reset_thread.start()

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/stop_tracker', Empty)
            self.path_ready = True
            with open(output_file_path, 'w') as file:
                for current_pose in waypoints:
                    file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
            rospy.loginfo('poses written to '+ output_file_path)
        ready_thread = threading.Thread(target=wait_for_path_ready)
        ready_thread.start()

        topic = self.addpose_topic

        # Wait for published waypoints or saved path  loaded
        while (not self.path_ready):
            try:
                pose_tmp = rospy.wait_for_message(topic, PoseStamped, timeout=1)
            except rospy.ROSException as e:
            	continue
                # if 'timeout exceeded' in e.message:
                    # continue  # no new waypoint within timeout, looping...
                # else:
                    # raise e
            
            #Convert PoseStamped to PoseWithCovariance Stamped
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = pose_tmp.header.frame_id
            pose.pose.pose.position.x     =    pose_tmp.pose.position.x
            pose.pose.pose.position.y     =    pose_tmp.pose.position.y
            pose.pose.pose.position.z     =    pose_tmp.pose.position.z
            pose.pose.pose.orientation.x = pose_tmp.pose.orientation.x
            pose.pose.pose.orientation.y = pose_tmp.pose.orientation.y
            pose.pose.pose.orientation.z = pose_tmp.pose.orientation.z
            pose.pose.pose.orientation.w = pose_tmp.pose.orientation.w

            rospy.loginfo("Received new waypoint")
            waypoints.append(changePose(pose, "odom"))
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        return 'completed'

class GoStart(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

    def execute(self, userdata):
        global waypoints
        waypoints = []

        with open(output_file_path, 'r') as file:
                reader = csv.reader(file, delimiter = ',')
                for row in reader:
                    print (row)
                    current_pose = PoseWithCovarianceStamped()
                    current_pose.pose.pose.position.x     =    float(row[0])
                    current_pose.pose.pose.position.y     =    float(row[1])
                    current_pose.pose.pose.position.z     =    float(row[2])
                    current_pose.pose.pose.orientation.x = float(row[3])
                    current_pose.pose.pose.orientation.y = float(row[4])
                    current_pose.pose.pose.orientation.z = float(row[6])
                    current_pose.pose.pose.orientation.w = -float(row[5])
                    waypoints.insert(0, current_pose)
                    self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
        self.start_journey_bool = True

        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                #This is the loop which exist when the robot is near a certain GOAL point.
                distance = 10
                while(distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
                    distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))

        return 'completed'

class GoEnd(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

    def execute(self, userdata):
        global waypoints
        waypoints = []

        with open(output_file_path, 'r') as file:
                reader = csv.reader(file, delimiter = ',')
                for row in reader:
                    print (row)
                    current_pose = PoseWithCovarianceStamped()
                    current_pose.pose.pose.position.x     =    float(row[0])
                    current_pose.pose.pose.position.y     =    float(row[1])
                    current_pose.pose.pose.position.z     =    float(row[2])
                    current_pose.pose.pose.orientation.x = float(row[3])
                    current_pose.pose.pose.orientation.y = float(row[4])
                    current_pose.pose.pose.orientation.z = float(row[5])
                    current_pose.pose.pose.orientation.w = float(row[6])
                    waypoints.append(current_pose)
                    self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
        self.start_journey_bool = True

        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            if not self.distance_tolerance > 0.0:
                self.client.wait_for_result()
                rospy.loginfo("Waiting for %f sec..." % self.duration)
                time.sleep(self.duration)
            else:
                #This is the loop which exist when the robot is near a certain GOAL point.
                distance = 10
                while(distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
                    distance = math.sqrt(pow(waypoint.pose.pose.position.x-trans[0],2)+pow(waypoint.pose.pose.position.y-trans[1],2))

        return 'completed'

class EndNav(State):
    def __init__(self):
        State.__init__(self, outcomes=['completed'])

    def execute(self, userdata):
        rospy.loginfo('######### NAV ENDED #########')

        # Thread which closes the Navigation node
        #def close_nav_node():
            # os.system('rosnode kill /move_base') #Kill the Move_Base node
        #closing_thread = threading.Thread(target=close_nav_node)
        #closing_thread.start()

        return 'completed'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

def main():
    rospy.init_node('waypoints_nav')

    sm = StateMachine(outcomes=['completed'])

    with sm:
        StateMachine.add('INIT', Init(),
                           transitions={'completed':'OPERATIONS'})
        StateMachine.add('OPERATIONS', Operations(),
                           transitions={'getTrack':'GET_TRACK',
                                        'goStart':'GO_START',
                                        'goEnd':'GO_END'})
        StateMachine.add('GET_TRACK', GetTrack(),
                           transitions={'completed':'OPERATIONS'})
        StateMachine.add('GO_START', GoStart(),
                           transitions={'completed':'END_NAV'})
        StateMachine.add('GO_END', GoEnd(),
                           transitions={'completed':'END_NAV'})
        StateMachine.add('END_NAV', EndNav(),
                           transitions={'completed':'OPERATIONS'})

    outcome = sm.execute()
