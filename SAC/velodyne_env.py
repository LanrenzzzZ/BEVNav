import rospy
import subprocess
from os import path
import torch
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from numpy import inf
import numpy as np
import random
import math
from gazebo_msgs.msg import ModelState, ModelStates
from squaternion import Quaternion
from geometry_msgs.msg import Twist,Pose
from sensor_msgs.msg import LaserScan, PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import os
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
from skimage.transform import resize
from collections import deque
from point_sample import regularize_pc, crop_pc
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
np.set_printoptions(threshold=np.inf)


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goalOK = True

    if -3.8 > x > -6.2 and 6.2 > y > 3.8:
        goalOK = False

    if -1.3 > x > -2.7 and 4.7 > y > -0.2:
        goalOK = False

    if -0.3 > x > -4.2 and 2.7 > y > 1.3:
        goalOK = False

    if -0.8 > x > -4.2 and -2.3 > y > -4.2:
        goalOK = False

    if -1.3 > x > -3.7 and -0.8 > y > -2.7:
        goalOK = False

    if 4.2 > x > 0.8 and -1.8 > y > -3.2:
        goalOK = False

    if 4 > x > 2.5 and 0.7 > y > -3.2:
        goalOK = False

    if 6.2 > x > 3.8 and -3.3 > y > -4.2:
        goalOK = False

    if 4.2 > x > 1.3 and 3.7 > y > 1.5:
        goalOK = False

    if -3.0 > x > -7.2 and 0.5 > y > -1.5:
        goalOK = False

    if x > 4.5 or x < -4.5 or y > 4.5 or y < -4.5:
        goalOK = False
    return goalOK


# Function to put the laser data in bins
def binning(lower_bound, data, quantity):
    width = round(len(data) / quantity)
    quantity -= 1
    bins = []
    for low in range(lower_bound, lower_bound + quantity * width + 1, width):
        bins.append(min(data[low:low + width]))
    return np.array([bins])


class GazeboEnv:
    """Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, launchfile, height, width, nchannels):

        self.odomX = 0
        self.odomY = 0

        self.goalX = 1
        self.goalY = 0.0

        self.upper = 5.0
        self.lower = -5.0
        self.velodyne_data = np.ones(20) * 10

        self.set_self_state = ModelState()
        self.set_self_state.model_name = 'r1'
        self.set_self_state.pose.position.x = 0.
        self.set_self_state.pose.position.y = 0.
        self.set_self_state.pose.position.z = 0.
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
        self.gaps = [[-1.6, -1.57 + 3.14 / 20]]
        for m in range(19):
            self.gaps.append([self.gaps[m][1], self.gaps[m][1] + 3.14 / 20])
        self.gaps[-1][-1] += 0.03

        port = '11311'
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node('gym', anonymous=True)
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        self.gzclient_pid = 0

        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher('/r1/cmd_vel', Twist, queue_size=1)
        self.set_state = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        topic = 'vis_mark_array'
        self.publisher = rospy.Publisher(topic, MarkerArray, queue_size=3)
        topic2 = 'vis_mark_array2'
        self.publisher2 = rospy.Publisher(topic2, MarkerArray, queue_size=1)
        topic3 = 'vis_mark_array3'
        self.publisher3 = rospy.Publisher(topic3, MarkerArray, queue_size=1)
        topic4 = 'vis_mark_array4'
        self.publisher4 = rospy.Publisher(topic4, MarkerArray, queue_size=1)
        self.velodyne = rospy.Subscriber('/velodyne_points', PointCloud2, self.velodyne_callback, queue_size=10)
        self.laser = rospy.Subscriber('/r1/front_laser/scan', LaserScan, self.laser_callback, queue_size=10)
        self.odom = rospy.Subscriber('/r1/odom', Odometry, self.odom_callback, queue_size=10)
        self.image = rospy.Subscriber('/d435/color/image_raw', Image, self.image_callback, queue_size=10)
        self.depth_image_msg = rospy.Subscriber('/d435/depth/image_raw', Image, self.dapth_image_callback, queue_size=10)
        self.pointcloud = rospy.Subscriber('/d435/depth/color/points', PointCloud2, self.pointcloud_callback,
                                           queue_size=1)
        self.bridge = CvBridge()
        self._frames = deque([], maxlen=3)
    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, v):
        self.laser_pcd = v

    def laser_callback(self, scan):
        self.last_laser = scan

    def odom_callback(self, od_data):
        self.last_odom = od_data

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.image = cv2.resize(cv_image.copy(), (100, 100), interpolation = cv2.INTER_AREA)  # [100, 100 , 3]
        except CvBridgeError as e:
            print(e)

    def dapth_image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            nan_location = np.isnan(cv_image)
            cv_image[nan_location] = np.nanmax(cv_image)
            norm_image = (cv_image) * 255. / 5.
            norm_image[0, 0] = 255.
            norm_image = norm_image.astype('uint8')
            self.cv_image = cv2.resize(norm_image.copy(), (100, 100), interpolation=cv2.INTER_AREA) # [84, 84]
            self.cv_image = self.cv_image.reshape(1, 100, 100)
        except CvBridgeError as e:
            print(e)

    def pointcloud_callback(self, data):
        self.rgbd_pcd = data

    # Detect a collision from laser data
    def calculate_observation(self, data):
        min_range = 0.2
        min_laser = 2
        done = False
        col = False

        for i, item in enumerate(data.ranges):
            if min_laser > data.ranges[i]:
                min_laser = data.ranges[i]
            if (min_range > data.ranges[i] > 0):
                done = True
                col = True
        return done, col, min_laser

    # Perform an action and read a new state
    def step(self, act):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = act[0]
        vel_cmd.angular.z = act[1]
        self.vel_pub.publish(vel_cmd)

        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        data = self.last_laser
        dataOdom = self.last_odom

        # rgbd_pcd = None
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # try:
        #     self.unpause()
        # except (rospy.ServiceException) as e:
        #     print("/gazebo/unpause_physics service call failed")
        # while rgbd_pcd is None:
        #     try:
        #         rgbd_pcd = rospy.wait_for_message('/d435/depth/color/points', Image, timeout=0.5)
        #     except:
        #         pass
        laser_pcd = None
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")
        while laser_pcd is None:
            try:
                laser_pcd = rospy.wait_for_message('/velodyne_points', Image, timeout=0.5)
            except:
                pass

        laser_pcd = np.array(list(pc2.read_points(laser_pcd, skip_nans=True, field_names=("x", "y", "z"))))
        pcd = crop_pc(laser_pcd, 1000)
        pcd = np.array(pcd, dtype=np.float16)

        done, col, min_laser = self.calculate_observation(data)

        time.sleep(0.1)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # Calculate robot heading from odometry data
        self.odomX = dataOdom.pose.pose.position.x
        self.odomY = dataOdom.pose.pose.position.y
        quaternion = Quaternion(
            dataOdom.pose.pose.orientation.w,
            dataOdom.pose.pose.orientation.x,
            dataOdom.pose.pose.orientation.y,
            dataOdom.pose.pose.orientation.z)
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        # Calculate the angle distance between the robots heading and heading toward the goal
        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY
        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
        beta2 = (beta - angle)
        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = act[0]
        vel_cmd.angular.z = act[1]
        self.vel_pub.publish(vel_cmd)

        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goalX
        marker.pose.position.y = self.goalY
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "base_link"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(act[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "base_link"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(act[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

        markerArray4 = MarkerArray()
        marker4 = Marker()
        marker4.header.frame_id = "base_link"
        marker4.type = marker.CUBE
        marker4.action = marker.ADD
        marker4.scale.x = 0.1  
        marker4.scale.y = 0.1
        marker4.scale.z = 0.01
        marker4.color.a = 1.0
        marker4.color.r = 1.0
        marker4.color.g = 0.0
        marker4.color.b = 0.0
        marker4.pose.orientation.w = 1.0
        marker4.pose.position.x = 5
        marker4.pose.position.y = 0.4
        marker4.pose.position.z = 0

        markerArray4.markers.append(marker4)
        self.publisher4.publish(markerArray4)

        '''Bunch of different ways to generate the reward'''
        reward = act[0] / 2 - abs(act[1]) / 2 + self.distOld - Dist


        self.distOld = Dist

        # Detect if the goal has been reached and give a large positive reward
        if Dist < 0.3:
            target = True
            done = True
            self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
            reward = 80

        # Detect if ta collision has happened and give a large negative reward
        if col:
            reward = -100

        toGoal = [Dist, beta2, act[0], act[1]]

        return toGoal, pcd, reward, done, target

    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0., 0., angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        chk = False
        while not chk:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            chk = check_pos(x, y)
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        # object_state.pose.position.z = 0.
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)

        self.odomX = object_state.pose.position.x
        self.odomY = object_state.pose.position.y

        self.change_goal()
        self.random_box()
        self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        data = None
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")
        while data is None:
            try:
                data = rospy.wait_for_message('/r1/front_laser/scan', LaserScan, timeout=0.5)
            except:
                pass
        # rgbd_pcd = None
        # rospy.wait_for_service('/gazebo/unpause_physics')
        # try:
        #     self.unpause()
        # except (rospy.ServiceException) as e:
        #     print("/gazebo/unpause_physics service call failed")
        # while rgbd_pcd is None:
        #     try:
        #         rgbd_pcd = rospy.wait_for_message('/d435/depth/color/points', Image, timeout=0.5)
        #     except:
        #         pass
        laser_pcd = None
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")
        while laser_pcd is None:
            try:
                laser_pcd = rospy.wait_for_message('/velodyne_points', Image, timeout=0.5)
            except:
                pass

        laser_pcd = np.array(list(pc2.read_points(laser_pcd, skip_nans=True, field_names=("x", "y", "z"))))
        pcd = crop_pc(laser_pcd, 1000)
        pcd = np.array(pcd, dtype=np.float16)
        # for _ in range(3):
        #     self._frames.append(pcd)
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY

        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
        beta2 = (beta - angle)

        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        toGoal = [Dist, beta2, 0.0, 0.0]

        return toGoal, pcd

    # Place a new goal and check if its lov\cation is not on one of the obstacles
    def change_goal(self):
        if self.upper < 10:
            self.upper += 0.004
        if self.lower > -10:
            self.lower -= 0.004
        gOK = False
        while not gOK:
            self.goalX = self.odomX + random.uniform(self.upper, self.lower)
            self.goalY = self.odomY + random.uniform(self.upper, self.lower)
            gOK = check_pos(self.goalX, self.goalY)

    # Randomly change the location of the boxes in the environment on each reset to randomize the training environment
    def random_box(self):
        for i in range(4):
            name = 'cardboard_box_' + str(i)
            x = 0
            y = 0
            chk = False
            while not chk:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                chk = check_pos(x, y)
                d1 = math.sqrt((x - self.odomX) ** 2 + (y - self.odomY) ** 2)
                d2 = math.sqrt((x - self.goalX) ** 2 + (y - self.goalY) ** 2)
                if d1 < 1.5 or d2 < 1.5:
                    chk = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def _get_obs(self):
        assert len(self._frames) == 3
        return np.concatenate(list(self._frames), axis=0)
