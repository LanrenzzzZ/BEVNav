#!/usr/bin/env python
# coding=utf-8

import rospy
import thread,time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from move_base_msgs.msg import MoveBaseActionResult
from turn_on_wheeltec_robot.msg import supersonic
import os
from std_msgs.msg import Bool

#正前方4个超声波开始避障的距离，单位是m
start_avoid = 0

#左右两边超声波开始避障的距离，单位是m
start_avoidLR = 0
# 超声波自定义数据类型
dis = supersonic()

vel_msg = Twist()
forward_vel = Twist()

GoalID_=GoalID()

Goal=MoveBaseActionGoal()
# 左右转标志位、旋转小车寻找障碍物突破口相关变量
if_cmd_vel=True
TurnLeft = 0
TurnLeft2 = 0
TurnRight = 0
TurnRight2 = 0
Goal_Frame_id="map"
Goal_Position_x=0
Goal_Position_y=0
Goal_Position_z=0
Goal_Orientation_x=0
Goal_Orientation_y=0
Goal_Orientation_z=0
Goal_Orientation_w=0
Avoid_all = 0
if_pub_goal=False
i = 0
j = 0
L_time = 0
R_time = 100
block_count = 0
odom_position_z = 0
last_position_z = 0
if_navigation = False
if_odom = False
odom_z_finish = False
count_test = 0
if_not_goal_close = True
if_not_goal_reach = True
if_move_base_cancel=False
if_avoid=False
# if_avoid_pub= rospy.Publisher('if_avoid',Bool,queue_size=10)
# range_pub=rospy.Publisher('range_avoid',Range,queue_size=100)
# scan_pub=rospy.Publisher('scan_avoid',LaserScan,queue_size=10)

def get_distance(msg):
	"""获取超声波数据的回调函数"""
	global TurnLeft
	global TurnRight
	global Avoid_all
	global block_count
	global start_avoid
	global start_avoidLR
	global if_not_goal_close
	global if_avoid
	global if_cmd_vel
	global i
	global j
	global Goal_Position_x
	global Goal_Position_y
	global Goal_Position_z
	global Goal_Orientation_x
	global Goal_Orientation_y
	global Goal_Orientation_z
	global Goal_Orientation_w
	global if_move_base_cancel
	# 只有当超声波的距离合理时才会被接收，确保数据无误
	if msg.distanceA>=0 and msg.distanceA <=6:
		dis.distanceA = msg.distanceA
	else:
		dis.distanceA=3

	if msg.distanceB>=0 and msg.distanceA <=6:
		dis.distanceB = msg.distanceB
	else:
		dis.distanceB=3

	if msg.distanceC>=0 and msg.distanceC <=6:
		dis.distanceC = msg.distanceC
	else:
		dis.distanceC=3

	if msg.distanceD>=0 and msg.distanceD <=6:
		dis.distanceD = msg.distanceD
	else:
		dis.distanceD=3

	if msg.distanceE>=0 and msg.distanceE <=6:
		dis.distanceE = msg.distanceE
	else:
		dis.distanceE=3

	if msg.distanceF>=0 and msg.distanceF <=6:
		dis.distanceF = msg.distanceF
	else:
		dis.distanceF=3

	if msg.distanceG>=0 and msg.distanceG <=6:
		dis.distanceG = msg.distanceG
	else:
		dis.distanceG=3
		
	if msg.distanceH>=0 and msg.distanceH <=6:
		dis.distanceH = msg.distanceH
	else:
		dis.distanceH=3
	# 检测到障碍物，判断哪一边空间广阔，给出转弯标志位
	if (dis.distanceA < start_avoidLR or dis.distanceB < start_avoid or dis.distanceC < start_avoid or dis.distanceD < start_avoid or dis.distanceE < start_avoid or dis.distanceF < start_avoidLR) and if_not_goal_close:
		if_avoid=True

		if dis.distanceA + dis.distanceB + dis.distanceC < dis.distanceD + dis.distanceE + dis.distanceF:
			TurnLeft = 0
			TurnRight = 1
		else:
			TurnRight = 0
			TurnLeft = 1
	else:
		if_avoid=False
		TurnLeft = 0
		TurnRight = 0
	# if not if_not_goal_close:
	# 	rospy.loginfo("The goal is too closed")
	# 检测到障碍物围绕，通过自转测量哪个方位没有障碍物
	if block_count > 20:
		if dis.distanceA > start_avoidLR and dis.distanceB > start_avoid and dis.distanceC > start_avoid and dis.distanceD > start_avoid and dis.distanceE > start_avoid and dis.distanceF > start_avoidLR:
			block_count = 0
			R_time = 100
			L_time = 0

def DistanceAvoid(msg):
	"""订阅cmd_vel的回调函数，截断数据并重新发送"""
	global TurnLeft
	global TurnRight
	global L_time
	global R_time
	global Avoid_all
	global block_count
	global if_navigation
	global odom_position_z
	global count_test
	global if_odom
	global odom_z_finish
	global TurnLeft2
	global TurnRight2
	global i
	global j
	global if_cmd_vel
	global if_move_base_cancel
	global vel_msg
	global if_avoid
	vel_msg = msg

	# if not navigation
	# 被障碍物围绕，固定向左自传寻找突破口
	if not if_navigation:
		if block_count>20:
			if vel_msg.linear.x>0:
				vel_msg.linear.x=0
				vel_msg.angular.z = 0.5

		# 非围绕情况，只有左转和右转
		else:
			if TurnLeft == 1:
				if vel_msg.linear.x>0:
					L_time = time.time()
					vel_msg.linear.x=0
					vel_msg.angular.z = 0.5

			if TurnRight == 1:
				if vel_msg.linear.x>0:
					R_time = time.time()
					vel_msg.linear.x=0
					vel_msg.angular.z = -0.5
		# 左右转弯时间间隔太小，考虑是否被障碍物围绕
		if abs(L_time - R_time) < 5:
			block_count  = block_count + 1
		car_vel_pub.publish(vel_msg)
	#if navigation
	else:
		if_move_base_cancel=False
		if block_count>20:
			if vel_msg.linear.x>0:
				if_cmd_vel=True
				vel_msg.linear.x=0
				vel_msg.angular.z = 0.5
				# car_vel_pub.publish(vel_msg)
		# 非围绕情况，只有左转和右转
		else:
			if TurnLeft == 1:
				# rospy.loginfo("TurnLeft......")
				if msg.linear.x>0:
					L_time = time.time()
					vel_msg.linear.x=0
					if_cmd_vel=True
					# if vel_msg.angular.z > 0:
					vel_msg.angular.z = 0.5
					# if vel_msg.angular.z < 0:
						# vel_msg.angular.z = -0.5
					TurnLeft2 = 1
					# car_vel_pub.publish(vel_msg)
			# if TurnLeft == 0 and TurnRight == 0 and TurnLeft2 == 1:
			# 	# TurnLeft =0
			# 	# TurnLeft2 = 0
			# 	# j=0
			# 	if_cmd_vel=False
			# 	if j<20:
			# 		j=j+1
			# 		vel_msg.linear.x=0.2
			# 		vel_msg.angular.z=0
			# 	if j==20:
			# 		j=0
			# 		TurnLeft2=0
			# 	# rospy.loginfo("move base cancel")
			# 	# Pub_move_base_cancel(GoalID_)

			if TurnRight == 1:
				if vel_msg.linear.x>0:
					R_time = time.time()
					if_cmd_vel=True
					vel_msg.linear.x=0
					# if vel_msg.angular.z > 0:
						# vel_msg.angular.z = 0.5
					# if vel_msg.angular.z < 0:
					vel_msg.angular.z = -0.5
					TurnRight2 = 1
					# car_vel_pub.publish(vel_msg)
			if TurnLeft == 0 and TurnRight == 0 and (TurnLeft2 == 1 or TurnRight2==1):
				# TurnLeft =0
				# TurnLeft2 = 0
				# j=0
				if_cmd_vel=False
				if j<20:
					j=j+1
					print(j)
					vel_msg.linear.x=0.2
					vel_msg.angular.z=0
				if j==20:
					j=0
					TurnLeft2=0
					TurnRight2=0
					Pub_move_base_cancel(GoalID_)

				# rospy.loginfo("move base cancel")
				# Pub_move_base_cancel(GoalID_)
			# if TurnRight == 0 and TurnLeft==0 and TurnRight2 == 1:
			# 	TurnRight = 0
			# 	TurnRight2 = 0
			# 	j=0
			# 	if_cmd_vel=False
			# 	rospy.loginfo("move base cancel")
			# 	Pub_move_base_cancel(GoalID_)

		# 左右转弯时间间隔太小，考虑是否被障碍物围绕
		if abs(L_time - R_time) < 5:
			block_count  = block_count + 1
		# if TurnLeft2==0 and TurnRight2==0 :
		car_vel_pub.publish(vel_msg)

def MoveBaseActionGoalCb(msg):
	global Goal_Frame_id
	global Goal_Position_x
	global Goal_Position_y
	global Goal_Position_z
	global Goal_Orientation_x
	global Goal_Orientation_y
	global Goal_Orientation_z
	global Goal_Orientation_w
	Goal_Frame_id=msg.goal.target_pose.header.frame_id
	Goal_Position_x=msg.goal.target_pose.pose.position.x
	Goal_Position_y=msg.goal.target_pose.pose.position.y
	Goal_Position_z=msg.goal.target_pose.pose.position.z
	Goal_Orientation_x=msg.goal.target_pose.pose.orientation.x
	Goal_Orientation_y=msg.goal.target_pose.pose.orientation.y
	Goal_Orientation_z=msg.goal.target_pose.pose.orientation.z
	Goal_Orientation_w=msg.goal.target_pose.pose.orientation.w
	# rospy.loginfo_once(Goal_Frame_id)
	# rospy.loginfo_once("Goal_Position_x:%f\n",msg.goal.target_pose.pose.position.x)
	# rospy.loginfo_once("Goal_Position_y:%f\n",msg.goal.target_pose.pose.position.y)

def MoveBaseActionFeedbackCb(msg):
	global Goal_Position_x
	global Goal_Position_y
	global start_avoid
	global if_not_goal_close
	global if_not_goal_reach
	# print(start_avoid)
	feedback_position_x=msg.feedback.base_position.pose.position.x
	feedback_position_y=msg.feedback.base_position.pose.position.y
	if abs(feedback_position_x - Goal_Position_x)<start_avoid and abs(feedback_position_y - Goal_Position_y)<start_avoid and if_not_goal_reach:
		if_not_goal_close=False
		# print("feedback_position_x:%f\n",msg.feedback.base_position.pose.position.x)
		# print("feedback_position_y:%f\n",msg.feedback.base_position.pose.position.y)
	if not if_not_goal_reach:
		if_not_goal_reach=True
def MoveBaseActionResultCb(msg):
	global if_not_goal_close
	global if_not_goal_reach
	if msg.status.status ==3:
		if_not_goal_close=True
		if_not_goal_reach=False
		# rospy.loginfo("Goal has reached,goal close has closed")
def Pub_move_base_cancel(msg):
	global if_move_base_cancel
	global if_pub_goal
	rospy.loginfo("It is pub move_base_cancel")
	if_move_base_cancel=True
	move_base_cancel.publish(msg)
	if_pub_goal=False


def Pub_move_base_goal(msg):
	print(msg.goal.target_pose.pose.position.x)
	move_base_goal.publish(msg)

# def Pub_Forward_cmd_vel(msg):
# 	global vel_msg
# 	vel_msg=msg
# 	print(vel_msg.linear.x)
# 	car_vel_pub.publish(vel_msg)
def change_vel():
	"""初始化避障函数"""
	global start_avoid
	global start_avoidLR
	global if_navigation
	global if_odom
	global if_cmd_vel
	global TurnRight
	global TurnLeft
	global if_move_base_cancel
	global Goal_Frame_id
	global Goal_Position_x
	global Goal_Position_y
	global Goal_Position_z
	global Goal_Orientation_x
	global Goal_Orientation_y
	global Goal_Orientation_z
	global Goal_Orientation_w
	global j
	global if_pub_goal
	global car_vel_pub
	global move_base_goal
	global move_base_cancel
	global vel_msg
	global if_avoid
	rospy.init_node('supersonic_avoid',anonymous=True)

	# 订阅超声波话题、速度话题
	rospy.Subscriber('Distance',supersonic,get_distance)
	rospy.Subscriber('cmd_vel',Twist,DistanceAvoid)
	# rospy.Subscriber('odom',Odometry,Odomcallback)
	rospy.Subscriber('move_base/goal',MoveBaseActionGoal,MoveBaseActionGoalCb)
	rospy.Subscriber('move_base/feedback',MoveBaseActionFeedback,MoveBaseActionFeedbackCb)
	rospy.Subscriber('move_base/result',MoveBaseActionResult,MoveBaseActionResultCb)


	# 获取开始避障的距离
	start_avoid = rospy.get_param("~start_avoid",0.4)
	start_avoidLR = rospy.get_param("~start_avoidLR",0.3)
	if_navigation = rospy.get_param("navigation",False)
	rate=rospy.Rate(10)
	# 打印成功开启避障的消息和开始避障的距离
	print("\033[;32m已启用超声波避障功能\033[0m")
	print("获取到的前方开始避障距离：%.2f m" %start_avoid)
	print("获取到的侧方开始避障距离：%.2f m" %start_avoidLR)
	# print("\033[;32m参数修改路径\033[0m")
	# print("\033[;32m~/wheeltec_robot/src/turn_on_wheeltec_robot/launch/include/base_serial.launch\033[0m")
	# 死循环等待回调函数
	while not rospy.is_shutdown():
		# 创建一个话题发布者，用于发送cmd_vel的话题
		car_vel_pub = rospy.Publisher('/change_cmd_vel',Twist,queue_size=20)
		move_base_cancel=rospy.Publisher('/move_base/cancel',GoalID,queue_size=20)
		move_base_goal=rospy.Publisher('/move_base/goal',MoveBaseActionGoal,queue_size=20)

		# if TurnRight==0 and TurnLeft==0 and if_pub_goal==False and if_avoid==False and if_cmd_vel==False:
		# 	rospy.loginfo("111111111111111")
		# 	j=j+1
		# 	print(j)
		# 	#前进一会然后发布导航点
		# 	if j <10:
		# 		vel_msg.linear.x=0.2
		# 		vel_msg.angular.z=0
		# 		car_vel_pub.publish(vel_msg)
		# 	# if TurnLeft == 1 or TurnRight == 1 or if_pub_goal==True:
		# 	# 	break
		# 	# time.sleep(1)
		if if_move_base_cancel==True and if_pub_goal==False:
			Goal.goal.target_pose.header.frame_id=Goal_Frame_id
			Goal.goal.target_pose.pose.position.x=Goal_Position_x
			Goal.goal.target_pose.pose.position.y=Goal_Position_y
			Goal.goal.target_pose.pose.position.z=Goal_Position_z
			Goal.goal.target_pose.pose.orientation.x=Goal_Orientation_x
			Goal.goal.target_pose.pose.orientation.y=Goal_Orientation_y
			Goal.goal.target_pose.pose.orientation.z=Goal_Orientation_z
			Goal.goal.target_pose.pose.orientation.w=Goal_Orientation_w
			rospy.loginfo_once("Pub move_base_goal")
			if_move_base_cancel=False
			if_pub_goal=True
			Pub_move_base_goal(Goal)
		# 		j=0
		rate.sleep()
	# rospy.spin()

if __name__=='__main__':
	try:
		change_vel()
	except rospy.ROSInterruptException:
		pass
