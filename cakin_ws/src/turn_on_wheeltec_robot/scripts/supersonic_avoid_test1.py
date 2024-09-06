#!/usr/bin/env python
# coding=utf-8

import rospy
import _thread,time

# 用于记录、发布导航点
from geometry_msgs.msg import PoseStamped

# 中断导航相关
from actionlib_msgs.msg import GoalID

# 获取导航结果
from move_base_msgs.msg import MoveBaseActionResult

# 速度数据类型相关
from geometry_msgs.msg import Twist

# 超声波数据类型相关
from turn_on_wheeltec_robot.msg import supersonic

# 超声波类
class distance_avoid():
	def __init__(self):
		# 创建超声波避障ros节点
		rospy.init_node("supersonic_avoid") 

		# 超声波传感器成员
		self.distanceA = 3.0
		self.distanceB = 3.0
		self.distanceC = 3.0
		self.distanceD = 3.0
		self.distanceE = 3.0
		self.distanceF = 3.0

		# 是否遇到障碍物标志位
		self.avoid_flag = 0
		# 左转标志位
		self.turn_left = 0
		# 右转标志位
		self.turn_right = 0
		# 转圈标志位
		self.turn_around = 0
		self.filter_count = 0
		self.left_start = 0
		self.right_start = 0
		# 走直线标志位
		self.drive_line = 0
		self.drive_line_flag = 0
		# 中间开始避障的距离
		self.min_distance = 0.3
		# 左右两边开始避障的距离
		self.min_distance_LR = 0.3
		# 是否开启导航功能的标志位
		self.nav_mode_flag = 0
		# 用于计算ros时差的变量
		self.begin_time=rospy.Time.now()
		self.end_time=rospy.Time.now()
		self.drve_line_time=rospy.Time.now()
		self.turn_left_time = rospy.Time.now()
		self.turn_right_time = rospy.Time.now()
		# 保存导航点位的变量
		self.nav_goal_point = {'p_x':0,'p_y':0,'orien_z':0,'orien_w':0}
		# 创建重制导航目标点发布者
		self.Nav_Position_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
		# 创建导航中断话题发布者
		self.NavGoal_Cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=5)
		# 创建超声波避障速度话题发布者
		self.Avoid_vel_pub = rospy.Publisher('/change_cmd_vel',Twist,queue_size=10)
		# 创建导航结果话题订阅者
		self.Nav_result_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.Nav_Result_callback)
		# 创建速度话题订阅者
		self.Cmd_vel_sub = rospy.Subscriber('cmd_vel',Twist,self.DistanceAvoid_callback)
		# 创建超声波话题订阅者
		self.Supersonic_sub = rospy.Subscriber('Distance',supersonic,self.Get_distance_callback)
		# 创建导航目标点话题订阅者
		self.Nav_goal_sub = rospy.Subscriber("move_base_simple/goal",PoseStamped,self.Nav_goal_callback)

	# 取消导航功能	
	def Pub_NavGoal_Cancel(self):
		topic=GoalID()
		self.NavGoal_Cancel_pub.publish(topic)

	# 使用保存的导航点重新发布
	def Pub_Nav_Position(self):
		nav_goal=PoseStamped()
		nav_goal.header.frame_id = 'map'
		nav_goal.header.stamp = rospy.Time.now()
		nav_goal.pose.position.x = self.nav_goal_point['p_x']
		nav_goal.pose.position.y = self.nav_goal_point['p_y']
		nav_goal.pose.orientation.z = self.nav_goal_point['orien_z']
		nav_goal.pose.orientation.w = self.nav_goal_point['orien_w']
		self.Nav_Position_pub.publish(nav_goal)

	# 超声波话题订阅回调函数
	def Get_distance_callback(self,topic):
		# 只接收合理数值
		if topic.distanceA >=0 and topic.distanceA<=6:
			self.distanceA = topic.distanceA
		else:
			self.distanceA = 5.2

		if topic.distanceB >=0 and topic.distanceB<=6:
			self.distanceB = topic.distanceB
		else:
			self.distanceB = 5.2

		if topic.distanceC >=0 and topic.distanceC<=6:
			self.distanceC = topic.distanceC
		else:
			self.distanceC = 5.2

		if topic.distanceD >=0 and topic.distanceD<=6:
			self.distanceD = topic.distanceD
		else:
			self.distanceD = 5.2

		if topic.distanceE >=0 and topic.distanceE<=6:
			self.distanceE = topic.distanceE
		else:
			self.distanceE = 5.2

		if topic.distanceF >=0 and topic.distanceF<=6:
			self.distanceF = topic.distanceF
		else:
			self.distanceF = 5.2

		# 根据超声波的数值，给出避障方案
		if self.distanceA < self.min_distance_LR or self.distanceB < self.min_distance or self.distanceC < self.min_distance or self.distanceD < self.min_distance or self.distanceE < self.min_distance or self.distanceF < self.min_distance_LR:
			# 遇到障碍物开始计时
			if self.avoid_flag == 0:
				self.avoid_flag = 1
				self.begin_time=rospy.Time.now()

			# 给出避障方案：遇到障碍物时，计算两边的离障碍物的距离，往距离大的方向转
			if self.distanceA + self.distanceB + self.distanceC < self.distanceD + self.distanceE + self.distanceF:
				self.turn_right = 1
				self.turn_left = 0
			else:
				self.turn_left = 1
				self.turn_right = 0
		else:
			# 在遇到障碍物结束后，判断是否满足直线规划条件
			if self.avoid_flag == 1:
				self.avoid_flag = 0
				if (rospy.Time.now() - self.begin_time).secs>=1:
					# 导航模式才需要进行直线规划
					if self.nav_mode_flag ==1:
						self.drive_line = 1
						print("Start drive_line mode!")
			self.turn_right = 0
			self.turn_left = 0
			self.turn_around = 0

	# 导航结束回调函数
	def Nav_Result_callback(self,topic):
		pass

	# 小车速度回调函数
	def DistanceAvoid_callback(self,topic):
		change_vel = Twist()
		change_vel = topic

		# 根据超声波回调函数给出的方案进行避障
		# 导航模式才存在直线规划
		if self.drive_line == 1:
			if self.drive_line_flag == 0:
				self.drive_line_flag=1
				self.drve_line_time = rospy.Time.now()
			change_vel = Twist()
			change_vel.linear.x = 0.4

		# 非旋转绕障模式
		if self.turn_around == 0:
			# 向右避障
			if self.turn_right == 1:
				if change_vel.linear.x>0:
					# 记录1次时间，用于判断机器人是否被障碍物围绕
					if self.right_start == 0:
						self.right_start=1
						self.turn_right_time = rospy.Time.now()
					change_vel.linear.x = 0
					change_vel.angular.z = -0.8
			# 向左避障
			if self.turn_left == 1:
				if change_vel.linear.x>0:
					# 记录1次时间，用于判断机器人是否被障碍物围绕
					if self.left_start == 0:
						self.left_start=1
						self.turn_left_time = rospy.Time.now()					
					change_vel.linear.x = 0
					change_vel.angular.z = 0.8
		# 旋转绕障模式
		else:
			if change_vel.linear.x>0:
				change_vel.linear.x = 0
				change_vel.angular.z = 0.8

		# 旋转寻找突破口判断
		if self.left_start==1 and self.right_start==1:
			time = 0
			self.left_start=0
			self.right_start=0
			time = (self.turn_right_time - self.turn_left_time).secs
			time = abs(time)
			if time<=3:
				self.filter_count = self.filter_count + 1
			if self.filter_count >= 2:
				self.filter_count = 0
				self.turn_around = 1

		# 直线规划状态更新
		if self.drive_line_flag==1:
			if (rospy.Time.now() - self.drve_line_time).secs>=3:
				self.drive_line = 0
				self.drive_line_flag=0

		# 发布避障后的命令
		self.Avoid_vel_pub.publish(change_vel)

	# 导航目标点回调函数
	def Nav_goal_callback(self,topic):
		# 能进入该回调函数，说明当前小车处于导航开启的模式
		self.nav_mode_flag = 1
		# 保存导航点的信息，用于再次发布
		self.nav_goal_point['p_x'] = topic.pose.position.x
		self.nav_goal_point['p_y'] = topic.pose.position.y
		self.nav_goal_point['orien_z'] = topic.pose.orientation.z
		self.nav_goal_point['orien_w'] = topic.pose.orientation.w

if __name__ == '__main__':
	try:
		distance_avoid=distance_avoid() #创建超声波避障对象

		# 获取开始避障的距离
		distance_avoid.min_distance = rospy.get_param("~start_avoid",0.4)
		distance_avoid.min_distance_LR = rospy.get_param("~start_avoidLR",0.3)
		# 打印成功开启避障的消息和开始避障的距离
		print("\033[;32m已启用超声波避障\033[0m")
		print("获取到的前方开始避障距离：%.2f" %distance_avoid.min_distance)
		print("获取到的侧方开始避障距离：%.2f" %distance_avoid.min_distance_LR)

		# 死循环等待回调函数
		while not rospy.is_shutdown():
			rospy.spin()

	except rospy.ROSInterruptException:
		print('exception')
