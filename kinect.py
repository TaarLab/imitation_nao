# Run these commands in ROS terminal:
# roscd skeleton_markers
# rosrun rviz rviz -d markers.rviz
# roslaunch skeleton_markers markers.launch

import os
import sys
import time
import curses
import numpy as np
import math
import traceback
import json
import gzip

import argparse
import motion
import almath
from naoqi import ALProxy

import rospy
import tf

from kinematics import NaoKinematics
from convex_plygonal import Point, ConvexPolygonal
from filter import Filter

FRAME = 'openni_depth_frame'

class Main(object):
	"""Main class of the program

	This class contains methods to get data from kinect then imitate the
	human posture and keep balance of the NAO robot.

	"""
	alpha = 0.6
	right_shoulder = Filter(alpha)
	right_elbow = Filter(alpha)
	right_hand = Filter(alpha)
	left_shoulder = Filter(alpha)
	left_elbow = Filter(alpha)
	left_hand = Filter(alpha)
	alpha = 0.2
	right_hip = Filter(alpha)
	right_knee = Filter(alpha)
	right_foot = Filter(alpha)
	left_hip = Filter(alpha)
	left_knee = Filter(alpha)
	left_foot = Filter(alpha)
	alpha = 0.3
	torso_w = Filter(alpha)			# world frame
	left_hip_w = Filter(alpha)
	left_foot_w = Filter(alpha)
	right_hip_w = Filter(alpha)
	right_foot_w = Filter(alpha)

	pitch_error_filter = Filter(0.25)
	roll_error_filter = Filter(0.4)

	def __init__(self, ip, port):
		"""Init the main class

		Args:
			ip: IP address of the robot
			port: Port number of the robot
		"""

		#
		self.stdscr = None
		self.ip = ip
		self.port = port
		self.start = time.time()
		self.last = self.start
		self.fo = gzip.open("log.txt.gz", "w")
		self._log = []
		print self.ip, self.port
		self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
		self.postureProxy = ALProxy("ALRobotPosture", self.ip, self.port)
		self.memProxy = ALProxy("ALMemory", self.ip, self.port)
		self.dt = 1./20.
		self.is_waiting = True

		#  Imitation
		self.kinect_enabled = False
		if self.kinect_enabled:
			rospy.init_node('Imitation')
			self.tf_listener = tf.TransformListener()

		self.support_foot_tendency = 0.0		# -1: right foot, 0: middle, 1: left foot
		self.swing_leg_height = 0.0


		#  Nao
		self.angles = [[0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
		self.angles_command = self.angles[:]
		self.send = None
		self.nao = NaoKinematics()
		self.nao_init = NaoKinematics()
		self.nao_test = NaoKinematics()
		self.nao.setTheta(self.angles)
		self.nao_init.setTheta(self.angles)
		self.nao_test.setTheta(self.angles)
		self.com = None
		self.com_sensor = None

		self.gravity = [0.0, 0.0]
		self.gyro = [0.0, 0.0]
		self.Q0toR = None
		self.P0toR = None

		#  Support Area
		leg_points = [[ 0.102,  0.000, 0],
					  [ 0.085,  0.038, 0],
					  [ 0.054,  0.048, 0],
					  [-0.028,  0.052, 0],
					  [-0.049,  0.036, 0],
					  [-0.058,  0.005, 0],
					  [-0.049, -0.022, 0],
					  [-0.034, -0.034, 0],
					  [ 0.061, -0.039, 0],
					  [ 0.091, -0.031, 0]]

		self._left_leg_points = [self.nao_init.left_leg.Q[-1].T.dot(np.array(p)[:, np.newaxis]) for p in leg_points]
		self._right_leg_points = [self.nao_init.right_leg.Q[-1].T.dot((np.array(p)*[1,-1,1])[:, np.newaxis]) for p in leg_points]

		self.left_leg_points = None
		self.right_leg_points = None
		self.support_area = None

		#  Balance
		self.pitch_error = None
		self.pitch_prev_error = None
		self.pitch_prev_error_filter = None
		self.pitch_integral = 0.0

		self.pitch_current = 0.0
		self.pitch_command = 0.0
		self.pitch_target = 0.0

		self.roll_error = None
		self.roll_prev_error = None
		self.roll_prev_error_filter = None
		self.roll_integral = 0.0

		self.roll_current = 0.0
		self.roll_command = 0.0
		self.roll_target = 0.0

		self.support_foot_bend = 0.0
		self.support_foot_bend_rotation = 0.0

		self.single_support = False

		#  Config
		self.com_alpha = 1.0
		self.gravity_alpha = 1.0

		self.max_support_foot_tendency = 1.0
		self.min_support_foot_bend = 0.75
		self.max_support_foot_bend = 1.25


		# Pitch
		self.pitch_Kp = 0.60
		self.pitch_Ki = 0.96
		self.pitch_Kd = 0.09

		# Roll
		self.roll_Kp = 0.888
		self.roll_Ki = 0.35
		self.roll_Kd = 0.0875

		#  Debug
		self.debug = []
		self.test = None
		self.test2 = None

	def main(self, stdscr):
		"""Main function that runs when application starts

		This method starts a run loop and call other functions whe needed

		"""

		self.stdscr = stdscr
		self.stdscr.nodelay(True)
		np.set_printoptions(precision=5, suppress=True)
		self.fo.write('[\n')

		self.start_stance()

		if self.kinect_enabled:
			self.tf_listener.waitForTransform(FRAME, FRAME, rospy.Time(), rospy.Duration(60))

		key = ''
		while key != ord('q') and (self.kinect_enabled or time.time() - self.start <= 10.0):
			try:
				self.stdscr.clear()

				self.stdscr.addstr(1, 1, "Started. Hit 'q' to quit")
				key = self.stdscr.getch()

				self.debug = []

				self.log_time()
				if self.kinect_enabled and not first('kinect'):
					self.update_kinect()
					self.imitate()
				self.log_time()
				self.update()
				self.log_time()
				self.balance()
				self.apply()
				self.log_time()
				self.log()

			except:
				# stdscr.addstr(18, 1, "Unexpected error: " + str(sys.exc_info()[1]))
				stdscr.addstr(4, 1, traceback.format_exc())

			stdscr.refresh()
			time.sleep(self.dt - (time.time()-self.start) % self.dt)

		self.fo.write('\n]')
		self.fo.close()
		self.motionProxy.rest()

	def start_stance(self):
		"""Moves robot to start posture"""

		self.motionProxy.wakeUp()
		self.motionProxy.setStiffnesses("Body", 1.0)

		#self.postureProxy.goToPosture("Crouch", 1.0)
		self.postureProxy.goToPosture("StandInit", 2.0)

		a = 0.325			# zero roll
		angles = [0.0, -a , 0.0, 0.0, 0.052, a ]			# single pitch
		if self.kinect_enabled:
			angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		t = 3.0
		times = [t, t, t, t, t, t]
		names = ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll", "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll"]
		self.motionProxy.angleInterpolation(names, angles + angles, times + times, True)

		angles = [0.0, -a-0.04, -0.5, +1.0, -0.5, a]
		t = 1.0
		names = ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]
		self.motionProxy.angleInterpolation(names, angles, times, True)

		time.sleep(1)
		self.start = time.time()
		self.support_foot_tendency = 1.0
		self.single_support = True

	def update_kinect(self):
		"""Gets data from kinect and process them for later use"""

		self.torso_w = np.array(self.tf_listener.lookupTransform(FRAME, 'torso', rospy.Time(0))[0])[:, np.newaxis]
		self.left_hip_w = np.array(self.tf_listener.lookupTransform(FRAME, 'left_hip', rospy.Time(0))[0])[:, np.newaxis]
		self.left_foot_w = np.array(self.tf_listener.lookupTransform(FRAME, 'left_foot', rospy.Time(0))[0])[:, np.newaxis]
		self.right_hip_w = np.array(self.tf_listener.lookupTransform(FRAME, 'right_hip', rospy.Time(0))[0])[:, np.newaxis]
		self.right_foot_w = np.array(self.tf_listener.lookupTransform(FRAME, 'right_foot', rospy.Time(0))[0])[:, np.newaxis]

		#  Find which foot is above the other
		self.swing_leg_height = float(self.left_foot_w[1] - self.right_foot_w[1])

		#  Find image of torso on the x axis normalized for line between two feet
		s = float((self.torso_w[0] - self.right_foot_w[0]) / (self.left_foot_w[0] - self.right_foot_w[0]))
		s = clamp(s * 2.0 - 1.0, -1.0, 1.0)
		sign_s = sign(s)

		s /= self.max_support_foot_tendency

		alpha = 0.1
		self.support_foot_tendency = alpha * s + (1.0 - alpha) * self.support_foot_tendency
		self.single_support = False
		if abs(self.swing_leg_height) > 0.15:
			self.single_support = True
			self.support_foot_tendency = sign_s * clamp(abs(self.support_foot_tendency), 1, 1.1)

		hipd = distance(self.left_hip_w, self.right_hip_w)
		d = float((self.right_foot_w[0] - self.left_foot_w[0]) / hipd)
		f = clamp((d - self.min_support_foot_bend) / (self.max_support_foot_bend - self.min_support_foot_bend), 0.0, 2.5)
		self.support_foot_bend = abs(self.support_foot_tendency * f)

		self.debug.append(self.support_foot_tendency)
		self.debug.append(self.swing_leg_height)
		self.debug.append(hipd)
		self.debug.append(d)
		self.debug.append(f)
		self.debug.append(self.support_foot_bend)

		self.stdscr.addstr(7, 1, "tendency: %1.3f" % self.support_foot_tendency)
		self.stdscr.addstr(8, 1, "height: %2.2f" % self.swing_leg_height)
		self.stdscr.addstr(9, 1, "hip dist: %2.2f" % hipd)
		self.stdscr.addstr(10, 1, "leg dist scale: %1.3f" % d)
		self.stdscr.addstr(11, 1, "bend f: %1.3f" % f)
		self.stdscr.addstr(12, 1, "bend: %1.3f" % self.support_foot_bend)

	def imitate(self):
		"""Uses the data from kinect to imitate human

		This method gets data from a TransformListener and calls move_hand and move_leg

		"""
		# Left hand
		self.left_shoulder = np.array(self.tf_listener.lookupTransform('torso', 'left_shoulder', rospy.Time(0))[0])
		self.left_elbow = np.array(self.tf_listener.lookupTransform('torso', 'left_elbow', rospy.Time(0))[0])
		self.left_hand = np.array(self.tf_listener.lookupTransform('torso', 'left_hand', rospy.Time(0))[0])

		left_elbow = self.left_elbow - self.left_shoulder
		left_hand = self.left_hand - self.left_elbow

		self.move_hand([left_elbow, left_hand])

		# Right hand
		self.right_shoulder = np.array(self.tf_listener.lookupTransform('torso', 'right_shoulder', rospy.Time(0))[0])
		self.right_elbow = np.array(self.tf_listener.lookupTransform('torso', 'right_elbow', rospy.Time(0))[0])
		self.right_hand = np.array(self.tf_listener.lookupTransform('torso', 'right_hand', rospy.Time(0))[0])

		right_elbow = self.right_elbow - self.right_shoulder
		right_hand = self.right_hand - self.right_elbow

		self.move_hand([right_elbow, right_hand], True)

		# Left leg
		self.left_hip = np.array(self.tf_listener.lookupTransform('torso', 'left_hip', rospy.Time(0))[0])
		self.left_knee = np.array(self.tf_listener.lookupTransform('torso', 'left_knee', rospy.Time(0))[0])
		self.left_foot = np.array(self.tf_listener.lookupTransform('torso', 'left_foot', rospy.Time(0))[0])

		left_knee = self.left_knee - self.left_hip
		left_foot = self.left_foot - self.left_knee

		self.move_leg([left_knee, left_foot])

		# Right leg
		self.right_hip = np.array(self.tf_listener.lookupTransform('torso', 'right_hip', rospy.Time(0))[0])
		self.right_knee = np.array(self.tf_listener.lookupTransform('torso', 'right_knee', rospy.Time(0))[0])
		self.right_foot = np.array(self.tf_listener.lookupTransform('torso', 'right_foot', rospy.Time(0))[0])

		right_knee = self.right_knee - self.right_hip
		right_foot = self.right_foot - self.right_knee

		self.move_leg([right_knee, right_foot], True)

	def move_hand(self, hand_joints, right_hand=False):
		"""Moves robot's hand to the joint locations

		This method uses a geometric approach to calculate joint theta from
		joint locations in the cartesian space

		Args:
			hand_joints: Hand joint positions
			right_hand: Is this right or left hand

		"""

		if right_hand:
			names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		else:
			names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
		limited = []

		#  NAO Links:
		l = [0.015, 0.105, 0.05595, 0.05775, 0.01231]
		#  NAO Joint constraints:
		if right_hand:
			j_min = [-2.0857, -1.3265, -2.0857, 0.0349, -1.8238]
			j_max = [ 2.0857,  0.3142,  2.0857, 1.5446,  1.8238]
		else:
			j_min = [-2.0857, -0.3142, -2.0857, -1.5446, -1.8238]
			j_max = [ 2.0857,  1.3265,  2.0857, -0.0349,  1.8238]

		tolerance = 0.001

		#  Angles to compute:
		angles = [0.0, 0.0, 0.0, 0.0, 0.0]

		#  Remap first joint to nao
		q = np.array([
			[0, 0,  -1],
			[0, -1, 0],
			[-1, 0, 0],
		], dtype=float)

		v = hand_joints[0][:,np.newaxis]
		v = v * math.sqrt(l[0]*l[0] + l[1]*l[1]) / math.sqrt(v.T.dot(v))
		p = q.T.dot(v)

		#  Compute angles
		if right_hand:
			angles[1] = math.asin(p[2] / math.sqrt(l[0]*l[0] + l[1]*l[1])) + math.atan(l[0] / l[1])
		else:
			angles[1] = math.asin(p[2] / math.sqrt(l[0]*l[0] + l[1]*l[1])) + math.atan(-l[0] / l[1])

		if right_hand:
			x = p[0] / (l[1]*math.cos(angles[1]) + l[0]*math.sin(angles[1]))
			y = p[1] / (l[1]*math.cos(angles[1]) + l[0]*math.sin(angles[1]))
		else:
			x = p[0] / (l[1]*math.cos(angles[1]) - l[0]*math.sin(angles[1]))
			y = p[1] / (l[1]*math.cos(angles[1]) - l[0]*math.sin(angles[1]))
		angles[0] = math.atan2(y, x)

		#  Check constraints
		if angles[0] < j_min[0]+tolerance:
			angles[0] = j_min[0]+tolerance
			limited.append(names[0])
		if angles[0] > j_max[0]-tolerance:
			angles[0] = j_max[0]-tolerance
			limited.append(names[0])
		if angles[1] < j_min[1]+tolerance:
			angles[1] = j_min[1]+tolerance
			limited.append(names[1])
		if angles[1] > j_max[1]-tolerance:
			angles[1] = j_max[1]-tolerance
			limited.append(names[1])

		#  Remap second joint to nao
		q0 = np.array([
			[math.cos(angles[0]), 0,  math.sin(angles[0])],
			[math.sin(angles[0]), 0, -math.cos(angles[0])],
			[0, 1, 0],
		], dtype=float)

		q1 = np.array([
			[-math.sin(angles[1]), 0,  math.cos(angles[1])],
			[ math.cos(angles[1]), 0, math.sin(angles[1])],
			[0, 1, 0],
		], dtype=float)

		v = hand_joints[1][:,np.newaxis]
		v = v * l[2] / math.sqrt(v.T.dot(v))
		p = q1.T.dot(q0.T.dot(q.T.dot(v)))

		#  Compute angles
		if right_hand:
			angles[3] = math.acos(p[2] / l[2])
		else:
			angles[3] = -math.acos(p[2] / l[2])

		if right_hand:
			x = p[0] / (l[2] * math.sin(angles[3]))
			y = p[1] / (l[2] * math.sin(angles[3]))
		else:
			x = p[0] / (l[2] * math.sin(angles[3]))
			y = p[1] / (l[2] * math.sin(angles[3]))

		angles[2] = math.atan2(y, x)

		if right_hand:
			angles[4] = -angles[2] + math.pi/2
		else:
			angles[4] = -angles[2] - math.pi/2

		#  Check constraints
		if angles[2] < j_min[2]+tolerance:
			angles[2] = j_min[2]+tolerance
			limited.append(names[2])
		if angles[2] > j_max[2]-tolerance:
			angles[2] = j_max[2]-tolerance
			limited.append(names[2])
		if angles[3] < j_min[3]+tolerance:
			angles[3] = j_min[3]+tolerance
			limited.append(names[3])
		if angles[3] > j_max[3]-tolerance:
			angles[3] = j_max[3]-tolerance
			limited.append(names[3])
		if angles[4] < j_min[4]+tolerance:
			angles[4] = j_min[4]+tolerance
			limited.append(names[4])
		if angles[4] > j_max[4]-tolerance:
			angles[4] = j_max[4]-tolerance
			limited.append(names[4])

		if len(limited) > 0:
			self.stdscr.addstr(18 + int(right_hand), 1, "Joints limited: " + str(limited))

		self.send[1+int(right_hand)] = angles

	def move_leg(self, leg_joints, right_leg=False):
		"""Moves robot's leg to the joint locations

		This method uses a geometric approach to calculate joint theta from
		joint locations in cartesian space

		Args:
			leg_joints: Leg joint positions
			right_leg: Is this right or left leg

		"""

		if right_leg:
			names = ["RHipYawPitch", "RHipPitch", "RHipRoll", "RKneePitch", "RAnklePitch", "RAnkleRoll"]
		else:
			names = ["LHipYawPitch", "LHipPitch", "LHipRoll", "LKneePitch", "LAnklePitch", "LAnkleRoll"]
		limited = []

		leg_joints = np.array(leg_joints, dtype=float)

		#  NAO Links:.4
		l = [0.100, 0.1029, 0.04519]
		#  NAO Joint constraints:
		if right_leg:
			j_min = [-1.145303, -1.535889, -0.790477, -0.103083, -1.186448, -0.768992]
			j_max = [ 0.740810,  0.484090,  0.379472,  2.120198,  0.932056,  0.397935]
		else:
			j_min = [-1.145303, -1.535889, -0.379472, -0.092346, -1.189516, -0.397880]
			j_max = [ 0.740810,  0.484090,  0.790477,  2.112528,  0.922747,  0.769001]

		tolerance = 0.001

		#  Angles to compute:
		angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

		#  Remap first joint to nao
		q = np.array([
			[0, 0,  -1],
			[0, 1, 0],
			[1, 0, 0],
		], dtype=float)

		v = leg_joints[0][:, np.newaxis]
		v = v * l[0] / math.sqrt(v.T.dot(v))
		p = q.T.dot(v)

		#  Compute angles
		if right_leg:
			x = math.sqrt(p[0]*p[0] + p[1]*p[1]) / l[0]
			y = p[2] / l[0]
		else:
			x = math.sqrt(p[0]*p[0] + p[1]*p[1]) / l[0]
			y = p[2] / l[0]
		angles[2] += math.atan2(y, x)

		if right_leg:
			x = -p[1] / (l[0]*math.cos(angles[2]))
			y = p[0] / (l[0]*math.cos(angles[2]))
		else:
			x = -p[1] / (l[0]*math.cos(angles[2]))
			y = p[0] / (l[0]*math.cos(angles[2]))
		angles[1] += math.atan2(y, x)

		#  Check constraints
		if angles[0] < j_min[0]+tolerance:
			angles[0] = j_min[0]+tolerance
			limited.append(names[0])
		if angles[0] > j_max[0]-tolerance:
			angles[0] = j_max[0]-tolerance
			limited.append(names[0])
		if angles[1] < j_min[1]+tolerance:
			angles[1] = j_min[1]+tolerance
			limited.append(names[1])
		if angles[1] > j_max[1]-tolerance:
			angles[1] = j_max[1]-tolerance
			limited.append(names[1])
		if angles[2] < j_min[2]+tolerance:
			angles[2] = j_min[2]+tolerance
			limited.append(names[2])
		if angles[2] > j_max[2]-tolerance:
			angles[2] = j_max[2]-tolerance
			limited.append(names[2])

		#  Remap second joint to nao
		q1 = np.array([
			[ math.sin(angles[1]), 0, -math.cos(angles[1])],
			[-math.cos(angles[1]), 0, -math.sin(angles[1])],
			[0, 1, 0],
		], dtype=float)

		q2 = np.array([
			[math.cos(angles[2]), 0, -math.sin(angles[2])],
			[math.sin(angles[2]), 0,  math.cos(angles[2])],
			[0, -1, 0],
		], dtype=float)

		v = leg_joints[1][:, np.newaxis]
		v = v * l[2] / math.sqrt(v.T.dot(v))
		p = q2.T.dot(q1.T.dot(q.T.dot(v)))

		#  Compute angles

		if right_leg:
			x = p[0] / l[1]
			y = p[1] / l[1]
		else:
			x = p[0] / l[1]
			y = p[1] / l[1]

		angles[3] += math.atan2(y, x)

		#  Check constraints
		if angles[3] < j_min[3]+tolerance:
			angles[3] = j_min[3]+tolerance
			limited.append(names[3])
		if angles[3] > j_max[3]-tolerance:
			angles[3] = j_max[3]-tolerance
			limited.append(names[3])

		if angles[4] < j_min[4]+tolerance:
			angles[4] = j_min[4]+tolerance
			limited.append(names[4])
		if angles[4] > j_max[4]-tolerance:
			angles[4] = j_max[4]-tolerance
			limited.append(names[4])
		if angles[5] < j_min[5]+tolerance:
			angles[5] = j_min[5]+tolerance
			limited.append(names[5])
		if angles[5] > j_max[5]-tolerance:
			angles[5] = j_max[5]-tolerance
			limited.append(names[5])

		if len(limited) > 0:
			self.stdscr.addstr(16 + int(right_leg), 1, "Joints limited: " + str(limited))

		tmp = angles[1]
		angles[1] = angles[2]
		angles[2] = tmp

		self.send[3+int(right_leg)] = angles

	def update(self):
		"""Gets data from robot and process them for later use"""

		if time.time() - self.start <= 2.0:
			self.is_waiting = True
		else:
			self.is_waiting = False

		#  Get robot angles
		head = ["HeadYaw", "HeadPitch"]
		left_hand = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
		right_hand = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		left_leg = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll"]
		right_leg = ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]


		angles = []
		a = self.motionProxy.getAngles(head+left_hand+right_hand+left_leg+right_leg, True)
		angles.append(a[0:2])
		angles.append(a[2:7])
		angles.append(a[7:12])
		angles.append(a[12:18])
		angles.append(a[18:24])

		angles_command = []
		a = self.motionProxy.getAngles(head+left_hand+right_hand+left_leg+right_leg, False)
		angles_command.append(a[0:2])
		angles_command.append(a[2:7])
		angles_command.append(a[7:12])
		angles_command.append(a[12:18])
		angles_command.append(a[18:24])

		self.angles = angles
		self.angles_command = angles_command

		if self.send is None:
			self.send = angles[:]

		gx = self.memProxy.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value")
		gy = self.memProxy.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value")
		self.gravity[0] = self.gravity_alpha * gx + (1.0 - self.gravity_alpha) * self.gravity[0]
		self.gravity[1] = self.gravity_alpha * gy + (1.0 - self.gravity_alpha) * self.gravity[0]

		self.com_sensor = np.array(self.motionProxy.getCOM("Body", motion.FRAME_TORSO, True))[:, np.newaxis]

		#  Update model
		self.test = angles[:]
		self.test[3] = angles_command[3][:]
		self.test[4] = angles_command[4][:]
		self.test[3][4] = angles[3][4]
		self.test[4][4] = angles[4][4]
		self.test[3][5] = angles[3][5]
		self.test[4][5] = angles[4][5]
		self.nao.setTheta(self.test)

		#  Update robot ground frame
		#   Ground frame based on best foot
		if self.support_foot_tendency >= 0.0:
			self.Q0toR = self.nao_init.left_leg.Q[-1].dot(self.nao.left_leg.Q[-1].T)
		else:
			self.Q0toR = self.nao_init.right_leg.Q[-1].dot(self.nao.right_leg.Q[-1].T)

		#  Support Area
		self.left_leg_points = [(self.nao.left_leg.P[-1] + self.nao.left_leg.Q[-1].dot(p))[:, 0] for p in self._left_leg_points]
		self.right_leg_points = [(self.nao.right_leg.P[-1] + self.nao.right_leg.Q[-1].dot(p))[:, 0] for p in self._right_leg_points]

		if self.support_foot_tendency >= 0.0:
			z = min(self.Q0toR.dot(p)[2] for p in self.left_leg_points)
		else:
			z = min(self.Q0toR.dot(p)[2] for p in self.right_leg_points)
		self.P0toR = np.array([[0], [0], [-z]])

		#  Update support polygon
		if self.support_foot_tendency >= 0.0:
			sa = self.left_leg_points
		else:
			sa = self.right_leg_points

		points = list()
		for p in sa:
			pr = self.Q0toR.dot(p)
			points.append(Point(pr[0], pr[1]))

		self.support_area = ConvexPolygonal(*points)

	def balance(self):
		"""Keeps balance of the robot by controlling Vankle motors"""

		#   Ankle Pitch
		if self.support_foot_tendency >= 0.0:
			self.pitch_current = self.angles[3][4] - self.support_foot_bend * -0.25
			self.roll_current = self.angles[3][5]
		else:
			self.pitch_current = self.angles[4][4] - self.support_foot_bend * -0.25
			self.roll_current = self.angles[4][5]

		#  Compute target theta
		if self.support_foot_tendency >= 0.0:
			Vankle = (self.Q0toR.dot(self.nao.left_leg.P[5]) + self.P0toR)
		else:
			Vankle = (self.Q0toR.dot(self.nao.right_leg.P[5]) + self.P0toR)

		Vcom = (self.Q0toR.dot(self.nao.com) + self.P0toR)

		v = (Vcom - Vankle)
		r2 = v[0]**2 + v[2]**2
		x = self.support_area.mid.x - float(Vankle[0])
		y = math.sqrt(r2 + x**2)

		com_theta = math.atan2(v[2], v[0])
		sa_theta = math.atan2(y, x)


		self.stdscr.addstr(13, 1, "com r: %1.3f" % math.sqrt(r2))

		self.pitch_target = self.pitch_current + sa_theta - com_theta

		error = sa_theta - com_theta

		self.stdscr.addstr(5, 1, "pitch error: % 1.3f" % math.degrees(error))

		#  Filter error
		if self.pitch_error is None:
			self.pitch_error = error
			self.pitch_prev_error_filter = error

		self.pitch_error = error
		self.pitch_error_filter = error

		#  Controller
		if not self.is_waiting:
			self.pitch_integral += self.pitch_error * self.dt
		derivative = (self.pitch_error_filter - self.pitch_prev_error_filter) / self.dt
		cp = self.pitch_Kp * self.pitch_error + self.pitch_Ki * self.pitch_integral + self.pitch_Kd * derivative

		self.pitch_prev_error = self.pitch_error
		self.pitch_prev_error_filter = self.pitch_error_filter

		#  Apply
		self.pitch_command = self.pitch_current + cp

		#   Ankle roll
		s = (self.support_foot_tendency + 1.0)/2.0
		lf = (self.Q0toR.dot(self.nao.left_leg.P[-1]) + self.P0toR)[1] + 0.0053
		rf = (self.Q0toR.dot(self.nao.right_leg.P[-1]) + self.P0toR)[1] - 0.0053
		t = rf + (lf-rf)*s

		r2 = v[1]**2 + v[2]**2
		if self.support_foot_tendency >= 0.0:
			x = t-lf
		else:
			x = t-rf
		x = 0.0
		y = math.sqrt(r2 + x**2)

		com_theta = math.atan2(v[2], v[1])
		t_theta = math.atan2(y, x)

		error = com_theta - t_theta

		self.stdscr.addstr(6, 1, "roll error: % 1.3f" % math.degrees(error))

		if self.roll_error is None:
			self.roll_error = error
			self.roll_prev_error_filter = error

		self.roll_error = error
		self.roll_error_filter = error

		#  Controller
		if not self.is_waiting:
			self.roll_integral += self.roll_error * self.dt
		derivative = (self.roll_error_filter - self.roll_prev_error_filter) / self.dt
		cr = self.roll_Kp * self.roll_error + self.roll_Ki * self.roll_integral + self.roll_Kd * derivative

		self.roll_prev_error_filter = self.roll_error_filter

		#  Apply
		self.roll_target = self.roll_current + cr

		j_min = [-1.189516]
		j_max = [0.922747]
		tolerance = 0.001


		if not self.is_waiting:
			if self.support_foot_tendency >= 0.0:
				self.send[3][2] += self.support_foot_bend * -0.25
				self.send[3][3] += self.support_foot_bend * 0.5
				self.send[3][4] = self.angles[3][4] + cp
				self.send[4][4] = self.send[3][4] + self.send[3][2] + self.send[3][3] - self.send[4][2] - self.send[4][3]
				self.send[3][5] = self.angles[3][5] + float(cr)
				self.send[4][5] = self.send[3][5] + self.send[3][1] - self.send[4][1]
				self.send[0][1] = self.send[3][2] + self.send[3][3] + self.send[3][4]
			else:
				self.send[4][2] += self.support_foot_bend * -0.25
				self.send[4][3] += self.support_foot_bend * 0.5
				self.send[4][4] = self.angles[4][4] + cp
				self.send[3][4] = self.send[4][4] + self.send[4][2] + self.send[4][3] - self.send[3][2] - self.send[3][3]
				self.send[0][1] = self.send[4][2] + self.send[4][3] + self.send[4][4]

	def apply(self):
		"""Sends command data to the robot in one call to reduce the delay"""

		if self.is_waiting:
			return

		head = ["HeadYaw", "HeadPitch"]
		left_hand = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
		right_hand = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		left_leg = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll"]
		right_leg = ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"]


		self.motionProxy.setAngles(
			head+left_hand+right_hand+left_leg+right_leg,
			self.send[0]+self.send[1]+self.send[2]+self.send[3]+self.send[4], 0.8)

	def log(self):
		"""Logs necessary variables for later use"""

		#  Log all data
		log = self._log
		self._log = []

		#  Skeleton
		log.append(self.nao.head.P[2])
		log.append(self.nao.left_hand.P[0])
		log.append(self.nao.left_hand.P[4])
		log.append(self.nao.left_hand.P[5])
		log.append(self.nao.right_hand.P[0])
		log.append(self.nao.right_hand.P[4])
		log.append(self.nao.right_hand.P[5])
		log.append(self.nao.left_leg.P[0])
		log.append(self.nao.left_leg.P[3])
		log.append(self.nao.left_leg.P[5])
		log.append(self.nao.left_leg.P[6])
		log.append(self.nao.right_leg.P[0])
		log.append(self.nao.right_leg.P[3])
		log.append(self.nao.right_leg.P[5])
		log.append(self.nao.right_leg.P[6])

		log.append(self.com_sensor)
		log.append(self.nao.com)
		log.append(self.nao.head.com)
		log.append(self.nao.left_hand.com)
		log.append(self.nao.right_hand.com)
		log.append(self.nao.left_leg.com)
		log.append(self.nao.right_leg.com)

		#  Feet soles
		log.append(self.left_leg_points)
		log.append(self.right_leg_points)

		#  Gravity
		log.append(self.gravity + [0.0])

		#  Support polygon
		sa = []
		for p in self.support_area.convex:
			sa.append([p.x, p.y, 0.0])
		log.append(sa)
		log.append([self.support_area.mid.x, self.support_area.mid.y, 0.0])

		#  Balance
		log.append(self.Q0toR)
		log.append(self.P0toR)
		log.append(self.pitch_error)
		log.append(self.pitch_error_filter)

		log.append(self.pitch_current)
		log.append(self.pitch_command)
		log.append(self.pitch_target)
		log.append(self.roll_error)
		log.append(self.roll_error_filter)
		log.append(self.roll_current)
		log.append(self.roll_command)
		log.append(self.roll_target)

		#
		log.append(self.debug)

		if not first('log'):
			self.fo.write(',\n')
		self.fo.write(to_json(log))

	def log_time(self):
		"""Logs time for delay analysis"""
		self._log.append(time.time() - self.start)


_first_dict = {}
def first(id):
	"""This function returns True the first time it runs with a specific id

	Args:
		id: id

	Returns:
		True if it's the first time

	"""
	if id in _first_dict:
		return False
	else:
		_first_dict[id] = True
		return True

def to_json(data):
	"""Convert data to json format"""
	out = ""
	if type(data) is np.ndarray:
		if data.ndim == 2 and data.shape[1] == 1:
			data = data[:, 0]
		data = list(data)
	if type(data) is list:
		for d in data:
			if out != "":
				out += ","
			out += to_json(d)
		out = '[' + out + ']'
	elif type(data) is float or type(data) is np.float_:
		# out = '% 12.8f' % data
		out = '%.6f' % data
	elif type(data) is str:
		out = json.dumps(data)
	else:
		out = str(data)
	return out

def distance(A, B):
	"""Returns euclidean distance of two vectors (points)"""
	C = B - A
	return math.sqrt(C.T.dot(C))

def clamp(x, min, max):
	"""Returns a number clamped between a minimum and a maximum"""
	if x < min:
		x = min
	if x > max:
		x = max
	return x

def sign(x):
	"""Returns sign of a number"""
	if x is bool:
		if x:
			return 1.0
		return -1.0
	if x < 0:
		return -1.0
	return 1.0

def Rx(theta):
	"""Returns rotation matrix around X axis

	Args:
		theta (float): Rotation

	Returns:
		numpy.ndarray: R tation matrix

	"""
	sin = math.sin(theta)
	cos = math.cos(theta)
	return np.array([
		[1, 0,    0],
		[0, cos, -sin],
		[0, sin,  cos]])

def Ry(theta):
	"""Returns rotation matrix around Y axis

	Args:
		theta (float): Rotation

	Returns:
		numpy.ndarray: R tation matrix

	"""
	sin = math.sin(theta)
	cos = math.cos(theta)
	return np.array([
		[ cos, 0,  sin],
		[ 0,   1,  0],
		[-sin, 0,  cos]])

def Rz(theta):
	"""Returns rotation matrix around Z axis

	Args:
		theta (float): Rotation

	Returns:
		numpy.ndarray: R tation matrix

	"""
	sin = math.sin(theta)
	cos = math.cos(theta)
	return np.array([
		[cos, -sin, 0],
		[sin,  cos, 0],
		[0,    0,   1]])

def Euler(R):
	"""Get Euler representation of a transform matrix

	Args:
		R (numpy.ndarray): Transform matrix

	Returns:
		tuple: Euler representation

	"""
	if R[2][0] == -1.0:
		phi = 0
		theta = math.pi/2
		psi = math.atan2(R[0][1], R[0][2])
	elif R[2][0] == 1.0:
		phi = 0
		theta = -math.pi/2
		psi = math.atan2(-R[0][1], -R[0][2])
	else:
		theta = -math.asin(R[2][0])
		cos_theta = math.cos(theta)
		psi = math.atan2(R[2][1]/cos_theta, R[2][2]/cos_theta)
		phi = math.atan2(R[1][0]/cos_theta, R[0][0]/cos_theta)
	return psi, theta, phi


if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="127.0.0.1",
						help="Robot ip address")
	parser.add_argument("--port", type=int, default=9559,
						help="Robot port number")

	args = parser.parse_args()
	m = Main(args.ip, args.port)
	curses.wrapper(m.main)
