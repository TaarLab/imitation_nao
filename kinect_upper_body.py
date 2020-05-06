# -*- encoding: UTF-8 -*-
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

import argparse
import motion
import almath
from naoqi import ALProxy

import rospy
import tf

class Main:
	def __init__(self, ip, port=9559):
		self.ip = ip
		self.port = port
		self.start = time.time()
		self.fo = open("joint_angles.txt", "w")
		print self.ip, self.port
		self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
		self.postureProxy = ALProxy("ALRobotPosture", self.ip, self.port)

		self.right_elbow = None
		self.right_hand = None
		self.left_elbow = None
		self.left_hand = None

		self.right_knee = None
		self.right_foot = None
		self.left_knee = None
		self.left_foot = None

		self.stdscr = None

	def main(self, stdscr):
		self.stdscr = stdscr
		self.stdscr.nodelay(True)
		np.set_printoptions(precision=5, suppress=True)

		self.motionProxy.wakeUp()

		self.postureProxy.goToPosture("Crouch", 1.0)
		# self.postureProxy.goToPosture("StandInit", 1.0)
		# angles = [0.0, -0.2, 0.0, 0.5, -0.3, 0.0]
		# names = ["RHipYawPitch", "RHipPitch", "RHipRoll", "RKneePitch", "RAnklePitch", "RAnkleRoll"]
		# self.motionProxy.setAngles(names, angles, 0.1)
		# names = ["LHipYawPitch", "LHipPitch", "LHipRoll", "LKneePitch", "LAnklePitch", "LAnkleRoll"]
		# self.motionProxy.setAngles(names, angles, 0.1)
		time.sleep(1)
		self.fo.write(self.motionProxy.getSummary())

		rospy.init_node('Test1')
		rate = rospy.Rate(20)
		fixed_frame = 'openni_depth_frame'
		tf_listener = tf.TransformListener()
		tf_listener.waitForTransform(fixed_frame, fixed_frame, rospy.Time(), rospy.Duration(60.0))

		key = ''
		while key != ord('q'):

			self.stdscr.clear()

			self.stdscr.addstr(1, 1, "Started. Hit 'q' to quit")
			key = self.stdscr.getch()

			# stdscr.addch(20, 25, key)
			# if key == curses.KEY_UP:
				# stdscr.addstr(2, 20, "Up")
			# elif key == curses.KEY_DOWN:
				# stdscr.addstr(3, 20, "Down")

			#  List all available joints
			# stdscr.addstr(7, 1, str(tf_listener.getFrameStrings()))

			try:
				# Get left hand positions
				hand_alpha = 0.3
				leg_alpha = 0.2

				(right_shoulder, _) = tf_listener.lookupTransform('torso', 'right_shoulder', rospy.Time(0))
				(right_elbow, _) = tf_listener.lookupTransform('torso', 'right_elbow', rospy.Time(0))
				(right_hand, _) = tf_listener.lookupTransform('torso', 'right_hand', rospy.Time(0))

				right_shoulder = np.array(right_shoulder)
				right_elbow = np.array(right_elbow) - right_shoulder
				right_hand = np.array(right_hand) - right_elbow - right_shoulder

				if self.right_elbow is None:
					self.right_elbow = right_elbow
				if self.right_hand is None:
					self.right_hand = right_hand

				self.right_elbow = hand_alpha*right_elbow + (1-hand_alpha)*self.right_elbow
				self.right_hand = hand_alpha*right_hand + (1-hand_alpha)*self.right_hand

				# self.stdscr.addstr(7, 1, str([self.right_elbow, self.right_hand]))
				self.move_hand([self.right_elbow, self.right_hand], True)



				(left_shoulder, _) = tf_listener.lookupTransform('torso', 'left_shoulder', rospy.Time(0))
				(left_elbow, _) = tf_listener.lookupTransform('torso', 'left_elbow', rospy.Time(0))
				(left_hand, _) = tf_listener.lookupTransform('torso', 'left_hand', rospy.Time(0))

				left_shoulder = np.array(left_shoulder)
				left_elbow = np.array(left_elbow) - left_shoulder
				left_hand = np.array(left_hand) - left_elbow - left_shoulder

				if self.left_elbow is None:
					self.left_elbow = left_elbow
				if self.left_hand is None:
					self.left_hand = left_hand

				self.left_elbow = hand_alpha*left_elbow + (1-hand_alpha)*self.left_elbow
				self.left_hand = hand_alpha*left_hand + (1-hand_alpha)*self.left_hand

				# self.stdscr.addstr(9, 1, str([left_elbow, left_hand]))
				self.move_hand([self.left_elbow, self.left_hand])



				(right_hip, _) = tf_listener.lookupTransform('torso', 'right_hip', rospy.Time(0))
				(right_knee, _) = tf_listener.lookupTransform('torso', 'right_knee', rospy.Time(0))
				(right_foot, _) = tf_listener.lookupTransform('torso', 'right_foot', rospy.Time(0))

				right_hip = np.array(right_hip)
				right_knee = np.array(right_knee) - right_hip
				right_foot = np.array(right_foot) - right_knee - right_hip

				if self.right_knee is None:
					self.right_knee = right_knee
				if self.right_foot is None:
					self.right_foot = right_foot

				self.right_knee = leg_alpha*right_knee + (1-leg_alpha)*self.right_knee
				self.right_foot = leg_alpha*right_foot + (1-leg_alpha)*self.right_foot

				# self.stdscr.addstr(7, 1, str([right_knee, right_foot]))
				# self.move_leg([self.right_knee, self.right_foot], True)



				(left_hip, _) = tf_listener.lookupTransform('torso', 'left_hip', rospy.Time(0))
				(left_knee, _) = tf_listener.lookupTransform('torso', 'left_knee', rospy.Time(0))
				(left_foot, _) = tf_listener.lookupTransform('torso', 'left_foot', rospy.Time(0))

				left_hip = np.array(left_hip)
				left_knee = np.array(left_knee) - left_hip
				left_foot = np.array(left_foot) - left_knee - left_hip

				if self.left_knee is None:
					self.left_knee = left_knee
				if self.left_foot is None:
					self.left_foot = left_foot

				self.left_knee = leg_alpha*left_knee + (1-leg_alpha)*self.left_knee
				self.left_foot = leg_alpha*left_foot + (1-leg_alpha)*self.left_foot

				# self.stdscr.addstr(9, 1, str([left_knee, left_foot]))
				# self.move_leg([self.left_knee, self.left_foot])

			except:
				stdscr.addstr(18, 1, "Unexpected error: " + str(sys.exc_info()[1]))
				stdscr.addstr(4, 1, traceback.format_exc())

			stdscr.refresh()
			rate.sleep()

		self.postureProxy.goToPosture("Crouch", 1.0)
		# self.motionProxy.rest()

	def move_hand(self, hand_joints, right_hand=False):
		debug = np.zeros((3,2))

		if right_hand:
			names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"]
		else:
			names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
		limited = []

		hand_joints = np.array(hand_joints, dtype=float)

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
		# q = np.array([
		# 	[1, 0,  0],
		# 	[0, 0, 1],
		# 	[0, -1, 0],
		# ], dtype=float)
		q = np.array([
			[0, 0,  -1],
			[0, -1, 0],
			[-1, 0, 0],
		], dtype=float)

		v = hand_joints[0][:,np.newaxis]
		v = v * math.sqrt(l[0]*l[0] + l[1]*l[1]) / math.sqrt(v.T.dot(v))
		p = q.T.dot(v)
		debug[:, 0:1] = p

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
		# v = v * math.sqrt((l[2]+l[3])*(l[2]+l[3]) + l[4]*l[4]) / math.sqrt(v.dot(v))
		v = v * l[2] / math.sqrt(v.T.dot(v))
		p = q1.T.dot(q0.T.dot(q.T.dot(v)))
		debug[:, 1:2] = p

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

		# self.stdscr.addstr(7 + int(right_hand) * 4, 1, str(debug))
		# print " ".join('%0.5f' % f for f in angles)
		if len(limited) > 0:
			self.stdscr.addstr(18 + int(right_hand), 1, "Joints hslimited: " + str(limited))
			# return

		fractionMaxSpeed = 0.5
		self.motionProxy.setAngles(names, angles, fractionMaxSpeed)


	def move_leg(self, leg_joints, right_leg=False):
		debug = np.zeros((3,2))

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
		angles = [0.0, -0.45, 0.0, 0.7, -0.35, 0.0]		#StandInit
		angles = [0.0, 0.0, 0.0, 0.4, -0.4, 0.0]		#StandZero

		#  Remap first joint to nao
		q = np.array([
			[0, 0,  -1],
			[0, 1, 0],
			[1, 0, 0],
		], dtype=float)

		v = leg_joints[0][:, np.newaxis]
		v = v * l[0] / math.sqrt(v.T.dot(v))
		p = q.T.dot(v)
		debug[:, 0:1] = p

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
		debug[:, 1:2] = p

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

		# if right_leg:
		# 	angles[4] += -angles[1] -angles[3]
		# else:
		# 	angles[4] += -angles[1] -angles[3]

		if right_leg:
			angles[5] += angles[2]
		else:
			angles[5] += angles[2]


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

		# self.stdscr.addstr(7 + int(right_leg) * 4, 1, str(debug))
		self.stdscr.addstr(7 + int(right_leg) * 4, 1, " ".join('%0.5f' % f for f in angles))
		if len(limited) > 0:
			self.stdscr.addstr(16 + int(right_leg), 1, "Joints limited: " + str(limited))
			# return

		fractionMaxSpeed = 0.2
		self.motionProxy.setAngles(names, angles, fractionMaxSpeed)

		# t = 1.0
		# times = [t, t, t, t, t]
		# isAbsolute = True
		# motionProxy.angleInterpolation(names, angles, times, isAbsolute)
		# time.sleep(t)

		# useSensors = True
		# sensors = motionProxy.getAngles(names, useSensors)
		#
		# if not right_hand:
		# 	fo.write("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n" % (time.time(), angles[0], angles[1], angles[2], angles[3], sensors[0], sensors[1], sensors[2], sensors[3]))

def clamp(x, min, max):
	if x < min:
		x = min
	if x > max:
		x = max
	return x

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("--ip", type=str, default="127.0.0.1",
						help="Robot ip address")
	parser.add_argument("--port", type=int, default=9559,
						help="Robot port number")

	args = parser.parse_args()
	m = Main(args.ip, args.port)
	curses.wrapper(m.main)
