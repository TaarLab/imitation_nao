
import numpy as np
import math


class NaoKinematics:
	def __init__(self):
		pi = math.pi
		self.head = Chain(
			[[0, 0, pi/2, 0],
			 [0, 0, 0,    0]],
			[[-0.00001, 0, -0.02742],
			 [-0.00112, 0,  0.05258]],
			[1, 2],
			[0.06442, 0.60533],
			[[-1,  0, 0],
			 [ 0, -1, 0],
			 [ 0,  0, 1]],
			[0, 0, 0.1265]
		)
		self.left_hand = Chain(
			[[0,        0,          pi/2,   0],
			 [0.015,    0,          pi/2,   pi/2],
			 [0,        0.105,      pi/2,   pi],
			 [0,        0,          pi/2,   pi],
			 [0,        0.05595,    0,      0]],
			[[-0.00165, -0.02663,  0.00014],
			 [ 0.02455,  0.00563,  0.00330],
			 [-0.02744,  0,       -0.00014],
			 [ 0.02556,  0.00281,  0.00076],
			 [ 0.03434, -0.00088,  0.00308]],
			[1, 1, 3, 4, 5],
			[0.07504, 0.15777, 0.06483, 0.07761, 0.18533],
			[[1,  0, 0],
			 [0,  0, 1],
			 [0, -1, 0]],
			[0, 0.098, 0.100]
		)
		self.right_hand = Chain(
			[[0,        0,          pi/2,   0],
			 [0.015,    0,         -pi/2,  -pi/2],
			 [0,        0.105,      pi/2,   0],
			 [0,        0,          pi/2,   pi],
			 [0,        0.05595,    0,      0]],
			[[-0.00165,  0.02663,  0.00014],
			 [ 0.02455, -0.00563,  0.00330],
			 [-0.02744,  0,       -0.00014],
			 [ 0.02556, -0.00281,  0.00076],
			 [ 0.03434,  0.00088,  0.00308]],
			[1, 1, 3, 4, 5],
			[0.07504, 0.15777, 0.06483, 0.07761, 0.18533],
			[[1,  0, 0],
			 [0,  0, 1],
			 [0, -1, 0]],
			[0, -0.098, 0.100]
		)
		t = math.sqrt(2)/2
		self.left_leg = Chain(
			[[0,        0,  pi/2,  0],
			 [0,        0,  pi/2, -3*pi/4],
			 [0.100,    0,  0,     pi],
			 [0.1029,   0,  0,     0],
			 [0,        0,  pi/2,  0],
			 [0.04519,  0,  0,     0]],
			[[-0.00781, -0.01114,  0.02661],
			 [-0.01549,  0.00029, -0.00515],
			 [ 0.00138,  0.00221, -0.05373],
			 [ 0.00453,  0.00225, -0.04936],
			 [ 0.00045,  0.00029,  0.00685],
			 [ 0.02542,  0.00330, -0.03239]],
			[1, 2, 2, 3, 5, 5],
			[0.06981, 0.13053, 0.38968, 0.29142, 0.13416, 0.16184],
			[[ 0, -1,  0],
			 [-t,  0,  t],
			 [-t,  0, -t]],
			[0, 0.050, -0.085]
		)
		self.right_leg = Chain(
			[[0,        0,  pi/2,  0],
			 [0,        0,  pi/2,  3*pi/4],
			 [0.100,    0,  0,     pi],
			 [0.1029,   0,  0,     0],
			 [0,        0,  pi/2,  0],
			 [0.04519,  0,  0,     0]],
			[[-0.00781,  0.01114,  0.02661],
			 [-0.01549, -0.00029, -0.00515],
			 [ 0.00138, -0.00221, -0.05373],
			 [ 0.00453, -0.00225, -0.04936],
			 [ 0.00045, -0.00029,  0.00685],
			 [ 0.02542, -0.00330, -0.03239]],
			[1, 2, 2, 3, 5, 5],
			[0.06981, 0.13053, 0.38968, 0.29142, 0.13416, 0.16184],
			[[ 0, -1, 0],
			 [ t,  0, t],
			 [-t,  0, t]],
			[0, -0.050, -0.085]
		)

		self.torso_mass = 1.0496
		self.torso_com = np.array([-0.00413, 0, 0.04342])[:,np.newaxis]

		self.mass = self.torso_mass + self.head.mass + self.left_hand.mass + self.right_hand.mass + \
		            self.left_leg.mass + self.right_leg.mass

		self.com = None

	def setTheta(self, theta):
		self.head.setTheta(theta[0])
		self.left_hand.setTheta(theta[1])
		self.right_hand.setTheta(theta[2])
		self.left_leg.setTheta(theta[3])
		self.right_leg.setTheta(theta[4])

		# temporary
		com = self.torso_mass*self.torso_com + self.head.mass*self.head.com + \
		      self.left_hand.mass*self.left_hand.com + self.right_hand.mass*self.right_hand.com + \
		      self.left_leg.mass*self.left_leg.com + self.right_leg.mass*self.right_leg.com

		self.com = com / self.mass


class Chain:
	def __init__(self, dh, com_pos, com_joint, link_mass, base_q, base_a):
		# check arguments for error
		self.len = len(dh)
		if self.len < 2 or self.len != len(com_pos) or self.len != len(link_mass):
			raise TypeError("Bad input")

		self._q = []
		self._a = []
		self._theta = []

		for i in range(self.len):
			sin = math.sin(dh[i][2])
			cos = math.cos(dh[i][2])
			self._q.append(np.array([
				[1, -cos,  sin],
				[1,  cos, -sin],
				[0,  sin,  cos],
			]))
			self._a.append(np.array([
				[dh[i][0]],
				[dh[i][0]],
				[dh[i][1]],
			]))
			self._theta.append(dh[i][3])


		self._base_q = np.array(base_q)
		self._base_a = np.array(base_a)[:, np.newaxis]

		self.q = []
		self.a = []
		self.Q = []
		self.P = []

		self._com = None
		self.setTheta()

		self._com = []
		self._mass = link_mass

		for i in range(self.len):
			if com_joint[i] > i+1:
				raise TypeError("Bad com_joint")

			com = self.Q[com_joint[i]].T.dot(np.array(com_pos[i])[:, np.newaxis])
			for j in range(com_joint[i], (i+1)):
				com = self.q[j].T.dot(com - self.a[j])
			self._com.append(com)

		self.com = None
		self.mass = sum(link_mass)
		self.setTheta()

	def setTheta(self, theta=None):
		if theta is None:
			theta = [0 for i in range(self.len)]

		self.Q = [self._base_q]
		self.P = [self._base_a]

		for i in range(self.len):
			sin = math.sin(theta[i]+self._theta[i])
			cos = math.cos(theta[i]+self._theta[i])
			q = np.array([
				[cos, sin, sin],
				[sin, cos, cos],
				[0,   1,   1],
			])
			a = np.array([
				[cos],
				[sin],
				[1],
			])
			q *= self._q[i]
			a *= self._a[i]
			self.q.append(q)
			self.a.append(a)

			self.Q.append(self.Q[i].dot(q))
			self.P.append(self.P[i] + self.Q[i].dot(a))

		if self._com is not None:
			com = np.zeros((3,1))
			for i in range(self.len):
				com += self._mass[i] * (self.P[i+1] + self.Q[i+1].dot(self._com[i]))
			self.com = com / self.mass

	def test(self):
		for i in range(self.len):
			print (self.P[i + 1] + self.Q[i + 1].dot(self._com[i]))


def test():
	np.set_printoptions(precision=8, suppress=True)
	nao = NaoKinematics()
	chain = Chain([dh], com_pos, com_joint, link_mass, base_q, base_a)
	nao.setTheta([[0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])
	# print nao.left_hand.test()
	# print time.clock()
	# nao.setTheta([[0,0],[1,1,1,1,1],[0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])
	# print time.clock()
	#print nao.left_hand.test()
	for t in nao.right_leg._com:
		print t


if __name__ == "__main__":
	test()