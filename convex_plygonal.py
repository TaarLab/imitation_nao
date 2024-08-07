import math
from operator import attrgetter


class Point:
	def __init__(self, x, y):
		self.x = float(x)
		self.y = float(y)

	def __repr__(self):
		return str((self.x, self.y))

	def __add__(self, other):
		return Point(self.x + other.x, self.y + other.y)

	def __sub__(self, other):
		return Point(self.x - other.x, self.y - other.y)

	def __neg__(self):
		return Point(-self.x, -self.y)

	def __mul__(self, other):
		return Point(self.x * other, self.y * other)

	def __div__(self, other):
		return Point(self.x / other, self.y / other)

	def distance(self, P):
		return math.sqrt((self.x - P.x)**2 + (self.y - P.y)**2)


class ConvexPolygonal:
	def __init__(self, *args):
		if len(args) < 3:
			raise TypeError("Not enough points (input less than 3 points)")

		# Graham Scan convex hull algorithm
		points = list(args)

		# get bottom-left point az pivot
		#points.sort(cmp = lambda p, q: (q.x-p.x if p.y==q.y else q.y-p.y))
		points.sort(key = attrgetter('y', 'x'), reverse = True)
		convex = [points.pop()]

		# sort by angle
		points.sort(cmp = lambda p, q: sign(orientation(convex[0], q, p)))

		# remove points with same angle
		i = 1
		while i < len(points):
			if orientation(convex[0], points[i-1], points[i]) == 0:
				if points[i].distance(convex[0]) >= points[i-1].distance(convex[0]):
					del points[i-1]
				else:
					del points[i]
			else:
				i += 1

		if len(points) < 2:
			raise TypeError("Not enough points (convex hull not possible)")

		convex.append(points.pop(0))
		convex.append(points.pop(0))

		# find convex hull
		for p in points:
			while orientation(convex[-2], convex[-1], p) < 0:
				convex.pop()
			convex.append(p)

		mid = Point(0, 0)
		for p in convex:
			mid += p
		self.mid = mid / len(convex)

		self.convex = convex

		self.scale = 1
		self.sconvex = convex
		self.border = 0

	def set_scale(self, scale=1):
		self.scale = scale
		mid = self.mid
		self.sconvex = [mid + (p - mid)*scale for p in self.convex]

	def set_border(self, border):
		self.border = border
		pass

	def is_inside(self, P):
		for i in range(len(self.sconvex)):
			if distance(self.sconvex[i-1], self.sconvex[i], P) < self.border:
				return False
		return True

	def get_distance(self, P):
		""" Positive values mean inside """
		min = distance(self.sconvex[-1], self.sconvex[0], P) - self.border
		for i in range(1, len(self.sconvex)):
			d = distance(self.sconvex[i - 1], self.sconvex[i], P) - self.border
			if d < min:
				min = d
		return min

	def __str__(self):
		return "<scale=%d, border=%d, %s" % (self.scale, self.border, str(self.convex))


def sign(x):
	if x > 0:
		return 1
	if x < 0:
		return -1
	return 0


def orientation(P0, P1, P2):
	""" Returns < 0 if points are clockwise, = 0 if collinear, > 0 if ccw """
	return (P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y)


def distance(P0, P1, P2):
	""" Returns the signed distance between P2 and P0P1 line, < 0 if points are clockwise and > 0 if ccw """
	return orientation(P0, P1, P2) / math.sqrt((P1.x - P0.x)**2 + (P1.y - P0.y)**2)


def test():
	# l1 = DirLine(Point(0,0),Point(1,0))
	# print l1.get_point_distance(Point(0,-1))
	# s = SupportArea(Point(0,0),Point(2,0),Point(2,2),Point(0,2))
	# print s.is_inside_border(Point(1, 1.5), 0.5)

	sa = ConvexPolygonal(Point(0, 0), Point(2, 2), Point(0, 2), Point(2, 0))
	sa.set_scale(2)
	sa.set_border(1)
	print sa.is_inside(Point(1, 1))
	print sa.get_distance(Point(11, 10))
	print sa

	# print type(np.array([]))


if __name__ == "__main__":
	test()
