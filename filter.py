
class Filter(object):
	def __init__(self, alpha=1.0, initval=None):
		self.val = initval
		self.alpha = 1.0
		self.ralpha = 0.0
		self.set_alpha(alpha)

	def set_alpha(self, alpha):
		if alpha < 0.0:
			alpha = 0.0
		elif alpha > 1.0:
			alpha = 1.0
		self.alpha = alpha
		self.ralpha = 1.0 - alpha

	def set(self, obj, val):
		self.val = self.alpha * val + self.ralpha * self.val

	def reset(self, val):
		self.val = val

	def __get__(self, obj, objtype):
		return self.val

	def __set__(self, obj, val):
		if self.val is None:
			self.val = val
		else:
			self.set(obj, val)
		self.__set__ = self.set


def test():
	class Test(object):
		def __init__(self):
			self.f = Filter(0.5)
	t = Test()
	t.f = 5.3
	t.f = 5.0
	t.f = 5.0
	print t.f


if __name__ == "__main__":
	test()