from math import *


def _trapeze_center_of_mass(x0, a, b, x3, height):
	if height ==0:
		return None
	# left triangle
	base = (a -x0)
	left_centroid = x0 + base / 3.
	left_area = height * base * 0.5
	# rectangle
	rect_centroid = a + (b-a) * 0.5
	rect_area = height * (b -a)
	# right triangle
	base = (x3 - b)
	right_centroid = b + base / 3.
	right_area = height * base * 0.5
	
	accum =  left_area * left_centroid +  rect_area * rect_centroid + right_area * right_centroid  
	total_area = left_area + rect_area + right_area
	
	centroid_x = accum / total_area
	return centroid_x, total_area
	
class mf:
	
	def __init__(self, p):
		'A constructor'
		
		self._p = p[:]
	
	
	
		
class trimf(mf):
	"""A triangular membership function. 
	Takes three points as constructor argument"""

	def evaluate(self,x):
		if x<=self._p[0]:
			return 0
		elif self._p[0] < x <= self._p[1]:
			return (x - self._p[0])/(self._p[1] - self._p[0])
		elif self._p[1] < x <= self._p[2]:
			return 1 - (x - self._p[1])/(self._p[2] - self._p[1])
		else:
			return 0
	
	def defuzz(self, height):
		""" Returns the center of mass position along x and area
		"""
		x0 = self._p[0]
		x1 = self._p[1]
		x2 = x1
		x3 = self._p[2]
		# xa and xb are the new summits of our trapeze at y = height 
		a = x0 + height * (x1 - x0)
		b = x3 - height * (x3 - x2)
		com = _trapeze_center_of_mass(x0, a, b, x3, height)
		return com
			
class trapmf(mf):
	"""A trapezoidal membership function
	Takes 4 points as constructor"""
	
	def evaluate(self,x):
		if x<=self._p[0]:
			return 0
		elif self._p[0] < x <= self._p[1]:
			return (x - self._p[0])/(self._p[1] - self._p[0])
		elif self._p[1] < x <= self._p[2]:				
			return 1
		elif self._p[2] < x <= self._p[3]:
			return 1 - (x - self._p[2])/(self._p[3] - self._p[2])
		else:
			return 0
		
	def defuzz(self, height):
		if height == 0:
			return (0,0)
		
		x0 = self._p[0]
		x1 = self._p[1]
		x2 = self._p[2]
		x3 = self._p[3]
		# xa and xb are the new summits of our trapeze at y = height 
		a = x0 + height * (x1 - x0)
		b = x3 - height * (x3 - x2)
		com = _trapeze_center_of_mass(x0, a, b, x3, height)
		return com
	
#class gaussmf(mf):
#	'A gaussian membership function'
#	
#	def evaluate(self,x):
#		return exp(-((x-self._p[0])/self._p[1])**2)

					