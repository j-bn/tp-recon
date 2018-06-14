#!/bin/env python3.6
# -*- encoding: utf-8 -*-

"""
geometry.py: Provides a library for 2D and GPS gemetric operations.
Part of the reconnaissance system for Team Peryton's (University of Surrey) entry to the IMechE's 2018 UAS Competition.
"""

__author__ = "James Thornton"

import math, random

# Constants
pi = math.pi
tau = 2 * pi

# Utility functions
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def deg(radians):
	# [wiki] As stated, one radian is equal to 180/π degrees. Thus, to convert from radians to degrees, multiply by 180/π.
	return radians * 180 / pi

def rad(degrees):
	# [wiki] Conversely, to convert from degrees to radians, multiply by π/180.
	return degrees * pi / 180

class Vector2:
	# Represents a vector/point in 2D space
	# similar to https://docs.unity3d.com/ScriptReference/Vector2.html

	def __init__(self, x=0, y=0):
		self.x = x
		self.y = y

	# Static definitions
	def angleVector(a):
		return Vector2(math.cos(a), math.sin(a))

	def randomOnUnitCircle():
		return Vector2.angleVector(random.random() * tau)

	def randomInUnitCircle():
		return Vector2.randomOnUnitCircle() * random.random()

	def zero():
		return Vector2(0,0)

	def one():
		return Vector2(1,1)

	# Mutators
	def rotate(self, a):
		# rotate a vector CCW by a radians		
		
		x2 = math.cos(a) * self.x - math.sin(a) * self.y
		y2 = math.sin(a) * self.x + math.cos(a) * self.y

		# avoid modifying x before calculation of y
		self.x = x2
		self.y = y2

	# Properties
	def alignedWith(self, v):
		# Returns a vector with the same length aligned with v
		return v.normalized() * self.magnitude() 	# [TODO] could be more efficient using sqrMagnitude?

	def perpendicular(self):
		return Vector2(self.y, -self.x)

	def sqrMagnitude(self):
		return self.x ** 2 + self.y ** 2

	def magnitude(self):
		return self.sqrMagnitude() ** 0.5

	def normal(self):
		return Vector2(-self.y, self.x)

	def normalized(self):
		return self / self.magnitude()

	def toPixel(self):
		return [int(round(self.x)), int(round(self.y))]

	def bearing(self):
		# Returns the bearing in radians of the vector, clockwise relative to north (0=N=+y)
		# Atan2 returns atan(y / x), in radians between -pi and +pi
		return math.atan2(self.x, self.y)

	# Static methods
	def dot(l, r):
		return l.x * r.x + l.y * r.y

	def angle(a, b):
		# Returns the unsigned angle in radians between from and to
		cos = Vector2.dot(a, b) / (a.magnitude() * b.magnitude()) 	# [TODO] More efficient way?
		cc = clamp(cos, -1, 1) # Fix rounding errors
		return math.acos(cc)

	def snap(a, b):
		# Snap direction vector a on to the closest vector parralel to b (+b or -b)
		vp = b
		vn = -b
		ap = Vector2.angle(a, vp)
		an = Vector2.angle(a, vn)

		if ap < an:
			# +b vector is closest
			return a.alignedWith(vp)
		else:
			# -b vector is closest
			return a.alignedWith(vn)


	# Operators
	def __str__(self):
		# to 1/1000 = 0.001 (mm) accuracy
		return "(" + str(round(self.x,3)) + "," + str(round(self.y,3)) + ")"

	def __add__(self, b):
		return Vector2(self.x + b.x, self.y + b.y)

	def __sub__(self, b):
		return Vector2(self.x - b.x, self.y - b.y)

	def __mul__(self, b):
		t = type(b)
		if t is Vector2:
			return Vector2(self.x * b.x, self.y * b.y)
		elif t is int or t is float:
			return Vector2(self.x * b, self.y * b)

	__rmul__ = __mul__ # b * Vector2

	def __truediv__(self, b): # / returning a float, as opposed to // which returns an integer
		t = type(b)
		if t is Vector2:
			return Vector2(self.x / b.x, self.y / b.y)
		elif t is int or t is float:
			return Vector2(self.x / b, self.y / b)

	def __neg__(self):
		return Vector2(-self.x, -self.y)

class Rect2:
	# Represents a rectangle of any rotation in 2D space
	# Defined in terms of a center, primary vector and aspect ratio

	# A-----B
	# |  C  |
	# D-----D
	#
	# AB/2 = primary
	# AR = W/H

	# Main constructor
	def __init__(self, center=Vector2.zero(), primary=Vector2.zero(), aspect=1):
		self.center = center
		self.primary = primary
		self.aspect = aspect

	# Static definitions
	def aabb(center, size):
		# size is actually extents
		ar = size.x / size.y
		return Rect2(center, Vector2(size.x, 0), ar)

	# Mutators
	def rotate(self, a):
		# rotate a vector CCW by a radians (same as Vector2)	
		self.primary.rotate(a)
		return self

	# Properties
	def width(self):
		# defined as the full length of the rectangle in the primary axis
		return self.primary.magnitude() * 2

	def height(self):
		# defined as the full length of the rectangle in the secondary axis
		return self.width() / self.aspect # [TODO] more efficient than secondary.magnitude?

	def size(self):
		return Vector2(self.width(), self.height())

	def area(self):
		return self.width()**2 / self.aspect

	def secondary(self):
		return self.primary.normal() / self.aspect

	def corner(self, n):
		# Returns point 1-4 : A-D proceding from the 'top-left' clockwise around the section

		xn = [-1,+1,+1,-1]
		yn = [-1,-1,+1,+1]

		i = n-1
		x = xn[i]
		y = yn[i]

		return self.center + x * self.primary + y * self.secondary()

	def alignedWith(self, v):
		# Returns an identical rect rotated so that it's primary axis is aligned with v
		pri = self.primary.alignedWith(v)
		return Rect2(self.center, pri, self.aspect)

	# Operators
	def __add__(self, b):
		return Rect2(self.center + b, self.primary, self.aspect)

	def __mul__(self, b):
		t = type(b)
		if t is int or t is float:
			return Rect2(self.center * b, self.primary * b, self.aspect)

	def __str__(self):
		return "Rectangle centred at " + str(self.center) + " with " + str(round(self.primary.bearing(), 2)) + " deg rotation and size " + str(self.size())

	# Packing
	def packUniform(container, w, h, returnMode=0):
		# In 2D space, aproximates the most efficient grid packing of a uniform rectangles WxH
		# to fully cover a 'container' rectangle, with overlap
		# returns a Rect2 list defining all the packed rectangles

		# assumes x,w in primary direction and y,h in secondary (to avoid aircraft having to turn to capture images)

		# return modes: 0 = rects, 1 = centers, 2 = waypoints

		vc = container.center
		vp = container.primary
		vs = container.secondary()

		nvp = vp.normalized()
		nvs = vs.normalized()

		lp = vp.magnitude()
		ls = vs.magnitude()

		# m = primary, n = secondary
		# should be ceil but floor seems to work - cause must be somewhere else
		m = math.ceil(lp * 2 / w)
		n = math.ceil(ls * 2 / h)

		# grid size
		gw = m * w
		gh = n * h

		# construct grid
		out = []
		for i in range(m): # 0 - m-1
			for j in range(n): # 0 - n-1
				# in rectangle space
				x = w * i - (gw/2) + (w/2)
				y = h * j - (gh/2) + (h/2) 

				rc = vc + nvp * x + nvs * y

				if returnMode == 1: # return centers
					out.append(rc)
				elif returnMode == 2: # return centres and headings
					out.append((rc, nvs))
				else: # return rects
					r = Rect2(rc, nvp * w / 2, w/h)
					out.append(r)

		return out

class GPSPosition:
	# Defines a GPS position by lat, long
	# Lattiude and longditude are measured in degrees from the equator and prime meridian, respectively
	# East and north are considered positive

	def __init__(self, lat, lon):
		self.lat = lat
		self.lon = lon

	def __str__(self):
		# 1 degree = ~111m, so to achieve ~mm (0.001m) accuracy, GPS will have to be ~0.00001 deg (5dp)
		return "(" + str(round(self.lat,5)) + "," + str(round(self.lon,5)) + ")"

class GPSLocale:
	# Enables conversion between GPS and cartesian co-ordinates in a local area

	# Average radius of earth
	r = 6371000 # m

	# Constructor
	def __init__(self, gpsOrigin):
		self.gpsOrigin = gpsOrigin

		# Calculate conversion factors
		r = self.r
		lat = rad(gpsOrigin.lat)
		rx = r * math.cos(lat) 	# Radius of horizontal section at origin lattitude 
		cx = 2 * pi * rx 		# Perimeter of circle rx (the horizontal slice)
		cy = 2 * pi * r 		# Perimeter of circle r (the full earth)
		self.cfx = cx / 360		# Number of metres per 1 degree change in longditude
		self.cfy = cy / 360 	# Number of metres per 1 degree change in lattitude

	def toVector(self, gps):
		dLat = gps.lat - self.gpsOrigin.lat
		dLon = gps.lon - self.gpsOrigin.lon
		dX = dLon * self.cfx
		dY = dLat * self.cfy

		return Vector2(dX, dY)

	def toGPS(self, vector):
		dX = vector.x
		dY = vector.y
		dLon = dX / self.cfx 
		dLat = dY / self.cfy
		lon = dLon + self.gpsOrigin.lon
		lat = dLat + self.gpsOrigin.lat

		return GPSPosition(lat, lon)