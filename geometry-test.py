
import time
import sys, pygame

from geometry import * # geometry.py

modifyVector = Vector2.zero()
modifyVector.y = 10
print("Checking if Vector2 is a value type, should be 0,0: ", Vector2.zero())


# Vector2 rotate test
# -------------------
# v = Vector2(10,0)
# for i in range(1, 1000):
# 	v.rotate(rad(90))
# 	print(v)
# 	print(v.magnitude())

# Vector2 snap issues
# -------------------
if 0:
	a = Vector2(0.4,0.9)
	b = Vector2(-0.4,-0.9)

	dot = Vector2.dot(a, b)
	print("a.b = ", dot)
	cos = dot / (a.magnitude() * b.magnitude()) 	# [TODO] More efficient way?
	print("cos(a.b / |a||b|) =", cos)
	print(math.acos(cos))

# Vector2 bearing test
# --------------------
headings = [Vector2(0,1), Vector2(2,0), Vector2(-3,0), Vector2(0,-4)]
for heading in headings:
	print(heading, "->", deg(heading.bearing()), "deg")

# GPS locale test
# ---------------
print("GPS Locale test")

origin = GPSPosition(51.242346, -0.590729) 	# Lampost outside TB, CC on campus
locale = GPSLocale(origin)

print("origin", origin)

originToV = locale.toVector(origin)
originToVToGPS = locale.toGPS(originToV)

print("origin converted to vector and back:", originToV, originToVToGPS)

shift100East = originToV + Vector2(100, 0)
shift100EastGPS = locale.toGPS(shift100East)

print("shifted 100m east:", shift100EastGPS)

shift100North = originToV + Vector2(0, 100)
shift100NorthGPS = locale.toGPS(shift100North)

print("shifted 100m north:", shift100NorthGPS)


# Graphical rect test
# -------------------
if 0:
	print("Showing rectangle test")
	pygame.init()

	font = pygame.font.SysFont('Helvetica', 13)

	size = width, height = 480, 360
	centre = Vector2(width, height) / 2
	rect = Rect2.aabb(centre, Vector2(100, 70))
	w = 60
	h = 35

	# Functions
	def pygDrawRect(rect, color=(255,255,255)):
		corners = []
		for i in range(1,5): # 1-4
			corners.append(rect.corner(i).toPixel())

		pygame.draw.lines(screen, color, 1, corners) 	# rectangle

	def pygDrawText(text, color, pos):
		textsurface = font.render(text, False, color)
		screen.blit(textsurface, pos)

	# Colours
	red = (255,0,0)
	green = (0,255,0)
	blue = (0,0,255)
	darkBlue = (0,0,128)
	white = (255,255,255)
	black = (0,0,0)
	pink = (255,200,200)

	screen = pygame.display.set_mode(size)

	while 1:
		for event in pygame.event.get():
			if event.type == pygame.QUIT: sys.exit()

		# pack rectangles
		imageRects = Rect2.packUniform(rect, w, h)

		# clear screen
		screen.fill(black)

		# debug text
		pygDrawText('rects: ' + str(len(imageRects)), white, (5,5))

		# axes
		pygame.draw.line(screen, blue, 	rect.center.toPixel(), (rect.center + rect.primary).toPixel()) 	# primary axis
		pygame.draw.line(screen, red,  	rect.center.toPixel(), (rect.center + rect.secondary()).toPixel()) 	# secondary axis

		# search area
		pygDrawRect(rect, red)

		# image rects
		for ir in imageRects:
			pygame.draw.circle(screen, green, ir.center.toPixel(), 2)
			pygDrawRect(ir, pink)

		pygame.display.update()

		time.sleep(1/60)

		# modify scene for next run
		rect.rotate(rad(1))
		w /= 1.001
		h /= 1.001
