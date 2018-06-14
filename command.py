#!/bin/env python2.7
# -*- encoding: utf-8 -*-

"""
command.py: The main script of the reconnaissance system, incorporating aircraft guidance; image handling; and interface abstraction.
Part of the reconnaissance system for Team Peryton's (University of Surrey) entry to the IMechE's 2018 UAS Competition.
"""

__author__ = "James Thornton"

# Import modules etc.
import time, datetime
import sys
import pygame
from geometry import * # geometry.py
from recon import * # recon.py
import random
import concurrent.futures
import io
from fcinterface.dronekit_interface import FCInterface

isLinux = sys.platform.startswith('linux')
if isLinux:
	from picamera import PiCamera
	import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library

pyVersion = sys.version
print("command.py running on Py", pyVersion)

# General Notes
# 
# Arturo: UAS will fly somewhat like a aeroplane - keeping heading roughly in line with velocity
# 		probably can't give a waypoint a heading, but can set heading after arrival
# 		(UAS can yaw on the spot by using counter-rotating motors)


# TODO:
# [DONE] Have Pi stop sending commands to Pixhawk if manual override is engaged (i.e. check before sending any commands that mode is still guided)
# [DONE] Setup a safe auto-launch - have a predefined setup profile, on startup Pi connects and waits for pixhawk to be armed, and then takes control and takes off

# Utility functions
# -----------------

def exit():
	logFile.close()
	
	if isLinux:
		GPIO.cleanup() # Clean up
	
	realMission = isLinux and not simulateFCInterface and not enableDisplay and not simulateCamera and not enableSITL
	if realMission:
		log("Real mission detected, shutting down...")
		# sends shutdown command to host
		shutdownCmd = "/usr/bin/sudo /sbin/shutdown -r now"
		subprocess.Popen(shutdownCmd.split(), stdout=subprocess.PIPE)
	else:
		log("Non-real mission, exiting program...")
	
	sys.exit()
	# sys.exit is better practice than exit or quit
	# [https://stackoverflow.com/questions/19747371/python-exit-commands-why-so-many-and-when-should-each-be-used/19747562]

def log(*objects, end='\n'):
	# print(*objects, sep=' ', end='\n', file=sys.stdout, flush=False)

	print(*objects, end=end) 				# to stdout

	if logFile:
		print(*objects, end=end, file=logFile) 	# to log file

def header(s):
	return "\n" + underline(s)

def underline(s):
	return s + "\n" + (len(s) * "-")

def randomSearchAreas(n):
	areas = []

	for i in range(n):
		area = 40000
		width = 100 + random.random() * 200
		height = area / width
		ar = width / height

		center = Vector2.randomOnUnitCircle() * 600
		primary = Vector2.randomOnUnitCircle() * width / 2

		rect = Rect2(center, primary, ar)
		areas.append(rect)

	return areas

def frustrumFieldRect(alt, fovP, fovS):
	# defined in terms of primary, secondary axes
	# both position and orientation set to zero, as this is used in the aircraft reference frame
	pos = Vector2.zero()
	priDir = Vector2(0,1)

	# extents
	ep = 2 * alt * math.tan(fovP/2)
	es = 2 * alt * math.tan(fovS/2)

	pl = ep / 2
	ar = ep / es

	p = priDir.normalized() * pl

	return Rect2(pos, p, ar)

def createSearchAreas(n):
	global inputSearchAreas, gpsLocale
	
	inputSearchAreas = overrideBoolInput('manual SA input', inputSearchAreas)

	if inputSearchAreas: # manually input search areas
		log(header("Search Area Setup"))

		# [TODO] Handle multiple input formats
		# [TODO] Handle GPS and cartesian input formats

		rects = [None] * n

		coordMode = 'gps' if inputC(" Enter Y tp use spatial GPS coordinates: ").lower() == "y" else 'cartesian'

		for i in range(0,n):
			log(" Search area #" + str(i+1))
			log("  Centre position")
			if coordMode == 'gps':
				gps = inputGPSPos()
				vec = gpsLocale.toVector(gps)
				cx = vec.x
				cy = vec.y
			elif coordMode == 'cartesian':
				cx = float(inputC("  X coord (m) = "))
				cy = float(inputC("  Y coord (m) = "))
			an = float(inputC("  Rotation (CW, deg) = "))
			sp = float(inputC("  Full size in Primary axis (m) - 'width' when not rotated = "))
			ss = float(inputC("  Full size in Secondary axis (m) - 'height' when not rotated = "))

			center = Vector2(cx, cy)
			size = Vector2(sp, ss) / 2
			rects[i] = Rect2.aabb(center, size).rotate(-rad(an))

		log(rects)
		return rects
	else:
		return randomSearchAreas(n)

def overrideBoolInput(name, default):
	ovr = inputC("Enter [Y/n] to override {}: ".format(name))
	if ovr:
		if ovr.lower() == 'y':
			return 1
		else:
			return 0
	else:
		#log('defaulted to ' + str(default), end='')
		return default

def overrideInput(name, default):
	ovr = inputC("Enter value to override {}: ".format(name))
	if ovr:
		return ovr
	else:
		return default

def loadFileLines(filename):
	file = open(filename)
	lines = file.read().splitlines() 
	file.close()

	return lines

def inputGPSPos():
	log("  Enter GPS position (degrees)")
	lat = float(inputC("   Lat:"))
	lon = float(inputC("   Lon:"))
	return GPSPosition(lat, lon)

# Setup Log File
# --------------

# open('file.txt', 'w', bufsize) - 0 = unbuffered, 1 = line buffered, x = any size
# https://stackoverflow.com/questions/3167494/how-often-does-python-flush-to-a-file#3168436

logFile = open('log.txt', 'w+', 1) # also empties the file
log('Log file set up at', datetime.datetime.now())

# Auto-Configuration
# ------------------

configLines = None 	# emptied one element at a time

if len(sys.argv) > 1: 	# first element is the script name
	filename = 'config/' + sys.argv[1] + '.cfg'
	log("Loading configuration file:", filename)
	configLines = loadFileLines(filename)

def inputC(prompt):
	# Configurable version of input()

	if configLines == None:
		return input(prompt)
	else:
		if len(configLines) > 0:
			line = configLines.pop(0)
			answer = line.split('#')[0].rstrip('\t') 	# removes comments and tabs
		else:
			answer = ''

		log(prompt + answer)
		return answer

# Classes
# -------

class ImageCapture:
	def __init__(self, image, position, heading, altitude, pixelSize, searchAreaIndex):
		self.image = image
		self.position = position
		self.heading = heading
		self.altitude = altitude
		self.pixelSize = pixelSize
		self.searchAreaIndex = searchAreaIndex

		self.processed = 0

class Waypoint:
	# Defines a waypoint in Cartesian space that has a 2D position and can optionally specify an altutude and heading

	def __init__(self, position, heading=None, altitude=None):
		self.position = position
		self.heading = heading
		self.altitude = altitude

	def fromTuple(wpData, altitude):
		l = len(wpData)

		wp = Waypoint(wpData[0])
		if l > 1:
			wp.heading = wpData[1]
		if l > 2:
			wp.altitude = wpData[2]

		if altitude:
			wp.altitude = altitude

		return wp

	def __str__(self):
		out = "Waypoint at " + str(self.position) + "m"

		if self.heading:
			hdg = deg(self.heading.bearing())
			out += ", direction " + str(round(hdg,1)) + "deg"

		if self.altitude:
			out += ", altitude = " + str(self.altitude) + "m"

		return out

class FoundTarget:
	# Defines a found target by its world possition and charachter

	def __init__(self, position, character):
		self.position = position
		self.character = character

	def __str__(self):
		return "Target " + str(self.character) + " at " + str(self.position) + "m"

# 
# -----------------

#
# Remember in mission planning!!!
# - the flight direction doesn't have to equal the camera's 'up', the UAS can fly in any direction
# - using a 'choose closest each time' TSP solution should automatically create a diagnoal flight
#   patten if the camera's 'up' is kept in line with the shortest image axis
# 
# solution given Aruturo's info is to simply follow waypoints with as normal (with landscape camera 'up'
# [the shortest image axis] oriented parrelel with the UAS heading) and after reaching each one, set the heading
# to the closest one that is parrellel to the the relevant axis of the search area
#

# Initial commands
# ----------------

# clear archive directory - will be re-populated by recon.py on processing of each image
clearDir('out-archive')

# check platform name
log("sys.platform = " + sys.platform)

inputSearchAreas = 		0 # may be modified in createSearchAreas
simulateFCInterface = 	overrideBoolInput('simulate FC interface', 1)
simulateCamera = 		overrideBoolInput('simulate camera', 1)
enableDisplay = 		overrideBoolInput('enable drawing', 1) 			# headless mode
waitForArm = 			overrideBoolInput('wait for remote arm', 0) 	# for remote take-off trigger
enableSITL = 			overrideBoolInput('enable dronekit-sitl', 1)

# System Parameters
# -----------------
# 
# image will likely be landscape, so...
#
#  ^ aircraft 'forward' = primary axis
#  |                       = image up = decreasing j
#     i=0   i=W
# \ /  +-----+ j=0
#  X   :     :      --> aircraft 'right' = secondary axis 
# / \  +-----+ j=H         = image right = increasing i
#
cruiseSpeed = 16
captureAltOptimum = 80 	# roughly 70-80
captureAlt = float(overrideInput('capture altitude', captureAltOptimum))
fovS, fovP = 62.2, 48.8 	# https://www.raspberrypi.org/documentation/hardware/camera/README.md
resH, resV = 3280, 2464

gpioPinLED = 18

# Define coordinates
log(header("Coordinate Systems"))
log("North = N =  0 degrees = +ve Y cartesian")
log("East  = E = 90 degrees = +ve X cartesian")

# Initialise Interfaces
# ---------------------
log(header("Initialising Interfaces"))

# Initialise FC interface
if not simulateFCInterface:
	log('Initisalising FCI...')
	fcInterface = FCInterface('fcinterface/')
	fcInterface.enableLogging = overrideBoolInput('enable FCI logging', 0)

	# connect to flight control system
	if enableSITL: fcInterface.initSITL()
	fcInterface.connection()

	# try connection again if didnt work
	connected = len(fcInterface.getPosition()) == 2 	# should be a tuple with two elements
	if not connected:
		fcInterface.connection()  	# connection to PX4 sometimes only works the second time?

	def fcGetGPSPos():
		lat, lon = fcInterface.getPosition()
		return GPSPosition(lat, lon)

	def fcGetAlt():
		# returns altitude relative to the ground/home position
		return fcInterface.getAltitude()

	def fcSetWaypoint(lat, lon, altitude=None):
		if altitude:
			fcInterface.setWaypoint(lat, lon, altitude)
		else:
			fcInterface.setWaypoint(lat, lon)

	# run
	doFCTest = overrideBoolInput('run flight control test', 0)
	if doFCTest:
		log('Running test... (will disarm)')
		out = fcInterface.runTest()
		log("Test output:", out)
		time.sleep(5)
		log('Exiting...')
		exit()

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(gpioPinLED, GPIO.OUT)

# Setup GPS Locale
# ----------------
log(header("GPS Locale"))
gpsOriginSource = inputC("Enter GPS origin source ['man', 'drone']: ")
if gpsOriginSource == 'man':
	gpsOrigin = inputGPSPos()
elif gpsOriginSource == 'drone':
	gpsOrigin = fcGetGPSPos()
else:
	gpsOrigin = GPSPosition(51.242346, -0.590729)

gpsLocale = GPSLocale(gpsOrigin)
log("GPS locale set up at", gpsOrigin)

# Mission Parameters
# ------------------
log(header("Mission Setup"))

# Set take-off and landing point
if gpsOriginSource == 'drone':
	tolWP = Waypoint(Vector2(0,0), None, 0) # at origin 
else:
	tolWP = Waypoint(Vector2(20,10), None, 0)

log("TO/L waypoint:", tolWP)
searchAreaCount = int(overrideInput('serach area count', 2))
searchAreas = createSearchAreas(searchAreaCount)
targetsPerSearchArea = 2
altitudeLimits = [20 * 0.3048, 400 * 0.3048] # convert from feet to metres

# Describe key SA charachteristics
log("Key search area charachteristics:")
for sa in searchAreas:
	sDist = round(sa.center.magnitude(),2)
	sArea = round(sa.area(),2)
	log(" Search area of", sArea, "m2", sDist, "m from the origin")

targetSize = 2 # m, size=width=height
overlapFactor = float(inputC("Enter overlap factor: ") or 1) # defaults to 1...
# - where a single target rotated to (worst case)
# - 45 degrees and placed at the boundary between captures will appear only just fully in both images 
overlapRequired = targetSize * overlapFactor * 2**0.5

log("Overlap required:", round(overlapRequired, 2), "m")

# Plan Flight
# -----------
log(header("Flight Planning"))
# image height 'up' is considered in the aircraft 'forward' direction, but may be more than width
captureFieldRef = frustrumFieldRect(captureAlt, rad(fovP), rad(fovS)) # unrotated, unpositioned - relative to the aircraft in both position and orientation
w, h = captureFieldRef.width(), captureFieldRef.height()

log("Calculated capture field @", captureAlt, "m =", captureFieldRef.size(), "m")

# tracking variables
currentSearchArea = 0 # 0,1
nwpCurrentSearchArea = 0 # set equal to currentSearchArea only when setWaypoint is called - used by camera simulation etc.
searchAreaFoundTargets = [0] * searchAreaCount 	# count
foundTargets = []
numImagesProcessed = 0
flightComplete = 0
numImagesPlanned = 0
rtbInitiated = 0
shutdownFlag = 0

# adjust image sizes for flight planning (approximating overlap)
pw = w - overlapRequired
ph = h - overlapRequired
#log("Adjusted image field sizes for flight planning:", (pw, ph))
# [TODO] may lead to duplicates when targets are at edge of images
#			adjust recon.py to reject targets that are don't fit the target form
# 		and command.py to identify and mark duplicates

# subdivide search area into capture rects
if 1:  # [TODO] just for debugging - shows capture rects
	# captureRects are of the smaller size, not including overlap so dont show it's effect when drawn
	captureRects = []
	for sa in searchAreas:
		rects = Rect2.packUniform(sa, ph, pw, 0)
		captureRects.extend(rects)
		
	# test actual overlap
	# (compares distance between capture points to *real* image field size)
	log("Actual calculated overlap:\n correct number below should be just greater than root2 * target width = 2.83 for competition")
	a = captureRects[0]
	b = captureRects[1]
	centreDist = (a.center - b.center).magnitude()
	log(" Overlap by width:",  round(w - centreDist, 2), "m")
	log(" Overlap by height:", round(h - centreDist, 2), "m")

# define search area waypoints
saWaypoints = [[]] * searchAreaCount
saRange = range(0,len(searchAreas))
for s in saRange:
	sa = searchAreas[s]

	# generate capture points
	capturePoints = Rect2.packUniform(sa, ph, pw, 2)

	# create waypoints
	wpAlt = captureAlt
	saWaypoints[s] = [Waypoint.fromTuple(_, wpAlt) for _ in capturePoints]

	# increment counter
	numImagesPlanned += len(capturePoints)

# For camera simulation only, pre-define the indices of the image(s) which will contain targets
if simulateCamera:
	grassImage = "test-1cm-grass-fs.png"
	targetImages = ["test-1cm-text5-fs.png","test-1cm-text4-fs.png"]
	simTargetsPerImage = 2
	simImageHits = round(targetsPerSearchArea / simTargetsPerImage)
	simTargetIndices = [[]] * searchAreaCount  # a list of indices for each search area
	simImageNames= [[]] * searchAreaCount # a list of file names for each search area 
	for s in range(0, searchAreaCount):
		simSANumImagesPlanned = len(saWaypoints[s])
		simTargetIndices[s] = random.sample(range(0, simSANumImagesPlanned), simImageHits)
		
		# replace some indices with 'hit' images
		simImageNames[s] = [grassImage] * simSANumImagesPlanned
		for i in simTargetIndices[s]:
			simImageNames[s][i] = random.choice(targetImages)

	log("Simulated image hits will occur at indices: ", simTargetIndices)
	# images will be 'popped' from the *beginning* of this list in the camera simulation logic

log("Flight planning complete:", numImagesPlanned, "captures planned (max.)")

# Mission Tracking
# ----------------

# interface tracking variables
nextWaypoint = tolWP
lastWaypoint = tolWP
lastWaypointSetTime = 0

def getNextWaypoint():
	global currentSearchArea, searchAreaFoundTargets, flightComplete, rtbInitiated

	saIsLast = currentSearchArea >= searchAreaCount-1
	saSweepComplete = len(saWaypoints[currentSearchArea]) <= 0
	saTargetsFound = searchAreaFoundTargets[currentSearchArea]
	saAllTargetsFound = saTargetsFound >= targetsPerSearchArea

	# ending case
	if nextWaypoint == tolWP and lastWaypoint != tolWP:
		flightComplete = 1
		strTime = round(getTime(),2)
		log("Flight complete at T+", strTime, "s")
		return None

	# move to next search area if ok
	if saSweepComplete or saTargetsFound:
		log("Leaving search area:", ("sweep complete" if saSweepComplete else "all targets found"))

		if saIsLast:
			rtbInitiated = 1
			log("Returning to base")
			startLandingSequence()
			return tolWP
		else:
			currentSearchArea += 1
			searchAreaFoundTargets[currentSearchArea] = 0
			log("Proceeding to search area #" + str(currentSearchArea + 1))

	# proceed with search area, regardless of if it has changed
	curWaypoints = saWaypoints[currentSearchArea]

	# find closest remaining waypoint
	iClosest = None
	dClosest = None
	for i, wp in enumerate(curWaypoints): 	# [TODO] not efficient
		# can use nextWaypoint for current location because getNextWaypoint should only be called on reaching it
		d = (wp.position - nextWaypoint.position).magnitude()
		if iClosest == None or d < dClosest:
			iClosest = i
			dClosest = d

	closestWP = curWaypoints.pop(iClosest)
	log("Waypoint popped:", closestWP)

	return closestWP

# Wait before starting flight
# (takeoff procedure will arm the vehicle if not already)
inputC("Press enter to start flight...")
if waitForArm and not simulateFCInterface:
	success = fcInterface.waitForArm()
	if success:
		log('Successfully armed')
	else:
		log('Arming timed out, shutting down...')
		exit()

# In-Flight
# ---------

log(header("Flight Log"))
log("Initial waypoint:", nextWaypoint)

# Mission completion check
# checks when reconnaissance is complete
# used because theoretically this could be found in two ways
# 1. imageProcessed 	- all images processed (only significant if both search areas are complete)
# 2. getNextWaypoint 	- returning to base (very unlikely/impossible because last image will not have, so call is not included)
def checkMissionComplete():
	global rtbInitiated, numImagesProcessed, capturedImages, shutdownFlag

	allImagesCaptured = rtbInitiated

	numImagesCaptured = len(capturedImages)
	allCapturedImagesProcessed = numImagesCaptured == numImagesProcessed

	# log("Checking if mission complete:", "AIC =", allImagesCaptured, "and ACIP =", allCapturedImagesProcessed)

	if allImagesCaptured and allCapturedImagesProcessed:
		sTime = round(getTime(),2)
		log("Mission complete at T+", sTime, "s - all", numImagesCaptured, "images captured and processed")

		# Set LED on until it will turn off once the Pi shuts down (on next display/sim loop)
		if isLinux:
			GPIO.output(gpioPinLED, GPIO.HIGH)

		# [TODO] Summarise info
		missionReport()

		log("")
		inputC("Press enter to exit:") 	# inputC will return enter to skip this in cfg missions
		log("Exiting...")
		
		time.sleep(5)
		shutdownFlag = 1 # checkMissionComplete can be called by a thread callback, so exit needs to be called elsewhere

# Summarise, report and save mission data
def strTarget(target):
	gpsPos = gpsLocale.toGPS(target.position)
	return str(target) + " -> GPS" + str(gpsPos)

def missionReport():
	report = "Targets Found\n"
	for target in foundTargets:
		report += strTarget(target) + "\n"

	log(report)

# time
bootTime = time.clock()
def getTime():
	# Returns time in seconds since boot
	return time.clock() - bootTime

# Image Handling
# --------------

ipWorkerCount = 2 	# 2 seems to be a good number on RasPi with 18/05 software
log("Image processing workers:", ipWorkerCount)

capturedImages = []

# Setup workers
executor = concurrent.futures.ThreadPoolExecutor(max_workers=ipWorkerCount)

def imageProcessed(future):
	global capturedImages, searchAreaFoundTargets, numImagesProcessed

	ciIndex, targetsDescriptor = future.result()
	ci = capturedImages[ciIndex]

	# Save target info
	ci.processed = 1
	ci.targetsDescriptor = targetsDescriptor

	numImagesProcessed += 1
	numImagesCaptured = len(capturedImages)
	log("Image", ciIndex, "processed", "(", numImagesProcessed, "/", numImagesCaptured, ")")

	# free up memory (image saved in recon.py)
	ci.image = None

	# if targets found in image
	numTargetsFound = len(targetsDescriptor)
	if numTargetsFound > 0:
		log(" Target(s) found in image:", len(targetsDescriptor))

		# image constants
		hdg = ci.heading.normalized()
		hdgP = hdg.perpendicular()

		# store target info
		for td in targetsDescriptor:
			relPos = td[2] 	# this will be wrong when using test images
							# since pixel size is set arbritrarily, rather than based on altitude and fov

			relPosWorld = relPos[1] * hdg + relPos[0] * hdgP
			worldPos = ci.position - relPosWorld

			ft = FoundTarget(worldPos, td[0])
			foundTargets.append(ft)

			log(" ", ft)

		# important to use search area index at time of capture 
		saIndex = ci.searchAreaIndex
		searchAreaFoundTargets[saIndex] += numTargetsFound

		# [TODO] currently, this just passively sets the 'all targets found in SA' flag without triggering anything
		# the UAS will only move on from the SA when it reaches the next waypoint and has to set a next waypoint
		# could instead check here also if the cap has been reached and trigger a setWaypoint somehow, to reduce
		# flight time and have one less image to process
		
	# Check completeion
	checkMissionComplete()
	
def imageCaptured(imageCap):
	ciIndex = len(capturedImages)
	ci = imageCap

	log("Image", ciIndex, "captured, submitting to processing pool...")

	meta =  		"ID" + str(ciIndex)
	meta += "\n" + 	"Search Area #" 										+ str(ci.searchAreaIndex)
	meta += "\n" + 	"UAS Position (Cartesian, Relative to Origin) = " 		+ str(ci.position) + "m"
	meta += "\n" + 	"UAS Orientation (Forward Vector) = " 					+ str(ci.heading) + "m"
	meta += "\n" + 	"UAS Orientation (Bearing) = " 							+ str(round(deg(ci.heading.bearing()), 2)) + " deg"
	meta += "\n" + 	"UAS Altitude (Relative to Origin) = " 					+ str(round(ci.altitude, 2)) + "m"
	meta += "\n" + 	"Equiv. Pixel Size (At Altitude) = " 					+ str(round(ci.pixelSize, 2)) + "m"

	capturedImages.append(imageCap)

	future = executor.submit(processImage, imageCap.image, imageCap.pixelSize, ciIndex, meta)
	future.add_done_callback(imageProcessed)

# Interfaces
# ----------

# Initialise camera
if isLinux and not simulateCamera:
	camera = PiCamera()
	camera.resolution = resH, resV

# external interfaces:
# flight control (pixhawk)
# - set waypoints
# - get GPS location/altitude
# - receive notification on flight event (e.g. cruise reached, waypoint reached, stable hover achieved)
# camera
# - capture commands (including manual exposure settings)
# - RasPi camera library
# - other camera protocols e.g. MIPI CSI-2

# External calls
def camCapture():
	time.sleep(2) # must wait at least 2 seconds to allow camera to adjust to lighting

	stream = io.BytesIO()
	camera.capture(stream, format='jpeg')

	# "rewind" the stream to the beginning so we can read its content
	stream.seek(0)

	return Image.open(stream)

# Intermediary functions (including simulation)
def updateLocation():
	if simulateFCInterface:
		# hold position 
		if nextWaypoint == None:
			return lastWaypoint.position

		# linearly interpolate between waypoints at cruise speed
		a = lastWaypoint.position
		b = nextWaypoint.position
		s = cruiseSpeed
		t = getTime() - lastWaypointSetTime

		ab = b - a
		dist = ab.magnitude()  	# length
		tf = dist / s 			# t = d/s
		if tf == 0:
			tt = 1
		else:
			tt = clamp(t / tf, 0, 1)

		r = ab * tt
		v = r + a

		if tt > 0.999: # reached waypoint
			onStableHoverAchieved()

		return v
	else:
		gpsPos = fcGetGPSPos()
		#log(gpsPos)
		return gpsLocale.toVector(gpsPos)

def updateRotation():
	# returns the forward vector of the aircraft

	if simulateFCInterface:
		if nextWaypoint == None:
			return Vector2(0,1)

		# use the normalised vector between waypoints, assuming the path is direct and neglecting time to rotate
		a = lastWaypoint.position
		b = nextWaypoint.position

		d = (b - a).normalized()
		return d

	else:
		bearing = fcInterface.getHeading()  # returns heading in degress from North
		return Vector2.angleVector(bearing)

def getAltitude():
	if simulateFCInterface:
		return 0
	else:
		return fcGetAlt()

def setWaypoint(w):
	global nwpCurrentSearchArea, currentSearchArea, nextWaypoint, lastWaypoint, lastWaypointSetTime

	lastWaypoint = nextWaypoint
	nextWaypoint = w
	lastWaypointSetTime = getTime()
	nwpCurrentSearchArea = currentSearchArea
	
	if simulateFCInterface:
		pass

	else:
		gpsPos = gpsLocale.toGPS(w.position)
		heading = w.heading
		altitude = w.altitude 

		log("Setting FCI waypoint: GPS ", gpsPos)
		fcSetWaypoint(gpsPos.lat, gpsPos.lon, altitude)

def onStableHoverAchieved():
	global nextWaypoint

	log("Reached waypoint")

	# rotate to waypoint heading before continuing
	if not simulateFCInterface and nextWaypoint.heading:
		fcInterface.setHeading(deg(nextWaypoint.heading.bearing())) 	# set exact heading (not a parralel one) because the camera needs to point the correct direction [TODO] or maybe not - do the positions get translated based on captured image rect or planned rect?
		time.sleep(15) 	# wait for rotation to complete (assuming 20 deg/s, 180 deg rotation -> 9s) 	[TODO] should really be callback based

	# get next waypoint
	nwp = getNextWaypoint()

	if nwp:
		# capture image
		curAlt = nextWaypoint.altitude
		if getTime() > 1 and curAlt == captureAlt: # dont capture at initial waypoint
			capPos = nextWaypoint.position
			capRot = Vector2.snap(updateRotation(), nextWaypoint.heading)
			ic = captureImage(capPos, capRot)
			imageCaptured(ic)

	# set next waypoint
	setWaypoint(nwp)

def startTakeoffSequence():
	if simulateFCInterface:
		pass
	else:
		log("Starting FCI takeoff sequence...")
		fcInterface.startTakeoffSequence()

def startLandingSequence():
	if simulateFCInterface:
		pass
	else:
		log("Starting FCI landing sequence...")  # [TODO] CHECK THIS IS PRINTED IN THE SIMULATION BEFORE FLIGHT TESTING - IT SETS RTB
		fcInterface.startLandingSequence()

def captureImage(position, heading):
	global nwpCurrentSearchArea	
	
	# Returns an ImageCapture from the camera (or a simulation)
	# Position is passed in to avoid a circular call on updateLocation

	# https://www.raspberrypi.org/documentation/hardware/camera/README.md
	# Camera Module V2 Specs
	# Resolution: 		3280 x 2464 pixels
	# Focal length: 	3.04 mm
	# Horizontal FoV: 	62.2 degrees
	# Vertical FoV: 	48.8 degrees
	# Aperture: 		f/2.0
	
	if simulateCamera:
		#log("  nwpCurrentSearchArea =", nwpCurrentSearchArea)
		file = simImageNames[nwpCurrentSearchArea].pop(0) 	# 'pop' first item in SA-specific list
		log("Image simulated:", file)

		# Open image
		img = Image.open("test-images/" + file)

		# Allocate metadata
		pixelSize = 0.01
		altitude = captureAlt
		searchAreaIndex = currentSearchArea

	else:
		return camCapture()

	return ImageCapture(img, position, heading, altitude, pixelSize, searchAreaIndex)

# Start FCI cycle
if not simulateFCInterface:
	fcInterface.setNotificationCallback('waypointReached', onStableHoverAchieved)

	startTakeoffSequence()
	
	print("Takeoff sleep started T+", getTime())
	time.sleep(10) # wait for takeoff to complete
	print("Takeoff sleep finished T+", getTime())

	log("Setting initial waypoint...")
	setWaypoint(nextWaypoint)

# Initialize Display
# ------------------

dt = 1/30 if enableDisplay else 1/5

if enableDisplay:

	# pygame display
	pygame.init()

	size = width, height = 600, 600
	centre = Vector2(width, height) / 2
	scale = 0.5

	dirLength = 5

	red = (255,0,0)
	green = (0,255,0)
	blue = (0,0,255)
	darkBlue = (0,0,128)
	white = (255,255,255)
	black = (0,0,0)
	pink = (255,200,200)
	orange = (255,128,0)

	screen = pygame.display.set_mode(size)

	# Functions
	def pygDrawRect(rect, color=(255,255,255)):
		corners = []
		for i in range(1,5): # 1-4
			corners.append(rect.corner(i).toPixel())

		pygame.draw.lines(screen, color, 1, corners) 	# rectangle

	def pygDrawOrigin(pos, circleSize, crossSize):
		pygame.draw.circle(screen, black, pos, circleSize, 1)
		pygame.draw.line  (screen, black, (pos[0],pos[1]-crossSize), (pos[0],pos[1]+crossSize))
		pygame.draw.line  (screen, black, (pos[0]-crossSize,pos[1]), (pos[0]+crossSize,pos[1]))

	font = pygame.font.SysFont('Helvetica', 13)
	def pygDrawText(text, color, pos):
		textsurface = font.render(text, False, color)
		screen.blit(textsurface, pos)

# Sim/Display Loop
# ----------------
while 1:

	# respond to shutdown flag
	if shutdownFlag:		
		exit()
		break
	
	# blink light
	blink = int(getTime()) % 2 == 0 # time is even
	GPIO.output(gpioPinLED, GPIO.HIGH if blink else GPIO.LOW)

	# simulated waypoint tracking relies on regual polling of variables
	# so these must be called regardless of enableDisplay
	loc = updateLocation()
	forward = updateRotation() 	# must match positioning of camera on aircraft
	alt = getAltitude()

	if not simulateFCInterface:
		# handle notifications raised
		fcInterface.handleNotifications()

	# Display
	if enableDisplay:

		# handle events
		for event in pygame.event.get():
			if event.type == pygame.QUIT: exit() # handle window close
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESC:
					exit()
			elif event.type == pygame.MOUSEBUTTONDOWN:
				if event.button == 4: # scroll up
					scale *= 1.1
				elif event.button == 5: # scroll down
					scale *= 0.9

		# define world space features
		captureField = captureFieldRef.alignedWith(forward) + loc

		# translate vectors to canvas space
		cn = centre
		lc = loc * scale + centre
		nwd = None
		lwd = None
		if nextWaypoint:
			nw = nextWaypoint.position * scale + centre
			if nextWaypoint.heading:
				nwd = nw + nextWaypoint.heading.normalized() * dirLength
		if lastWaypoint:
			lw = lastWaypoint.position * scale + centre
			if lastWaypoint.heading:
				lwd = lw + lastWaypoint.heading.normalized() * dirLength

		# translate rect to canvas space
		cr = captureField * scale + centre 

		# clear canvas
		screen.fill(white)

		# draw axes
		axesArrowSize = 2;
		axesOffset = 5;
		axesLength = 30;
		
		pygame.draw.line(screen, black, (axesOffset, axesOffset), (axesOffset+axesLength, 	axesOffset)) 	#+x
		pygame.draw.line(screen, black, (axesOffset+axesLength-axesArrowSize, axesOffset-axesArrowSize), (axesOffset+axesLength-axesArrowSize, axesOffset+axesArrowSize))
		pygDrawText("x", black, (axesOffset+axesLength, axesOffset))
		
		pygame.draw.line(screen, black, (axesOffset, axesOffset), (axesOffset, axesOffset+axesLength)) 		#+y
		pygame.draw.line(screen, black, (axesOffset-axesArrowSize, axesOffset+axesLength-axesArrowSize), (axesOffset+axesArrowSize, axesOffset+axesLength-axesArrowSize))
		pygDrawText("y", black, (axesOffset, axesOffset+axesLength))

		# draw debug text
		pygDrawText("last: " + str(lastWaypoint), black, (20,20))
		pygDrawText("next: " + str(nextWaypoint), black, (20,40))
		pygDrawText("time: " + str(round(getTime(), 1)) + "s", black, (20,60))
		pygDrawText("alt: " + str(round(alt, 1)) + "m", black, (20,80))

		# draw captured images
		for ci in capturedImages:
			pos = ci.position * scale + centre
			pos2 = pos + ci.heading * 8
			color = green if ci.processed else orange
			pygame.draw.line(screen, color, pos.toPixel(), pos2.toPixel())

		# draw found targets
		for ft in foundTargets:
			tp = ft.position * scale + centre
			pygame.draw.circle(screen, red, tp.toPixel(), 3, 1)

			# draw character
			c = ft.character
			t = c[0] if isinstance(c, list) else c
			pygDrawText(t, red, tp.toPixel())

		# draw search areas
		for sa in searchAreas:
			r = sa * scale + centre
			pygDrawRect(r, darkBlue)

		# draw capture rects
		for rect in captureRects:
			r = rect * scale + centre
			pygDrawRect(r, pink)

		# draw capture field
		pygDrawRect(cr, red)

		# mark origin
		pygDrawOrigin(cn.toPixel(), 5, 6)

		# draw points
		pygame.draw.circle(screen, red, 	nw.toPixel(), 	5, 1)	# mark next waypoint
		if nwd: 													# and heading
			pygame.draw.line(screen, red, 	nw.toPixel(),	nwd.toPixel())

		pygame.draw.circle(screen, blue, 	lw.toPixel(), 	5, 1)	# mark last waypoint
		if lwd: 													# and heading
			pygame.draw.line(screen, blue, 	lw.toPixel(),	lwd.toPixel())

		pygame.draw.circle(screen, green, 	lc.toPixel(), 	3, 1)	# mark current position
	
		# cycle frame
		pygame.display.update()

	time.sleep(dt)