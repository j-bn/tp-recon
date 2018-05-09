#!/usr/bin/env python
# -*- encoding: utf-8 -*-

from __future__ import print_function
import os, sys
import shutil
import numpy
import math
# import colorsys
from PIL import Image, ImageOps
from matplotlib import pyplot as plt, image as mplimg
import pytesseract

# tesseract installed from https://github.com/UB-Mannheim/tesseract/wiki (3.05...) on windows
if sys.platform.startswith('win'):
	pytesseract.pytesseract.tesseract_cmd = 'C:\\Program Files (x86)\\Tesseract-OCR\\tesseract.exe'
elif sys.platform.startswith('linux'):
	pytesseract.pytesseract.tesseract_cmd = '\\usr\\bin\\tesseract'
else:
	print('ERROR Unkown platform, cannot set tesseract-ocr location')

# Control Variables
# -----------------

enableLogging = 0

# Utility Functions
# -----------------

def log(*s):
	if enableLogging:
		print(s)

def logHeader(s):
	log("")
	log(s)

def clearDir(folder):
	# Deletes everything inside a given directory
	# 'folder' appears to be a relative path - tested
	# [https://stackoverflow.com/questions/185936/how-to-delete-the-contents-of-a-folder-in-python]

	#folder = '/path/to/folder'
	for the_file in os.listdir(folder):
	    file_path = os.path.join(folder, the_file)
	    try:
	        if os.path.isfile(file_path):
	            os.unlink(file_path)
	        elif os.path.isdir(file_path): shutil.rmtree(file_path)
	    except Exception as e:
	        print(e)

def boundingSquareSize(w, a):
	# in 2D space, returns the side length of an axis-aligned square that bounds a square of side length w rotated around its centre by an angle a in radians

	# see https://stackoverflow.com/questions/10392658/calculate-the-bounding-boxs-x-y-height-and-width-of-a-rotated-element-via-jav

	ab = math.fmod(a, math.pi / 2)
	wb = (math.sin(ab) + math.cos(ab)) * w

	log("bounding square: ",w,"->",wb,"(a=",math.degrees(a)," deg)")

	return wb

def boundingSquare(w, c):
	# in 2D space, returns a tuple containing the edge coordinates (left, top, right, bottom) of an axis-aligned bounding box for a square with side length w and centre c

	cx, cy = c
	e = w / 2 	# extents

	# left, top, right, bottom
	bb = (cx - e, cy - e, cx + e, cy + e)

	return bb

def padBoundingBox(bb, p):
	# in 2D space, crops (-) or pads (+) the bounding box bb (left, top, right, bottom) with the number of pixels p

	# [TODO] better way?
	return (bb[0] - p, bb[1] - p, bb[2] + p, bb[3] + p)

# Mission Paramaters
# ------------------

charset = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789'

# -----------------------------------------------------------------------------------------------------------------

# Processing Functions
# --------------------

def getPixelSize(img, alt):
	# Uses altitude, given in metres, to calculate pixel size

	# get resolution from image
	resH = img.size[0]
	resV = img.size[1]
	Np = resH * resV

	# FoV from camera spec.
	fovH = math.radians(62.2)
	fovV = math.radians(48.8)

	# Simplified formula to calculate pixel size from altitude
	# assuming camera is pointing directly downwards and pixels are square
	# (angle factor could potentially be altered to find values for any tilt amount)
	x = 2 * alt * (math.tan(fovH/2) * math.tan(fovV/2) / Np)**0.5

	return x

def processImage(img, pixelSize, imgID=None, saveInputImageWithMetadata=None):

	# ignore enableLogging
	print("Started processing image", imgID)

	# Archive image
	# implemented to free up memory to fix problems on the ~22nd image
	# recon.py saves the images as part of the processing thread (to imrpove concurrency)
	# command.py can then safely delete the large image data in memory, allowing it to be garbage collected
	if saveInputImageWithMetadata:
		filePath = 'out-archive/image-' + str(imgID)

		# save image to file
		img.save(filePath + '.bmp')

		# save 
		f = open(filePath + '-meta.txt','w')
		f.write(saveInputImageWithMetadata)
		f.close()

	resH = img.size[0]
	resV = img.size[1]
	fieldH = resH * pixelSize	# m
	fieldV = resV * pixelSize	# m

	fieldA = fieldH * fieldV

	# calculate sizes at ground level, in image space
	# (assuming pixels are square)
	pixelArea = pixelSize**2		# m^2
	pixelsPerMetre = 1 / pixelSize 	# m^-1
	pixelCount = resH * resV

	# target characteristics
	# UAS 2018 Brief 7.1: "The ground marker shown below in Figure A1 is a red 1 m x 1 m central square,
	# incorporating an alphanumeric code in white, approximately 0.75 m high, within the
	# square. To help identification of the ground marker, a 2 m x 2 m white square will
	# surround the central area" 
	targetOuterSize = 2 	# actual size across, m
	targetInnerSize = 1 	# actual size across, m
	targetCharSize = 0.75	# actual size across, m

	targetOuterArea = targetOuterSize**2	# m^2

	targetOuterImageSize = targetOuterSize * pixelsPerMetre
	targetInnerImageSize = targetInnerSize * pixelsPerMetre

	# OLD CODE FOR CAMERA MDOE CALCULATIONS
	# # use situational and system information to derive some assumptions
	# alt = 20 		# altitude, metres
	# focalL = 2		# focal length, mm
	# sensorH = 3.69	# sensor width (horizontal), mm
	# sensorV = 2.76	# sensor height (vertical), mm
	# resH = 3280		# senoor resolution (horizontal)
	# resV = 2464		# sensor resolution (vertical)
	# # from Sony IMX219 (RasPi Cam V2) datasheet (approx.)

	# sensorD = (sensorH**2 + sensorV**2)**(0.5)	# sensor diagonal, mm
	# fovD = 2 * math.atan(sensorD / (2*focalL));		# FoV diagonal, rad
	# fovV = 2 * math.atan(sensorV / (2*focalL));		# FoV vertical, rad
	# fovH = 2 * math.atan(sensorH / (2*focalL));		# FoV horizontal, rad

	# # assuming camera is pointing directly downwards
	# # (angle factor could potentially be altered to find values for any tilt amount)
	# fieldH = 2 * alt * math.tan(fovH / 2) # projected image size at ground level (horizontal), m
	# fieldV = 2 * alt * math.tan(fovV / 2) # projected image size at ground level (vertical), m
	# fieldA = fieldH * fieldV

	# # target characteristics
	# # UAS 2018 Brief 7.1: "The ground marker shown below in Figure A1 is a red 1 m x 1 m central square,
	# # incorporating an alphanumeric code in white, approximately 0.75 m high, within the
	# # square. To help identification of the ground marker, a 2 m x 2 m white square will
	# # surround the central area" 
	# targetOuterSize = 2 	# actual size across, m
	# targetInnerSize = 1 	# actual size across, m
	# targetCharSize = 0.75	# actual size across, m

	# targetOuterArea = targetOuterSize**2	# m^2

	# # calculate sizes at ground level, in image space
	# # (assuming pixels are square)
	# pixelSize = fieldH / resH		# actual size across, m
	# pixelArea = pixelSize**2		# m^2
	# pixelsPerMetre = 1 / pixelSize 	# m^-1
	# pixelCount = resH * resV

	# targetOuterImageSize = targetOuterSize * pixelsPerMetre
	# targetInnerImageSize = targetInnerSize * pixelsPerMetre
	# END OLD CODE



	# Image Processing
	# ----------------
	logHeader("Pre-processing image")

	cols = img.size[0]
	rows = img.size[1]

	imgRGB = img.convert('RGB')
	imgData = numpy.array(imgRGB).transpose((1, 0, 2)) # convery to NumPy array and transpose back to x,y,c indexing from y,x,c default
	# [https://stackoverflow.com/questions/25537137/32-bit-rgba-numpy-array-from-pil-image]

	log(" Image data size: ", imgData.shape)
	log(" Top left pixel: ", imgData[0,0], len(imgData[0,0]))

	# 'snap' pixel values to discrete marker colours (green, red and white)
	log(" Applying pixel 'snap'...")

	# OLD SLOW METHOD
	# for i in range(cols): # for every col
	# 	#log("col: ", i)

	# 	for j in range(rows): # for every row
	# 		r,g,b = imgData[i,j]

	# 		# [TODO] could use hsl colour or not have an 'else' colour and just leave it blank to aid tuning
	# 		# [TODO] used to include functionality to save 'snapped' colour values back to the image, now removed and just edits 'map'
	# 		if b > 220: 	# white
	# 			n = 3
	# 		elif r > 230: 	# red
	# 			n = 2
	# 		else: 			# green
	# 			n = 1

	# 		mapData[i,j] = n

	# 		# count pixels by colour
	# 		# [TODO] more efficient way of counting numpy arrays?
	# 		mapCount[n] += 1
	# END OLD CODE

	# Classify colours into a map
	mapData = numpy.where(imgData[:,:,2] > 220, 3, numpy.where(imgData[:,:,0] > 230, 2, 1))

	log(" Top-left pixel classification:", mapData[0,0])

	# compute pixel map counts
	unique, counts = numpy.unique(mapData, return_counts=True)
	mapCount = dict(zip(unique, counts))
	if 0 not in mapCount:
		mapCount[0] = 0
	log(" Pixel counts by colour (null, green, red, white):", mapCount)

	# add a legend to the output map
	mapDataDebug = mapData.copy().transpose((1, 0)) # transpose back to the y,x,c space, as expected by imsave
	for x in range(4):
		mapDataDebug[x,0] = x

	# output map classification image
	if debugMode:
		mplimg.imsave('out/map.png', mapDataDebug) # seems to not be thread-safe

	# from here on, best for efficiency to work using the numerical map rather than pixel map

	logHeader("Analysing image")

	# calculate relevant areas
	backgroundArea = mapCount[1] * pixelArea
	targetsArea = fieldA - backgroundArea
	log(" Field area:", fieldA, "m2")
	log(" Background area:", backgroundArea, "m2")
	log(" Target(s) area:", targetsArea, "m2")
	log(" Target outer area:", targetOuterArea, "m2")

	# calculate predicted number of targets
	predictedTargetCount = targetsArea / targetOuterArea
	log(" Predicted target count:", predictedTargetCount)

	# stores midpoint positions of each change between adjacent pixels
	transients = []

	log("Finding transient points...")
	# order of iteration and appending here is important as it dictates the order of transients and therefore the
	# enables the coincidence of the reference and 'first corner' points
	for i in range(cols-1): # for every col, excluding the last
		#log("col: ", i)

		for j in range(rows-1): # for every row, excluding the last	
			n = mapData[i,j]

			# adjacent (one pixel to the right/down) cell values
			r = mapData[i+1,j] # right
			d = mapData[i,j+1] # down

			# isolate cells which differ from the adjacent cell in colour
			# and note the midpoint of this discontinuity
			if n != r:
				mi = i + 0.5 	# midpoint in (sub) pixel space
				transients.append([mi,j,-1])
			if n != d:
				mj = j + 0.5 	# midpoint in (sub) pixel space
				transients.append([i,mj,-1])

	log(" Found transients:", len(transients))

	# group transients by distance
	logHeader("Grouping transients by position...")

	maxGroupSize = (2*targetOuterImageSize**2)**0.5 	# pixels, represents circle diameter defined by image-space size of a target's longest (diagonal) dimension
	groupCount = int(round(predictedTargetCount))
	groupRange = range(groupCount)	# use zero-indexed groups, using negative numbers (-1) to indicate un-grouped transients

	# numpy arrays are zero-indexed and defined as (cols, rows)
	groupsRefPoint = numpy.empty((groupCount, 2))

	log(groupCount,"groups")

	for g in groupRange:
		log(" Group", g)

		refSet = 0
		refPoint = 0

		for i in transients:
			if not refSet:		# do nothing until an un-grouped point is found
				if i[2] < 0: 	# use first un-grouped point as reference transient point
					i[2] = g 		# define group around this point
					refPoint = i 	# use point as reference
					refSet = 1		# notify future iterations

					# save reference point for each group, as it represents the first 'corner'
					groupsRefPoint[g,:] = i[:2] 	# omit group index element

					log(" Group reference point set at", i)
			else:				# act on all non-referance points once a reference point is defined
				# with respect to group reference point
				dx = refPoint[0] - i[0]
				dy = refPoint[1] - i[1]
				dist = (dx**2 + dy**2)**0.5

				if dist <= maxGroupSize:
					i[2] = g 	# if within maximum group size, group together


	if debugMode:
		log("Displaying transient plot...")
		npData = numpy.array(transients)
		x, y, g = npData.T

		# # swap group indices for colors
		# colors = ['k','r','g','b','c','m','y']
		# for gi in g:
		# 	gi = colors[gi]

		plt.scatter(x,y,c=g,s=5)
		plt.show()

	# find centre of each transient group
	groupsPosSum = numpy.zeros((groupCount, 2)) 	# 2 col, N rows
	groupsPosCount = numpy.zeros((groupCount, 1)) 	# column vector
	for t in transients:
		g = t[2]
		groupsPosCount[g,0] += 1
		groupsPosSum[g,0] += t[0]
		groupsPosSum[g,1] += t[1]

	groupsPosAvg = groupsPosSum / groupsPosCount 	# [TODO] inffeccient? swap for proper matrix operation?
	log("Group average positions:", groupsPosAvg, "px")

	# convert center positions to field-space
	# [TODO] check all american vs british spellings
	targetsPos = groupsPosAvg * pixelSize

	# centre positions relative to the camera
	fieldOffset = numpy.array([fieldH, fieldV]) / 2
	targetsPos -= fieldOffset
	log("Target center positions:", targetsPos, "m")

	# calculate target angle using group reference points
	logHeader("Calculating rotations...")
	groupsAngle = numpy.empty(groupCount)
	for g in groupRange:
		log(" Group", g)

		groupRefPoint = groupsRefPoint[g]
		groupPosAvg = groupsPosAvg[g]

		dx = groupRefPoint[0] - groupPosAvg[0]
		dy = groupRefPoint[1] - groupPosAvg[1] - 0.5 	# subtract 0.5 to move from transient position back to pixel position

		log("  Corner displacement",[dx,dy])

		# calculate target orientation from corner displacement
		a = math.atan(dy / dx) + (math.pi / 4) 	# radians
		r = math.degrees(a)						# degrees

		# save angle
		groupsAngle[g] = a

		log("  r = ", r, "degrees")

	# isolate textual area
	logHeader("Isolating targets for text recognition...")
	groupsPossibleChars = [[]] * groupCount 	# initialise an empty list for the possible chars of each group
	for g in groupRange:
		log(" Group", g)

		# target inner box, in pixel space
		a = groupsAngle[g] 			# angle
		c = groupsPosAvg[g] 		# centre
		w = targetInnerImageSize 	# size

		# [TODO] Move all this to a 'crop to rotated square method' function

		# get bounding box (A)
		bbSize = boundingSquareSize(w, a)
		bbA = boundingSquare(bbSize, c)

		# crop to bounding box (A)
		imgA = img.crop(bbA) 	# (left, top, right, bottom)
		if debugMode: imgA.save("out/tgt-" + str(g) + "-a.bmp")

		# rotate (B)
		aDeg = math.degrees(a)
		imgB = imgA.rotate(aDeg, resample=Image.BICUBIC, expand=True) 	# rotate counter-clockwise (=positive) see http://matthiaseisen.com/pp/patterns/p0201/
		if debugMode: imgB.save("out/tgt-" + str(g) + "-b.bmp")

		# crop to inner box and process (C)
		hw = imgB.size[0] / 2 	# image has been cropped, so redifine in terms of centre which has been preserved
		bbC = boundingSquare(w, (hw,hw))
		bbC = padBoundingBox(bbC, -5) # crop a little further
		imgC = imgB.crop(bbC) # crop to bounding bo	
		if debugMode: imgC.save("out/tgt-" + str(g) + "-c.bmp")

		# pre-process for rotations (D)
		imgDRef = imgC.convert('L') 									# convert to grayscale
		imgDRef = imgDRef.point(lambda p: p > 127 and 255) 				# binarise
		imgDRef = ImageOps.invert(imgDRef) 								# invert (black text)
		bgColor = imgDRef.getpixel((0,0))
		imgDRef = ImageOps.expand(imgDRef, border=20, fill=bgColor) 	# expand image

		# initialise an empty list for possible chars of this group (unkown length)
		possibleChars = []

		# process OCR at each of 4 rotations (D)
		for a in range(0, 360, 90): # [0, 90, 180, 270]
			# [TODO] could instead re-use the last image and rotate 90 further each time - would this be faster?
			if a != 0:
				imgD = imgDRef.rotate(-a) 		# rotate clockwise (=negative)
			else:
				imgD = imgDRef

			# pass to Tesseract OCR engine
			# config='-psm 10' -> treat as a single charachter
			# https://stackoverflow.com/questions/31643216/pytesseract-dont-work-with-one-digit-image
			# https://stackoverflow.com/questions/44619077/pytesseract-ocr-multiple-config-options
			# however, with testing 6 seems to work better
			ocrConfig = '-psm 6'

			ocrText = pytesseract.image_to_string(imgD, config=ocrConfig)
			log("a=",a,"-> text:",ocrText)

			# ocrData = pytesseract.image_to_data(imgD, lang='eng', config=ocrConfig, output_type=pytesseract.Output.DICT) 	 
			# ocrCount = len(ocrData['level'])
			# ocrConfidence = ocrData['conf']
			# log("a=",a,"-> text:",ocrText,"confidence:",ocrConfidence)

			if debugMode: imgD.save("out/tgt-" + str(g) + "-d-" + str(a) + ".bmp")

			# save charachters that fit mission profile
			c = ocrText
			if len(c) == 1 and c in charset:
				possibleChars.append(c)

		groupsPossibleChars[g] = possibleChars

	# export final image
	if debugMode:
		log("Saving image...")
		img.save('out/out.bmp')

	# final output
	targetsDescriptor = list(zip(groupsPossibleChars, groupsPosAvg, targetsPos))

	logHeader("Final results")
	log(len(targetsDescriptor), "targets found:")
	for td in targetsDescriptor:
		possibleChars, imgPos, relPos = td
		log(possibleChars, "at", imgPos, "px in the image or, in relative to the UAS:", relPos, "m")

	return imgID, targetsDescriptor

# ------------------------------------------------------------------------------------------------------------------------------

# Initisialisation
# ----------------

# Clear the out folder for each run
clearDir('out')

debugMode = 0

# test image mode	resolution, ground pixel size 	->	field size
# camera mode		camera res, optics, alt 		->	pixel size not defined, calculate via triganometry etc.
testImageMode = 0

if len(sys.argv) > 1:
	testImageMode = 1
	enableLogging = 1
	testImagePath = "test-images/" + sys.argv[1]

if testImageMode:
	log("Test Image Mode")

	# Set ground resolution
	if testImagePath.endswith("25m.jpg"):
		groundRes = 3.38 # cm
	else:
		groundRes = 1

	pixelSize = groundRes / 100	# m

	# Load image
	log(" Loading " + testImagePath)

	log("Opening image...")
	img = Image.open(testImagePath)

	log("",img.format, img.size, img.mode)

	processImage(img, pixelSize)