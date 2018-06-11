import time
from picamera import PiCamera

fovS, fovP = 62.2, 48.8 	# https://www.raspberrypi.org/documentation/hardware/camera/README.md
resH, resV = 3280, 2464

camera = PiCamera()
camera.resolution = resH, resV

for i in range(0, 5):
	time.sleep(2)
	camera.capture('img-{}.jpg'.format(i), format='jpeg')