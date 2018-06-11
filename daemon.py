import os, sys
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import subprocess

cmd = "command.py sim"

stage = 0
def buttonPressed(channel):
	global stage	
            
	print("Button pushed")
	stage += 1
    
	if stage == 1: 	# arm
		print('Arming Pi...')
		if sys.platform.startswith('linux'):
			subprocess.call("python3 " + cmd, shell=True)
		else:
			os.system("start cmd /K python " + cmd) #/K keeps the window, /C executes and dies (popup)
	elif stage == 2: 	# shutdown
		exit()

def exit():
	GPIO.cleanup() # Clean up
	# shut down command.py and the Pi?

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

GPIO.add_event_detect(10, GPIO.RISING, callback=buttonPressed) # Setup event on pin 10 rising edge
print("Listening...")

# shutdown process
input("Press enter to exit\n") # Run until someone presses enter
exit()