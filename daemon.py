import os, sys
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
import subprocess
import pytesseract

isLinux = sys.platform.startswith('linux')
cmd = "command.py sim &"
shutdownCommand = "/usr/bin/sudo /sbin/shutdown -r now"

stage = 0
def buttonPressed(channel):	
	global stage, cmd
            
	print("Button pushed")
	stage += 1
    
	if stage == 1: 	# arm
		print('Arming Pi...')
		if isLinux:
			cmd = "python3 " + cmd
			print("Linux running:", cmd)
			subprocess.call(cmd, shell=True)
		else:
			os.system("start cmd /K python " + cmd) #/K keeps the window, /C executes and dies (popup)
	elif stage == 2: # disarm and shutdown
		if isLinux:
			pass
			#subprocess.Popen(shutdownCommand.split(), stdout=subprocess.PIPE)
		exit()

def exit():
	GPIO.cleanup() # Clean up
	sys.exit()

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

GPIO.add_event_detect(10, GPIO.RISING, callback=buttonPressed) # Setup event on pin 10 rising edge
print("Listening for button press...")

# shutdown process
input("Press enter to exit...") # Run until someone presses enter
exit()