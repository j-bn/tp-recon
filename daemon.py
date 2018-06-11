import os
os.system("start cmd /K python command.py sitl") #/K remains the window, /C executes and dies (popup)

# import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library

# stage = 0
# def buttonPressed(channel):
# 	print("Button pushed")
# 	stage += 1

# 	if stage == 1: 	# arm

# 	elif stage == 2: 	# shutdown
# 		exit()

# def exit():
# 	GPIO.cleanup() # Clean up
# 	# shut down command.py and the Pi?

# GPIO.setwarnings(False) # Ignore warning for now
# GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
# GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)

# GPIO.add_event_detect(10, GPIO.RISING, callback=buttonPressed) # Setup event on pin 10 rising edge

# # shutdown process
# message = input("Press enter to exit\n\n") # Run until someone presses enter
# exit()