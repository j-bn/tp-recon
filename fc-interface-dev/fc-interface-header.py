# Python 3
# lists required functions for direct access from command.py (Py3)
# most (all 'get' and 'set') functions need to be non-blocking or return very quickly (~0.1s)
# if this is not possible, a callback function would be ok
# callbacks are also ideal for notifications e.g. 'onWaypointReached'

# all that needs to be done is to fill out the below definitions to function correctly
# stack: command.py <--> fc-interface.py (this) <--> py2 interface (sub-process) <--> dronekit <--> Pixhawk

# feel free to modify these as appropriate (e.g. it might be better to combine getPosition and getAltitude or setWaypoint and setHeading)

# wherever possible, try to adhere to this coordinate system:
# North = N =  0 degrees = +ve Y cartesian
# East  = E = 90 degrees = +ve X cartesian

# Navigation functions
def getPosition():
	# Non-blocking
	# Returns the current location of the aircraft as either of the following (whichever is easier)
	# - GPS position (numerical lattitude, longditude), each to ~6 decimal places
	# - Cartesian vector - tuple or list of x and y position in metres, where +ve y is North and +ve x is East
	#   if using this option please also make available the position of the origin for reference

def getAltitude():
	# Non-blocking
	# Returns the current altitude of the aircraft in metres, either above sea level or the ground (say which)

def getHeading():
	# Non-blocking
	# Returns the current heading of the aircraft as a bearing in degrees 'clockwise from North' (North = 0, East = 90)

def setWaypoint(position):
	# Non-blocking
	# Sets the controller to proceed to a particular waypoint
	# Arguments: an x, y or GPS position (whichever is easier)

def setHeading(heading):
	# Non-blocking
	# Sets the controller to rotate (yaw) so that it is 'facing' in the specified direction
	# Arguments: a bearing specifying the direction (similar to getHeading)

def startTakeoffSequence():
	# Set the controller to take-off and reach a safe altitude (e.g. 20ft)

def startLandingSequence():
	# Set the controller to cease all other operations and land (ideally at the take-off point)

# Communication functions
def telemetryTransmit(string):
	# Transmit the given string via the controller's telemetry so that it is shown at the ground station

# Notification functions
def onActionCompleted(fn):
	# Non-blocking callback function
	# Called once by command.py during aircraft boot
	# Sets up a notification so that, every time a commanded action is completed (e.g. waypoint/heading reached, take-off completed), function fn will be called-back