# Script 1 (e.g. command.py)
# --------

interface = ... # link to script 2

def handleActionComplete(a,b,c...):
	# do stuff

interface.connect()
interface.setActionCompleteCallback(handleActionComplete)

while 1:
	# keep script running (may not be needed if script is doing stuff anyway)
	sleep(1/30) # some more appropriate time

# Script 2 (e.g. interface)
# --------

acCallbackFn = None

def connect()
	# usual stuff

def setActionCompleteCallback(fn):
	global acCallbackFn
	acCallbackFn = fn

def actionCompleted():
	if acCallbackFn:
		acCallbackFn() # callback to whatever function was set in setActionCompleteCallback()

# option 1, if DroneKit supports a listener for waypoint reached or 'idle' - really simple basically just link one callback to another
# not sure how it works, something like this
# see http://python.dronekit.io/automodule.html#dronekit.Vehicle
vehicle.parameters.add_listener('<waypoint reached>', actionCompleted)

while 1:
	# option 2, continuously poll
	waypointReached = <distance to waypoint> < ~0.5m
	if waypointReached:
		actionCompleted()

	# keep script running (may not be needed if script is doing stuff anyway)
	sleep(1/30) # some more appropriate time