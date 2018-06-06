# -*- coding: utf-8 -*-
"""
Created on Fri May 18 16:22:45 2018
PYTHON 2
list of functions interacting with dronekit, accepting strings as input to execute functions
@author: OliG
"""

from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math

def connection():
    # literally no clue what these 5 lines do but it might be needed but actually probably not
    # Parse connection argument
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--connect", help="connection string")
    args = parser.parse_args()
    if args.connect:
        connection_string = args.connect
    
	print("FCI-F started with Py", sys.version)    
    
    # Connect to the Vehicle
    print("Connecting")
    # referenced elsewhere in the file
    global vehicle
    
    # once heartbeat has been adjusted, timeout needs to be set to 5
    
    # location of USB connection to pixhawk
    connection_string       = '/dev/ttyACM0'
    vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout = 30)
    
    
    print('connected to vehicle')
    
    
    home_position_set = False

    # Create a message listener for home position fix
    @vehicle.on_message('HOME_POSITION')
    def listener(self, name, home_position):
        global home_position_set
        home_position_set = True
    
    # wait for GPS lock
    while not home_position_set:
        print("Waiting for home position...")
        time.sleep(1)

    # Sanity Checks
    print(" Type: %s" % vehicle._vehicle_type)
    print(" Armed: %s" % vehicle.armed)
    print(" System status: %s" % vehicle.system_status.state)
    print(" GPS: %s" % vehicle.gps_0)
    print(" Alt: %s" % vehicle.location.global_relative_frame.alt)
    
    # this is right? required for simple_goto
    vehicle.mode = VehicleMode("GUIDED")
    
    # completes command
    print('DONE')
    


def getHeading():
    print(repr(vehicle.heading))
    print('DONE')

def getPosition():
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    print(lat, lon)
    print('DONE')

def getAltitude():
    # returns altitude above mean sea level
    alt = vehicle.location.global_frame.alt
    print(alt)
    print('DONE')

def setWaypoint(position):
    lat, lon, alt = position
    # convert to integers as input is string
    lat = int(lat)
    lon = int(lon)
    alt = int(alt)
    # converts to co-ord system relative to home point
    point = LocationGlobalRelative(lat,lon,alt)
    vehicle.simple_goto(point)
    print('moving to', lat, '', lon, '', alt)
    print('DONE')

def setHeading(heading_relative):
    # retrieves values froms args list
    relative = heading_relative[1]
    
    heading = heading[0]
    
    # unsure about the behavour of this http://python.dronekit.io/guide/copter/guided_mode.html#guided-mode-copter-set-yaw
    # maybe need to sort out a way of setting back to relative?
    
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

def startTakeoffSequence():
    # arming vehicle and making sure it is armed, if it fails then nothing else will work. 
    for i in range(0,100):
        vehicle.armed = True
        print("Armed: %s" % vehicle.armed)
        if vehicle.armed == True:
            break
    
    print('TAKING OFF')
    vehicle.simple_takeoff(10)
    print('DONE')
	# Set the controller to take-off and reach a safe altitude (e.g. 20ft)

def startLandingSequence():
    # should probably check the parameters for this..
    vehicle.mode = VehicleMode("RTL")
    print('Return to Land executed')
    print('DONE')

# =============================================================================
# not currently working !!!!
#   
#
# # Communication functions
# def telemetryTransmit(string):
#     vehicle.send_mavlink(string)
#     print(string + 'sent')
#     print('DONE')
# 	# Transmit the given string via the controller's telemetry so that it is shown at the ground station
# 
# =============================================================================

# Notification functions
def onActionCompleted(fn):
    True
	# Non-blocking callback function
	# Called once by command.py during aircraft boot
    
    
    

while 1:
    # looks for command and optional arguments 
    args = raw_input("FCI-F >").split()
    cmd = args.pop(0)
    
    if cmd == "connection":
        connection()
    if cmd == "getHeading":
        getHeading()
    if cmd == "getPosition":
        getPosition()
    if cmd == "getPosition":
        getPosition()
    if cmd == "getAltitude":
        getAltitude
    if cmd == "setWaypoint":
        setWaypoint(*args)
    if cmd == "setHeading":
        setHeading(*args)
    if cmd == "startTakeoffSequence":
        startTakeoffSequence()
    if cmd == "startLandingSequence":
        startLandingSequence()

#    if cmd == "telemetryTransmit":
#        telemetryTransmit(*args)

    if cmd == "onActionCompleted":
        onActionCompleted()
    
    
    
    elif cmd == "exit":
        sys.exit()
        
    # needed??
    time.sleep(0.1)
