#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 14 09:48:06 2018
Dronekit interface
Passes and recieves strings from dronekit_functions which interacts with dronekit
@author: OliG
"""

import subprocess
import sys

print("FCI-I started with Py", sys.version)

class FCInterface:

    def __init__(self):
        moduleFolder = 'fcinterfacedev/'
        isLinux = sys.platform.startswith('linux')
        pyCommand = 'python2' # if isLinux else 'python'

        # opens cli running python2 dronekit functions
        self.py2 = subprocess.Popen([pyCommand, '-u', moduleFolder + 'dronekit_functions.py'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, universal_newlines=True)

        print("FCInterface initialised")
        print("Initial output:", self.py2.stdout.readline())


    def interface(self, command):
        """
        pass a function name to dronekit_functions
        returns single line string resulting from function. When DONE is passed function is considered complete and 
        script moves on.
        """
        print('FCI-I writing: ' + command)
        self.py2.stdin.write(command + '\n')
        self.py2.stdin.flush()
        print('FCI-I reading...')
        line = 0
        for i in range(0,20):
            # keeps last line printed
            cmdreturn = line
            
            line = self.py2.stdout.readline()
            print(line)
            if line.startswith('DONE'):
                return(cmdreturn)
                break



    def connection(self):
        """
        connection to vehicle through interface and performs basic setup including setting home.
        won't complete until connection has complete
        """
        
        self.interface('connection')
        
        return True

    def getHeading(self):
        """
        no args, returns heading in degress from North
        """
        ans = self.interface('getHeading')
        return ans 

    def getPosition(self):
        """
        no args, returns lat and lon as two vars
        """
        ans = self.interface('getPosition')
        
        # converts single string to two ints
        position = ans.split()
        lat = int(position[0])
        lon = int(position[1])
        return lat, lon

    def getAltitude(self):
        """
        returns altitude above mean sea level in meters, no args
        """
        ans = self.interface('getAltitude')
        return ans

    def setWaypoint(self, lat, lon, alt):
        """
        uses simple_goto
        """
        self.interface('setWaypoint' + ' ' + str(lat) + ' ' + str(lon) + ' ' + str(alt))

    def setHeading(self, heading, *args):
        """
        Pass a heading in degrees from North. I think if you pass True as a second argument the copter will return to 
        using a relative heading (it may travel faster)
        """
        try: 
            relative = str(args[0])
        except:
            relative = ''
        self.interface('setHeading' + ' ' + str(heading) + ' ' + relative)

    def startTakeoffSequence(self):
        """
        arms the copter then takes off to 10 meters
        """
        self.interface('startTakeoffSequence')

    def startLandingSequence(self):
        """
        starts landing copter
        """
        self.interface('startLandingSequence')


    # =============================================================================
    # NOT CURRENTLY WORKING !!
    # # Communication functions
    # def telemetryTransmit(string):
    #     self.interface('telemetryTransmit' + ' ' + string)
    # 	# Transmit the given string via the controller's telemetry so that it is shown at the ground station
    # 
    # =============================================================================
    # Notification functions
        

    def onActionCompleted(self, fn):
        True
    	# Non-blocking callback function
    	# Called once by command.py during aircraft boot
    	# Sets up a notification so that, every time a commanded action is completed (e.g. waypoint/heading reached, take-off completed), function fn will be called-back

# Test
fcInterface = FCInterface()
fcInterface.connection()