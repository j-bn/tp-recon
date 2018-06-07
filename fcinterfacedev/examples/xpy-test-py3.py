# see https://stackoverflow.com/questions/27863832/calling-python-2-script-from-python-3
#     https://stackoverflow.com/questions/9322796/keep-a-subprocess-alive-and-keep-giving-it-commands-python

#!/usr/bin/env python3
import subprocess
import sys

print("Started py3 script: " + sys.version)

# -u for unbuffered output
cmd = "python2 -u pyx-test-py2.py"  # launch python2 script using bash

# universal_newlines = 1 seems to negate using bytes to transmit data
process = subprocess.Popen(cmd.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE, universal_newlines = 1)

print("Py2 output: '", process.stdout.readline() + "'")

def countWrapper():
	process.stdin.write('count\n')
	process.stdin.flush()
	print("reading...")
	output = process.stdout.readline() 	# each command in test-py2.py must be met with a single line response for this to work
	print("Py2 output: ", output)

while 1:
	input("P3>")
	countWrapper()