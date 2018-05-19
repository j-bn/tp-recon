# see https://stackoverflow.com/questions/27863832/calling-python-2-script-from-python-3
#     https://stackoverflow.com/questions/9322796/keep-a-subprocess-alive-and-keep-giving-it-commands-python

#!/usr/bin/env python3
import subprocess

print("Started py3 script")

# -u for unbuffered output
cmd = "python2 -u test-py2.py"  # launch python2 script using bash

process = subprocess.Popen(cmd.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE)

def countWrapper():
	process.stdin.write(b'count\n')
	process.stdin.flush()
	print("reading...")
	output = process.stdout.readline() 	# each command in test-py2.py must be met with a single line response for this to work
	print("Py2 output: ", output)

while 1:
	input(">")
	countWrapper()