import sys
import time

n = 0

print("Started py2 script: " + sys.version)

def count():
	global n
	n += 1
	print("Counted to " + str(n))
	return n

# listen for input
while 1:
	cmd = raw_input("P2>")[:-1] # remove carraige return character

	if cmd == "count":
		count()
	elif cmd == "exit":
		sys.exit()
	else:
		print("Unkown cmd: '" + cmd + "'")