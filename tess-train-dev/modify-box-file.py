import math

charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789" # full alpha-numeric set
charCount = len(charset)
blockCount = charCount * 4

inFile = open("eng.box", "r", encoding="utf8") 
outFile = open("enr.box", "w") 

lineCount = blockCount

for i in range(lineCount):
	charI = math.floor(i / 4)
	char = charset[charI]
	print("Line", i, "-> character #", charI, "->", char)

	inLine = inFile.readline()
	outLine = char + str(i - charI * 4) + inLine[1:] 	# replace first character of line with correct one

	outFile.write(outLine)

inFile.close()
outFile.close()