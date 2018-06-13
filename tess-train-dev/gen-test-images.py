from PIL import Image, ImageOps, ImageFont, ImageDraw

charset = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789" # full alpha-numeric set

charCount = len(charset)
blockCount = charCount * 4 	# four orientations per character
print(charCount, "characters in total")
blockSize = 100 	# square

wb = hb = blockSize

font = ImageFont.truetype("arial.ttf", 96)

for i in range(charCount):
	char = charset[i]
	print("Character", i, ":", char)

	# draw to block image
	block = Image.new('RGB', (blockSize, blockSize), 'white')
	draw = ImageDraw.Draw(block)
	wt, ht = draw.textsize(char, font=font) 	# center text
	xt = (wb - wt) / 2
	yt = (hb - ht) / 2 - 10 	# 10px manual adjustment
	draw.text((xt, yt), char, 'black', font=font)

	for j in range(4):
		n = i * 4 + j 	# block number, first = 0
		#print("Block", n)

		# rotate incrementally
		if j > 0:
			block = block.rotate(90)

		# block top-left corner coordinates
		xb = 0 + blockSize * n
		yb = 0

		# add to main image
		block.save('test-images/' + char + '-' + str(j) + '.tif')
