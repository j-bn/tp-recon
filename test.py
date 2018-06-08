print("tests...")

a = [[1,2],[3,4],[5,6]]

for x in a:
	if x[0] == 5:
		x[1] = 100

print(a)
# YES, for each loops do pass non-primitive elements by reference so modifications are passed back and saved