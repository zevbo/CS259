import math
import matplotlib.pyplot as plt

def sign_of(i):
	if i > 0:
		return 1
	else:
		return -1
			

a = 100
step = 2
size = int(a / step)
xs1 = [i * step for i in range(-size, size + 1)]
ys1 = [math.sqrt((a ** 2 * x ** 2 - x ** 4)/(a**2)) * sign_of(x) for x in xs1]
xs2 = list(reversed(xs1))
ys2 = list(map(lambda p: -1 * p, reversed(ys1)))
ys = ys1 + ys2 
xs = xs1 + xs2

starting = 0
for i, x in enumerate(xs):
    if x == 0:
        starting = i
        break

for i, (x, y) in enumerate(zip(xs, ys)):
    index = i - starting
    if index < 0:
        index += len(xs)
    index += 1
    print("waypoints(2:3," + str(index) + ") = [" + str(x - 100) + "; " + str(y) + "];")

plt.plot(xs, ys)
plt.show()