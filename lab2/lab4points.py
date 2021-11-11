import math
import matplotlib.pyplot as plt

def sign_of(i):
	if i > 0:
		return 1
	else:
		return -1
		
a = 100	

def f(x):
    return math.sqrt((a ** 2 * x ** 2 - x ** 4)/(a**2))

# a^2y^2 = x^2a^2 - x^4
# 2a^2yy' = 2xa^2 - 3x^3
# y' = (2xa^2 - 3x^3)/(2a^2y)

def deriv(x):
    f_val = f(x)
    threshold = a / 1000
    y = (sign_of(f_val) * threshold) if abs(f_val) < threshold else f_val
    return (2*x*a**2 - 3*x**3) / (2*a**2*y)
def speed_at(x):
    return math.sqrt(1 + deriv(x) ** 2.0)

speed = 20
xs1 = []
x = -a
while(x <= 0):
    xs1.append(x)
    x += speed / speed_at(x)
xs1 = xs1 + list(reversed(list(map(lambda n: -1 * n, xs1))))
ys1 = [f(x) * sign_of(x) for x in xs1]
xs2 = list(reversed(xs1))
ys2 = list(map(lambda p: -1 * p, reversed(ys1)))
ys = ys1 + ys2 
xs = xs1 + xs2

starting = 0
for i, x in enumerate(xs):
    if x >= 0:
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