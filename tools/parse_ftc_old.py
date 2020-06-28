import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def get_time(x):
    x = x.split(' ')
    x_s = ' ';
    x_s = x_s.join(x[:2])
    #print(x_s)
    x = datetime.strptime(x_s, '%m-%d  %H:%M:%S.%f')
    return x;

n = 0
s = []
r = []
file1 = open("2.log")
for line in file1:
    if "setMotorPowers leftFront:" in line:
        t = line.split('setMotorPowers leftFront:');
        print(t[1])
        s.append(float(t[1]))
        r.append(get_time(line))

print('hi')

d = []
d.append(0)
length = len(r)
for i in range(length-1):
    t = r[i+1] - r[0]
    d.append(t.seconds + t.microseconds/1000000)
    print(d[i])

fig, ax = plt.subplots()
ax.plot(d, s)

ax.set(xlabel='time (s)', ylabel='leftFront power',
        title='motor power vs. time')
ax.grid()

plt.show()

print(len(s))
print(len(d))

ax.grid()

plt.show()