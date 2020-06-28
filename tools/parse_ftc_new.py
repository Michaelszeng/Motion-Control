import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def get_time(x):
    x = x.split(' ')
    x_s = ' ';
    x_s = x_s.join(x[:2])
    #print(x_s)
    x = datetime.strptime(x_s, '%Y-%m-%d  %H:%M:%S.%f')
    return x;

n = 0
s = []
r = []
file1 = open("test01.txt")
for line in file1:
    if "leftFront: setPower" in line:
        t = line.split('setPower');
        s.append(float(t[1]))
        r.append(get_time(line))

d = []
d.append(0)
length = len(r)
for i in range(length-1):
    t = r[i+1] - r[0]
    d.append(t.microseconds/1000000)
    print(d[i])

fig, ax = plt.subplots()
plt.plot(d)

print(len(s))
print(len(d))

ax.grid()

plt.show()