import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import sys


print("input log name",
       sys.argv[1])
x = str(sys.argv[1]) #name of log file

def get_time(x):
    x = x.split(' ')
    x_s = ' ';
    x_s = x_s.join(x[:2])
    #print(x_s)
    x = datetime.strptime(x_s, '%m-%d  %H:%M:%S.%f')
    return x;

def plot_graph(filename, parameter, singlePrint):
    n = 0
    s = []
    r = []

    file1 = open(filename)
    for line in file1:
        if parameter in line:
            #print("line: ", line)
            #print(filename)
            t = line.split(parameter);
            #print("Hello: ", t, filename, singlePrint, parameter)
            s.append(float(t[1]))
            r.append(get_time(line))

    d = []
    d.append(0)
    length = len(r)
    for i in range(length - 1):
        t = r[i + 1] - r[0]
        d.append(t.seconds + t.microseconds / 1000000)
        #print(d[i])

    # fig, ax = plt.subplots()
    # ax.plot(d, s)

    if singlePrint == 1:
        plt.style.use('ggplot')
        plt.figure()
        plt.plot(d, s, label = " ")
        plt.legend();
        plt.xlabel('time (s)')
        plt.ylabel(parameter)
        plt.show()
        #print(len(s))
        #print(len(d))
    elif singlePrint == 2:
        #print("one")
        return d,s;
    else:
        print("error, stupid")


[d1, s1] = plot_graph(x, 'setMotorPowers leftRear:', 2)
[d2, s2] = plot_graph(x, 'setMotorPowers leftFront:', 2)
[d3, s3] = plot_graph(x, 'setMotorPowers rightRear:', 2)
[d4, s4] = plot_graph(x, 'setMotorPowersrightFront:', 2)

plt.style.use('ggplot')
plt.figure()
plt.plot(d1, s1, label="leftRear")
plt.plot(d2, s2, label="leftFront")
plt.plot(d3, s3, label="rightRear")
plt.plot(d4, s4, label="rightFront")
plt.legend();
plt.xlabel('time (s)')
plt.ylabel('motorPower')
plt.show()

plot_graph(x, 'headingError ', 1)
plot_graph(x, 'xError ', 1)
plot_graph(x, 'yError ', 1)