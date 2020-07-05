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

def plot_graph(filename, parameter):
    n = 0
    s = []
    r = []

    file1 = open(filename)
    for line in file1:
        if parameter in line:
            t = line.split(parameter);
            print(t[1])
            s.append(float(t[1]))
            r.append(get_time(line))

    d = []
    d.append(0)
    length = len(r)
    for i in range(length - 1):
        t = r[i + 1] - r[0]
        d.append(t.seconds + t.microseconds / 1000000)
        print(d[i])

    # fig, ax = plt.subplots()
    # ax.plot(d, s)
    plt.style.use('ggplot')

    plt.figure()
    plt.plot(d, s, label = " ")
    plt.legend();
    plt.xlabel('time (s)')
    plt.ylabel(parameter)

    plt.show()

    print(len(s))
    print(len(d))

plot_graph('2.log', 'setMotorPowers leftRear:')
plot_graph('2.log', 'setMotorPowers leftFront:')
plot_graph('2.log', 'setMotorPowers rightRear:')
plot_graph('2.log', 'setMotorPowersrightFront:')

plot_graph('2.log', 'headingError ')
plot_graph('2.log', 'xError ')
plot_graph('2.log', 'yError ')
