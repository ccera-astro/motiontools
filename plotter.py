"""
============
Oscilloscope
============

Emulates an oscilloscope.
"""
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse


class Scope(object):
    def __init__(self, ax, maxt=3600, dt=1.0):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(-180.0, 180.0)
        self.ax.set_xlim(0, self.maxt)

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt:  # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,


def emitter(infile):
    fp = open(infile, "r")
    while True:
        line = fp.readline()
        if (len(line) > 5):
            v = float(line.replace("\n",""))
            yield v
        else:
            yield 0.0
        fp.seek(0)
        
parser = argparse.ArgumentParser(description="Plott elevation data")
parser.add_argument("--infile", type=str, help="Input file", default="/dev/null")
parser.add_argument("--axis", type=str, help="Axis name", default="ELEVATION")

args = parser.parse_args()


fig, ax = plt.subplots()
plt.grid(True)
plt.xlabel("Time(Seconds)")
plt.ylabel("Angle")
plt.title("Angle for axis %s" % args.axis)
scope = Scope(ax)

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter(args.infile), interval=1000,
                              blit=True)


plt.show()
