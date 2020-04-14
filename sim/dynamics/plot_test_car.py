#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
from utils.plot_window import plotWindow

structure = np.dtype([("t", np.float64),
                      ("pos", (np.float64, 3)),
                      ("rot", (np.float64, 3)),
                      ("omg", (np.float64, 3)),
                      ("vel", (np.float64, 3)),
                      ("alpha", (np.float64, 3)),
                      ("acc", (np.float64, 3)),
                      ("u", (np.float64, 2)),
                      ("vw", (np.float64, 2)),
                      ("wp", (np.float64, 3))
                      ])

# data = np.fromfile("/tmp/mc/SimCar.Random.log", dtype=structure)
# data = np.fromfile("/tmp/mc/SimCar.Controller.log", dtype=structure)
data = np.fromfile("/tmp/mc/SimCar.Waypoints.log", dtype=structure)

pw = plotWindow()


def make_plot(key, labels, extra_plot=None, extra_line=None, extra_label=None):
    f = plt.figure()
    for i, label in enumerate(labels):
        plt.subplot(3, 1, i+1)
        plt.plot(data['t'], data[key][:, i], label="x")
        if i == extra_plot:
            plt.plot(data['t'], extra_line, label=extra_label)
            plt.legend()
        plt.ylabel(label)
    plt.xlabel('t')
    return f


def plot_2d():
    f = plt.figure()
    plt.plot(data['pos'][:, 0], data['pos'][:, 1])
    plt.plot(data['wp'][:, 0], data['wp'][:, 1], 'x')
    return f


pw.addPlot("2D Pos", plot_2d())
pw.addPlot("Position", make_plot('pos', ['tx', 'ty', 'tz']))
pw.addPlot("Rotation", make_plot('rot', ['rx', 'ry', 'rz']))
pw.addPlot("Velocity", make_plot(
    'vel', ['vx', 'vy', 'vz'], 0, data['vw'][:, 0], "d"))
pw.addPlot("Omega", make_plot(
    'omg', ['ωx', 'ωy', 'ωz'], 2, data['vw'][:, 1], "d"))
pw.addPlot("Accel", make_plot('acc', ['ax', 'ay', 'az']))
pw.addPlot("Alpha", make_plot('alpha', ['αx', 'αy', 'αz']))
pw.addPlot("u", make_plot('u', ['τl', 'τr']))

pw.show()


pass
