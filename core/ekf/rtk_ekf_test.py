import numpy as np
import matplotlib.pyplot as plt

from common.logging import log_reader
from utils import plot_window, plotting

if __name__ == "__main__":
    data = log_reader.load("/tmp/RtkEkfTest")
    pw = plot_window.plotWindow("errorStateDynamicsTest")

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3, 1, i + 1, sharex=ax)
        plt.plot(data['t'], data['x']['trans'][:, i], label="x")
        plt.plot(data['t'], data['xhat']['trans'][:, i], label="xhat")
        plt.plot(data['t'], data['x_check']['trans'][:, i], label="x_check")
        plt.legend()
    pw.addPlot("Position", f)

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3, 1, i + 1, sharex=ax)
        plt.plot(data['t'], data['x']['trans'][:, i] - data['x_check']['trans'][:, i], label="x")
        plt.legend()
    pw.addPlot("Position Error", f)

    f = plt.figure()
    ax = None
    quat_keys = ['w', 'x', 'y', 'z']
    for i in range(4):
        ax = plt.subplot(4, 1, i + 1, sharex=ax)
        plt.plot(data['t'], data['x']['rot'][quat_keys[i]], label="x")
        plt.plot(data['t'], data['xhat']['rot'][quat_keys[i]], label="xhat")
        plt.plot(data['t'], data['x_check']['rot'][quat_keys[i]], label="x_check")
        plt.legend()
    pw.addPlot("Rotation", f)

    f = plt.figure()
    ax = None
    for i in range(4):
        ax = plt.subplot(4, 1, i + 1, sharex=ax)
        plt.plot(
            data['t'],
            data['x']['rot'][quat_keys[i]] - data['x_check']['rot'][quat_keys[i]],
            label="x"
        )
        plt.legend()
    pw.addPlot("Rotation Error", f)

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3, 1, i + 1, sharex=ax)
        plt.plot(data['t'], data['v'][:, i], label="v")
        plt.plot(data['t'], data['vhat'][:, i], label="vhat")
        plt.plot(data['t'], data['vcheck'][:, i], label="v_check")
        plt.legend()
    pw.addPlot("Velocity", f)

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3, 1, i + 1, sharex=ax)
        plt.plot(data['t'], data['v'][:, i] - data['vcheck'][:, i], label="v")
        plt.legend()
    pw.addPlot("Velocity Error", f)

    pw.show()
