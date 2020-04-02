
import numpy as np
import matplotlib.pyplot as plt

from common.logging import log_reader
from utils import plot_window, plotting


if __name__== "__main__":
    data = log_reader.load("/tmp/ImuModel/integrateRandomImu.log")
    pw = plot_window.plotWindow("integrateRandomImu")

    f = plt.figure()
    plt.plot(data['t'])

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3,1,i+1, sharex=ax)
        plt.plot(data['t'], data['x']['x']['trans'][:,i], label="x")
        plt.plot(data['t'], data['xhat']['trans'][:,i], label="xhat")
        plt.legend()
    pw.addPlot("Position", f)

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3,1,i+1, sharex=ax)
        plt.plot(data['t'], data['x']['dx']['linear'][:,i], label="x")
        plt.plot(data['t'], data['vhat'][:,i], label="xhat")
        plt.legend()
    pw.addPlot("Velocity", f)

    f = plt.figure()
    ax = None
    for i in range(3):
        ax = plt.subplot(3,1,i+1, sharex=ax)
        plt.plot(data['t'], data['x']['dx']['angular'][:,i], label="x")
        plt.plot(data['t'], data['vhat'][:,i], label="xhat")
        plt.legend()
    pw.addPlot("Omega", f)

    pw.show()
