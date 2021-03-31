import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

from common.logging.log_reader import load
from plotting.standard_plot import standard_plot
from plotting.plot_window import plotWindow
from plotting.plot_obs import plot_obs

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="../logs", required=False)
    args = parser.parse_args()

    if not os.path.exists(args.dir):
        raise RuntimeError(f"directory does not exist {args.dir}")

    data = load(args.dir)

    pw = plotWindow("Post Process")

    n = len(data[10])

    f = standard_plot(data[10], ['pose', 'trans', [0]], ['pose', 'trans', [1]], label="est")
    f = standard_plot(
        data[5][:n], ['pose', 'x', 'trans', [0]], ['pose', 'x', 'trans', [1]], label="truth", fig=f
    )
    pw.addPlot("2D Position", f)

    f = standard_plot(data[10], ['pose', 'trans', [0, 1, 2]], label="est")
    f = standard_plot(data[5][:n], ['pose', 'x', 'trans', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Translation", f)

    f = standard_plot(data[10], ['pose', 'rot', ['w', 'x', 'y', 'z']], label="est")
    f = standard_plot(data[5][:n], ['pose', 'x', 'rot', ['w', 'x', 'y', 'z']], label="truth", fig=f)
    pw.addPlot("Rotation", f)

    f = standard_plot(data[10], ['vel', [0, 1, 2]], label="est")
    f = standard_plot(data[5][:n], ['pose', 'dx', 'linear', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Velocity", f)

    f = standard_plot(data[10], ['omg', [0, 1, 2]], label="est")
    f = standard_plot(data[5][:n], ['pose', 'dx', 'angular', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Omega", f)

    f = standard_plot(data[10], ['acc', [0, 1, 2]], label="est")
    f = standard_plot(data[0][:n], ['imu', 'accel', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Accel", f)

    pw.show()

    pass
