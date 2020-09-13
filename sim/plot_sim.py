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
    parser.add_argument("--output_dir", default="../output", required=False)

    args = parser.parse_args()

    output_dir = args.output_dir

    if not os.path.exists(output_dir):
        raise RuntimeError("directory does not exist {}".format(output_dir))

    truth_data = load(output_dir + "/truth", normalize_time=False)
    imu_data = load(output_dir + "/imu", normalize_time=False)
    obs_data = load(output_dir + "/obs", normalize_time=False)

    pw = plotWindow()
    pw.addPlot("2D", standard_plot(
        truth_data,
        ['x', 'x', 'trans', 1],
        ['x', 'x', 'trans', 0],
    ))
    pw.addPlot("Pos", standard_plot(truth_data, ['x', 'x', 'trans', [0, 1, 2]]))
    pw.addPlot("Rot", standard_plot(truth_data, ['x', 'x', 'rot', ['w', 'x', 'y', 'z']]))
    pw.addPlot("Vel", standard_plot(truth_data, ['x', 'dx', 'linear', [0, 1, 2]]))
    pw.addPlot("Omega", standard_plot(truth_data, ['x', 'dx', 'angular', [0, 1, 2]]))
    pw.addPlot("Acc", standard_plot(truth_data, ['x', 'd2x', 'linear', [0, 1, 2]]))
    pw.addPlot("Alpha", standard_plot(truth_data, ['x', 'd2x', 'angular', [0, 1, 2]]))

    pw.addPlot("Accel", standard_plot(imu_data, ['x', 'accel', [0, 1, 2]]))
    pw.addPlot("Gyro", standard_plot(imu_data, ['x', 'gyro', [0, 1, 2]]))

    plot_obs(pw, obs_data)

    pw.show()
