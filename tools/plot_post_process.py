import argparse
import os
from re import L
import numpy as np
import matplotlib.pyplot as plt

from common.logging.log_reader import load
from plotting.standard_plot import difference_signals, standard_plot, raw_plot
from plotting.plot_window import plotWindow
from plotting.plot_obs import plot_obs_residual
from plotting.plot_keys import LOG_KEYS

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="../logs", required=False)
    args = parser.parse_args()

    if not os.path.exists(args.dir):
        raise RuntimeError(f"directory does not exist {args.dir}")

    data = load(args.dir)
    est = data[LOG_KEYS["ESTIMATE"]]
    truth = data[LOG_KEYS["TRUTH_POSE"]]
    imu = data[LOG_KEYS["IMU_SAMPLE"]]
    gps_obs = data[LOG_KEYS["GPS_OBS_RESIDUAL"]]
    gal_obs = data[LOG_KEYS["GAL_OBS_RESIDUAL"]]
    glo_obs = data[LOG_KEYS["GLO_OBS_RESIDUAL"]]

    pw = plotWindow("Post Process")

    n = len(est)

    f = standard_plot(est, ['pose', 'trans', [0]], ['pose', 'trans', [1]], label="est")
    f = standard_plot(
        truth[:n], ['pose', 'x', 'trans', [0]], ['pose', 'x', 'trans', [1]], label="truth", fig=f
    )
    pw.addPlot("2D Position", f)

    f = standard_plot(est, ['pose', 'trans', [0, 1, 2]], label="est")
    f = standard_plot(truth[:n], ['pose', 'x', 'trans', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Translation", f)

    t_error = difference_signals(est, ['pose', 'trans'], truth[:n], ['pose', 'x', 'trans'])
    f = raw_plot(est['t'], t_error, label="difference")
    pw.addPlot("Translation Error", f)

    f = standard_plot(est, ['p_ecef', [0, 1, 2]], label="est")
    f = standard_plot(truth[:n], ['T_e2g', 'trans', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("ECEF Position", f)

    f = standard_plot(est, ['pose', 'rot', ['w', 'x', 'y', 'z']], label="est")
    f = standard_plot(truth[:n], ['pose', 'x', 'rot', ['w', 'x', 'y', 'z']], label="truth", fig=f)
    pw.addPlot("Rotation", f)

    f = standard_plot(est, ['vel', [0, 1, 2]], label="est")
    f = standard_plot(truth[:n], ['pose', 'dx', 'linear', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Velocity", f)

    f = standard_plot(est, ['omg', [0, 1, 2]], label="est")
    f = standard_plot(truth[:n], ['pose', 'dx', 'angular', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Omega", f)

    f = standard_plot(est, ['acc', [0, 1, 2]], label="est")
    f = standard_plot(imu[:n], ['imu', 'accel', [0, 1, 2]], label="truth", fig=f)
    pw.addPlot("Accel", f)

    f = standard_plot(est, ['gps_clk', [0, 1]], label="gps")
    f = standard_plot(est, ['gal_clk', [0, 1]], label="gal", fig=f)
    f = standard_plot(est, ['gal_clk', [0, 1]], label="gal", fig=f)
    f = standard_plot(truth[:n], ['clk', [0, 1]], label="truth", fig=f)
    pw.addPlot("Clock", f)

    pw.addPlot("gps", plot_obs_residual(gps_obs))
    pw.addPlot("gal", plot_obs_residual(gal_obs))
    pw.addPlot("glo", plot_obs_residual(glo_obs))

    pw.show()

    pass
