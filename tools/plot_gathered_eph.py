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

    output_dir = args.dir

    if not os.path.exists(output_dir):
        raise RuntimeError("directory does not exist {}".format(output_dir))

    obs_data = load(output_dir + "/obs", normalize_time=True)

    pw = plotWindow()
    plot_obs(pw, obs_data)

    pw.show()
