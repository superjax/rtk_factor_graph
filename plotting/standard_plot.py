import sys
sys.path.append('/home/james/Code/globalai')

import numpy as np
import numpy.lib.recfunctions
import matplotlib.pyplot as plt

from common.logging import log_reader
from common.logging.log_reader import load


def bottom_field(data, keys):
    if type(keys) is list or type(keys) is tuple:
        field = data
        for key in keys:
            if type(key) is list or type(key) is tuple:
                print([k for k in key])
                field = np.vstack([field[k] for k in key]).T
            else:
                field = field[key]
    else:
        field = data[keys]
    if len(field.shape) > 2:
        field = field[:, :, 0]
    return field


def standard_plot(name, data, y_key, x_key="t"):
    xfield = bottom_field(data, x_key)
    yfield = bottom_field(data, y_key)
    num_plots = yfield.shape[1]

    f = plt.figure()
    plt.suptitle(name)
    for i in range(num_plots):
        plt.suptitle
        plt.subplot(num_plots, 1, i + 1)
        plt.plot(xfield, yfield[:, i])
    return f

