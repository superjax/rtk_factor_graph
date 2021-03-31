import sys
sys.path.append('/home/james/Code/globalai')

import numpy as np
import numpy.lib.recfunctions
import matplotlib.pyplot as plt

from common.logging import log_reader
from common.logging.log_reader import load


def extract(field, key):
    if isinstance(key, str):
        return field[key]
    elif isinstance(key, int):
        return field[:, key, 0]
    else:
        raise RuntimeError("Unknown key")


def bottom_field(data, keys):
    if type(keys) is list or type(keys) is tuple:
        field = data
        for key in keys:
            if type(key) is list or type(key) is tuple:
                field = np.vstack([extract(field, k) for k in key]).T
            else:
                field = extract(field, key)
    else:
        field = extract(data, keys)
    if len(field.shape) > 2:
        field = field[:, :, 0]
    return field


def standard_plot(data, y_key, x_key="t", title=None, fig=None, label=None, **kwargs):
    xfield = bottom_field(data, x_key)
    yfield = bottom_field(data, y_key)

    # Re-use existing figure if set
    if fig is None:
        fig = plt.figure()
    else:
        fig = plt.figure(fig.number)

    # Set title
    if title is not None:
        plt.suptitle(title)

    # Draw data
    if len(yfield.shape) > 1:
        num_plots = yfield.shape[1]
        ax = None
        for i in range(num_plots):
            ax = plt.subplot(num_plots, 1, i + 1, sharex=ax)
            plt.plot(xfield, yfield[:, i], label=label, **kwargs)
            if label is not None and i == 0:
                plt.legend()
    else:
        plt.plot(xfield, yfield, label=label, **kwargs)
        if label is not None:
            plt.legend()

    return fig
