import numpy as np
import matplotlib.pyplot as plt

GNSS_ID_TO_NAME = {0: "GPS", 1: "SBAS", 2: "GAL", 3: "COM", 5: "QZSS", 6: "GLO"}


def plot_obs(pw, obs):
    gnss_ids = np.unique(obs['x']['gnss_id'])

    for g in gnss_ids:
        o = obs[obs['x']['gnss_id'] == g]
        sat_freqs = np.unique(np.vstack((o['x']['freq'], o['x']['sat_num'])), axis=1)

        f = None
        for i, (freq, sat) in enumerate(sat_freqs.T):
            if freq == 0:
                continue
            if f == None:
                f, axs = plt.subplots(nrows=len(sat_freqs.T), ncols=3, sharex=True, sharey=False)
            os = o[(o['x']['sat_num'] == sat) & (o['x']['freq'] == freq)]

            fields = ["pseudorange", "doppler", "carrier_phase"]
            row = axs[i]
            for j, (ax, field) in enumerate(zip(row, fields)):
                ax.plot(os['t'], os['x'][field])
                if j == 0:
                    ax.set_ylabel(GNSS_ID_TO_NAME[g] + ":" + str(sat))
                if i == 0:
                    ax.set_title(field)
                ax.grid()
        if f is not None:
            pw.addPlot(GNSS_ID_TO_NAME[g], f)


def plot_obs_residual(obs):
    f = plt.figure()

    sat_ids = np.unique(obs['sat_id'])

    assert len(sat_ids) < 30
    print(sat_ids)
    for i, sat in enumerate(sat_ids):
        plt.subplot(len(sat_ids), 2, i * 2 + 1)
        residuals = obs[obs['sat_id'] == sat]
        # plt.plot(residuals['t'], residuals['z'][:, 0], label="z")
        # plt.plot(residuals['t'], residuals['zhat'][:, 0], 'x', label="zhat")
        plt.plot(residuals['t'], residuals['res'][:, 0], label="residual")
        if i == 0:
            plt.suptitle('prange')
            plt.legend()

        plt.subplot(len(sat_ids), 2, i * 2 + 2)
        # plt.plot(residuals['t'], residuals['z'][:, 1], label="z")
        # plt.plot(residuals['t'], residuals['zhat'][:, 1], 'x', label="zhat")
        plt.plot(residuals['t'], residuals['res'][:, 1], label="residual")
        if i == 0:
            plt.suptitle('doppler')
            plt.legend()

    return f
