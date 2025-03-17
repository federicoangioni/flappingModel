import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

from analysis_utils import LogReader, wrap_yaw
from plot_settings import *

save_figures = False

dir_names = {'rl': 'relative_localization', 'lf': 'leader_follower'}
drone_types = ['cf', 'fd']

def get_ot_relative_position(log: LogReader):
    N = len(log.t)
    yaw0 = log.ot[0].attitude[2]
    rel_pos_global = log.ot[1].position - log.ot[0].position
    
    rel_pos_loc = np.array([
        np.cos(yaw0)*rel_pos_global[0] + np.sin(yaw0)*rel_pos_global[1],
        -np.sin(yaw0)*rel_pos_global[0] + np.cos(yaw0)*rel_pos_global[1],
        rel_pos_global[2]
    ])
    
    rel_yaw = log.ot[1].attitude[2] - log.ot[0].attitude[2]

    return np.array([*rel_pos_loc, rel_yaw])


for exp, dir_name in dir_names.items():
    DIR_DATA = os.path.join(os.path.dirname(__file__), 'data', dir_name)
    for d_type in drone_types:
        # Plot one sample flight for each experiment
        fname = d_type + '_' + dir_name + '.csv'

        f = os.path.join(DIR_DATA, fname)
        flight_data = LogReader(f)

        rel_pos_ot = get_ot_relative_position(flight_data)

        # relative position
        if exp == 'rl':
            fig, ax = plt.subplots(4,1)
            ax[0].plot(flight_data.t, rel_pos_ot[0], label='Optitrack', linewidth=1.0)
            ax[0].plot(flight_data.t, flight_data.relPos[0], label='Estimate', linewidth=1.0)
            ax[0].set_ylabel(r'$x_{rel}$ [m]')
            ax[1].plot(flight_data.t, rel_pos_ot[1], label='Optitrack', linewidth=1.0)
            ax[1].plot(flight_data.t, flight_data.relPos[1], label='Estimate', linewidth=1.0)
            ax[1].set_ylabel(r'$y_{rel}$ [m]')
            ax[2].plot(flight_data.t, rel_pos_ot[2], label='Optitrack', linewidth=1.0)
            ax[2].plot(flight_data.t, flight_data.relPos[2], label='Estimate', linewidth=1.0)
            ax[2].set_ylabel(r'$z_{rel}$ [m]')
            ax[3].plot(flight_data.t, wrap_yaw(rel_pos_ot[3]), label='Optitrack', linewidth=1.0)
            ax[3].plot(flight_data.t, wrap_yaw(flight_data.relPos[3]), label='Estimate', linewidth=1.0)
            ax[3].set_ylabel(r'$\psi_{rel}$ [rad]')
            ax[3].set_xlabel(r'$t$ [s]')
            ax[3].legend(ncol=2, loc='lower right')
            for a in ax:
                a.grid(True)
            fig.tight_layout()
            ax[0].set_ylim(-5.5,5.5)
            ax[1].set_ylim(-5.5,5.5)
            ax[2].set_ylim(-5.5,5.5)
            ax[3].set_ylim(-5.5,5.5)

        # absolute position
        elif exp == 'lf':
            fig, ax = plt.subplots(3,1)
            ax[0].plot(flight_data.t, flight_data.ot[0].position[0], label='Leader', linewidth=1.0)
            ax[0].plot(flight_data.t, flight_data.ot[1].position[0], label='Follower', linewidth=1.0)
            ax[0].plot(flight_data.t, flight_data.ot[0].position[0]-2.0, 
                        linestyle='--', color='k', linewidth=0.6, label='Follower Target')
            ax[0].set_ylabel(r'$x$ [m]')
            ax[1].plot(flight_data.t, flight_data.ot[0].position[1], label='Leader', linewidth=1.0)
            ax[1].plot(flight_data.t, flight_data.ot[1].position[1], label='Follower', linewidth=1.0)
            ax[1].plot(flight_data.t, flight_data.ot[0].position[1], 
                        linestyle='--', color='k', linewidth=0.6, label='Follower Target')
            ax[1].set_ylabel(r'$y$ [m]')
            ax[2].plot(flight_data.t, flight_data.ot[0].position[2], label='Leader', linewidth=1.0)
            ax[2].plot(flight_data.t, flight_data.ot[1].position[2], label='Follower', linewidth=1.0)
            ax[2].plot(flight_data.t, flight_data.ot[0].position[2]-0.2, 
                        linestyle='--', color='k', linewidth=0.6, label='Follower Target')
            ax[2].set_ylabel(r'$z$ [m]')
            ax[2].set_xlabel(r'$t$ [s]')
            ax[2].legend(ncol=3, loc='lower right')
            for a in ax:
                a.grid(True)
            fig.tight_layout()
            ax[0].set_ylim(-4,4)
            ax[1].set_ylim(-4,4)
            ax[2].set_ylim(-4,4)

        if save_figures:
            save_name = d_type + '_' + dir_name + '.eps'
            fname = os.path.join(DIR_PLOTS, save_name)
            plt.savefig(fname, format='eps')


# Data for rmse boxplot
rmse_plt_data = dict(
    cf_rl = [[],[],[],[]],
    cf_lf = [[],[],[],[]],
    fd_rl = [[],[],[],[]],
    fd_lf = [[],[],[],[]]
)
for exp, dir_name in dir_names.items():
    DIR_DATA = os.path.join(os.path.dirname(__file__), 'data', dir_name)
    for fname in os.listdir(DIR_DATA):
        if fname.startswith('.'):
            continue

        if 'cf' in fname:
            drone_type = 'cf'
        elif 'fd' in fname:
            drone_type = 'fd'
                
        key = drone_type + '_' + exp

        f = os.path.join(DIR_DATA, fname)
        flight_data = LogReader(f)

        # relative localization error
        rel_pos_ot = get_ot_relative_position(flight_data)
        rel_pos_error = flight_data.relPos - rel_pos_ot
        rel_pos_error[3] = wrap_yaw(flight_data.relPos[3]) - wrap_yaw(rel_pos_ot[3])

        # rel loc rmse after 30s of flight until 5s before end of flight
        flight_data.prune_data(flight_data.ot[1].position[2]>0.1)
        flight_data.prune_data(flight_data.t > flight_data.t[0]+30)
        flight_data.prune_data(flight_data.t < flight_data.t[-1]-5)

        rel_pos_ot = get_ot_relative_position(flight_data)
        rel_pos_error = flight_data.relPos - rel_pos_ot
        rel_pos_error[3] = wrap_yaw(flight_data.relPos[3]) - wrap_yaw(rel_pos_ot[3])

        rmse_plt_data[key][0].append(np.sqrt(np.mean(rel_pos_error[0]**2)))
        rmse_plt_data[key][1].append(np.sqrt(np.mean(rel_pos_error[1]**2)))
        rmse_plt_data[key][2].append(np.sqrt(np.mean(rel_pos_error[2]**2)))
        rmse_plt_data[key][3].append(np.sqrt(np.mean(rel_pos_error[3]**2)))

fig, ax = plt.subplots(1,1)
ax_yaw = ax.twinx()

bp_labels = [r'$x_{rel}$', r'$y_{rel}$', r'$z_{rel}$', r'$\psi_{rel}$']

# cf rel loc
bp_kwargs = boxplot_kwargs_with_colors(*cf_colors)
ax.boxplot(rmse_plt_data['cf_rl'][0:3], labels=bp_labels[0:3], 
            positions=[1,2,3], **bp_kwargs)
cf_rl_bp = ax_yaw.boxplot(rmse_plt_data['cf_rl'][3], labels=[bp_labels[3]], 
            positions=[4], **bp_kwargs)

# fd rel loc
bp_kwargs = boxplot_kwargs_with_colors(*fd_colors)
ax.boxplot(rmse_plt_data['fd_rl'][0:3], labels=bp_labels[0:3], 
            positions=[6,7,8], **bp_kwargs)
fd_rl_bp = ax_yaw.boxplot(rmse_plt_data['fd_rl'][3], labels=[bp_labels[3]], 
            positions=[9], **bp_kwargs)

# cf leader follower
bp_kwargs = boxplot_kwargs_with_colors(*cf_colors)
bp_kwargs['boxprops']['hatch'] = '///'
ax.boxplot(rmse_plt_data['cf_lf'][0:3], labels=bp_labels[0:3], 
            positions=[11,12,13], **bp_kwargs)
cf_lf_bp = ax_yaw.boxplot(rmse_plt_data['cf_lf'][3], labels=[bp_labels[3]], 
            positions=[14], **bp_kwargs)

# fd leader follower
bp_kwargs = boxplot_kwargs_with_colors(*fd_colors)
bp_kwargs['boxprops']['hatch'] = '///'
ax.boxplot(rmse_plt_data['fd_lf'][0:3], labels=bp_labels[0:3], 
            positions=[16,17,18], **bp_kwargs)
fd_lf_bp = ax_yaw.boxplot(rmse_plt_data['fd_lf'][3], labels=[bp_labels[3]], 
            positions=[19], **bp_kwargs)

legend_elements = [
    Patch(facecolor=cf_colors[0], edgecolor=cf_colors[1], label='Crazyflie'),
    Patch(facecolor='w', edgecolor='k', label='Free flight'),
    Patch(facecolor=fd_colors[0], edgecolor=fd_colors[1], label='Flapper Drone'),
    Patch(facecolor='w', edgecolor='k', hatch='///', label='Leader-Follower'),
]

ax.legend(handles=legend_elements, ncol=2, loc="lower left")

ax.set_ylabel('RMSE relative position [m]')
ax_yaw.set_ylabel('RMSE relative yaw [rad]')
ax.set_ylim(0,1.5)
ax_yaw.set_ylim(0,1.5)

ax.grid(True, axis='y')

if save_figures:
    save_name = 'rel_loc_box.eps'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='eps')

plt.show()