import os
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

from analysis_utils import LogReader
from plot_settings import *

DIR_DATA = os.path.join(os.path.dirname(__file__), 'data/velocity')

save_figures = True

# Plot one sample flight for each experiment
for drone_type in ['cf', 'fd_flat', 'fd_vz']:
    fname = drone_type + '_velocity_model.csv'
    f = os.path.join(DIR_DATA, fname)
    flight_data = LogReader(f)

    fig, ax = plt.subplots(3,1)
    ax[0].plot(flight_data.t, flight_data.ot[0].velocity[0], label='Optitrack', linewidth=1)
    ax[0].plot(flight_data.t, flight_data.ob.velocity[0], label='Estimate', linewidth=1)
    ax[0].set_ylabel(r'$v_x$ [m/s]')
    ax[1].plot(flight_data.t, flight_data.ot[0].velocity[1], label='Optitrack', linewidth=1)
    ax[1].plot(flight_data.t, flight_data.ob.velocity[1], label='Estimate', linewidth=1)
    ax[1].set_ylabel(r'$v_y$ [m/s]')
    ax[2].plot(flight_data.t, flight_data.ot[0].velocity[2], label='Optitrack', linewidth=1)
    ax[2].plot(flight_data.t, flight_data.ob.velocity[2], label='Estimate', linewidth=1)
    ax[2].set_ylabel(r'$v_z$ [m/s]')
    ax[2].legend(loc='lower left')
    ax[2].set_xlabel(r'$t$ [s]')
    for a in ax:
        a.set_ylim(-2.5, 2.5)
        a.grid(True)
    fig.tight_layout()

    if save_figures:
        save_name = drone_type + '_velocity.eps'
        fname = os.path.join(DIR_PLOTS, save_name)
        plt.savefig(fname, format='eps')
    

rmse_cf = [[],[],[]]
rmse_fd_flat = [[],[],[]]
rmse_fd_vz = [[],[],[]]

for fname in os.listdir(DIR_DATA):
    f = os.path.join(DIR_DATA, fname)
    flight_data = LogReader(f)

    ex = []
    ey = []
    ez = []
    
    for k,t in enumerate(flight_data.t):
        ex.append(flight_data.ob.velocity[0] - flight_data.ot[0].velocity[0])
        ey.append(flight_data.ob.velocity[1] - flight_data.ot[0].velocity[1])
        ez.append(flight_data.ob.velocity[2] - flight_data.ot[0].velocity[2])

    rmse_x = np.sqrt(np.mean(np.array(ex)**2))
    rmse_y = np.sqrt(np.mean(np.array(ey)**2))
    rmse_z = np.sqrt(np.mean(np.array(ez)**2))

    if 'cf' in fname:
        rmse_cf[0].append(rmse_x)
        rmse_cf[1].append(rmse_y)
        rmse_cf[2].append(rmse_z)

    elif 'fd_flat' in fname:
        rmse_fd_flat[0].append(rmse_x)
        rmse_fd_flat[1].append(rmse_y)
        rmse_fd_flat[2].append(rmse_z)

    elif 'fd_vz' in fname:
        rmse_fd_vz[0].append(rmse_x)
        rmse_fd_vz[1].append(rmse_y)
        rmse_fd_vz[2].append(rmse_z)


md_props = dict(color='black')
fig, ax = plt.subplots(1,1)
bp_kwargs = boxplot_kwargs_with_colors(*cf_colors)
ax.boxplot(rmse_cf, labels=[r'$v_x$', r'$v_y$', r'$v_z$'], positions=[1,2,3], **bp_kwargs)
bp_kwargs = boxplot_kwargs_with_colors(*fd_colors)
ax.boxplot(rmse_fd_flat, labels=[r'$v_x$', r'$v_y$', r'$v_z$'], positions=[5,6,7], **bp_kwargs)
bp_kwargs['boxprops']['hatch'] = '///'
ax.boxplot(rmse_fd_vz, labels=[r'$v_x$', r'$v_y$', r'$v_z$'], positions=[9,10,11], **bp_kwargs)

ax.grid(True, axis='y')
ax.set_ylim(0,0.5)
ax.set_ylabel('RMSE [m/s]')


legend_elements = [
    Patch(facecolor=cf_colors[0], edgecolor=cf_colors[1], label='Crazyflie'),
    Patch(facecolor='w', edgecolor='k', label='Level flight'),
    Patch(facecolor=fd_colors[0], edgecolor=fd_colors[1], label='Flapper Drone'),
    Patch(facecolor='w', edgecolor='k', hatch='///', label=r'Large $v_z$'),
]

ax.legend(handles=legend_elements, ncol=2, loc="lower right")

if save_figures:
    save_name = 'velocity_box.eps'
    fname = os.path.join(DIR_PLOTS, save_name)
    plt.savefig(fname, format='eps')

plt.show()