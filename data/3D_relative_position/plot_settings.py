import os
import matplotlib.pyplot as plt
from pathlib import Path

DIR_PLOTS = os.path.join(os.path.dirname(__file__), 'plots')
Path(DIR_PLOTS).mkdir(exist_ok=True)

# Set parameters for plot fonts
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Computer Modern Roman"],
})

boxplot_kwargs = {
    'widths': 0.5,
    'medianprops': dict(color='black')
}

cf_colors = ['#abdda4', '#50a044']  # green
fd_colors = ['#2b83ba', '#075181']  # blue

def boxplot_kwargs_with_colors(face_color, line_color):
    bp_kwargs = {
        'widths': 0.5,
        'patch_artist': True,
        'boxprops': dict(color=line_color, facecolor=face_color),
        'medianprops': dict(color=line_color),
        'capprops': dict(color=line_color),
        'flierprops': dict(markeredgecolor=line_color),
        'whiskerprops': dict(color=line_color)
    }
    return bp_kwargs