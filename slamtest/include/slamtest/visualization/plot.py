import sys
import matplotlib.pyplot as plt
import numpy as np

next_fig_idx = 0
figs = {}
axes = {}

def RemoveFigId(fig_id):
    del figs[fig_id]
    del axes[fig_id]

def AddFigure(fig_title, force_square):
    global next_fig_idx
    figs[next_fig_idx], axes[next_fig_idx] = plt.subplots(1,1)
    if force_square:
        axes[next_fig_idx].axis('equal')
    figs[next_fig_idx].suptitle(fig_title)
    f_id = next_fig_idx
    figs[next_fig_idx].canvas.mpl_connect('close_event', lambda evt: RemoveFigId(f_id))
    next_fig_idx = next_fig_idx + 1
    return f_id

def RedrawFigure(fig_idx):
    if not fig_idx in figs.keys():
        print("Invalid Figure index in RedrawFigure:", fig_idx, file=sys.stderr)
        return
    figs[fig_idx].canvas.draw()

def Plot(fig_idx, Xs, fmt_string):
    if not fig_idx in figs.keys():
        print("Invalid Figure index in Plot:", fig_idx, file=sys.stderr)
        return
    state = np.zeros((2,len(Xs)))
    for i in range(len(Xs)):
        x_t = Xs[i]
        state[0,i] = x_t[0]
        state[1,i] = x_t[1]
    axes[fig_idx].plot(state[0,:],state[1,:],fmt_string)
    RedrawFigure(fig_idx)

def SetAxisLims(fig_idx, x_lim, y_lim):
    if not fig_idx in figs.keys():
        print("Invalid Figure index in SetAxisLims:", fig_idx, file=sys.stderr)
        return
    axes[fig_idx].set_xlim(x_lim)
    axes[fig_idx].set_ylim(y_lim)

def ClearFigure(fig_idx):
    if not fig_idx in figs.keys():
        print("Invalid Figure index in ClearFigure:", fig_idx, file=sys.stderr)
        return
    axes[fig_idx].clear()

def ShowFigures():
    for k in figs.keys():
        figs[k].show()

def PauseFigures(t):
    if t < 0:
        plt.show()
    else:
        plt.pause(t)