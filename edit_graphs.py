"""
edit_graphs.py

This file contains code to load and edit pickled matplotlib figures.
"""

import pickle
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import glob

def edit_towing_agents():
    """
    Polishes up the graphs produced from the towing agent simulations. Edit anything below as needed.
    """
    # load the pickled figure
    with open("./data/figures/towing_agents_graph_2025-08-03.png.pkl", "rb") as f:
        fig = pickle.load(f)

    # change graph colors of towing agents
    legend_colors = ["red", "yellow", "deepskyblue", "blue", "lime"]

    # access all axes in the figure
    for ax in fig.axes:
        # change title font size
        # ax.set_title("Failed Agent X-Position Over Time")
        ax.set_title("")
        ax.title.set_fontsize(30)

        ax.set_ylabel("X-Position")

        # change x/y label font sizes
        ax.xaxis.label.set_fontsize(30)
        ax.yaxis.label.set_fontsize(30)

        ax.set_xlim(0, 5000)
        ax.set_ylim(-4, 30)

        # change tick label font sizes
        for tick in ax.xaxis.get_major_ticks():
            tick.label1.set_fontsize(30)
        for tick in ax.yaxis.get_major_ticks():
            tick.label1.set_fontsize(30)

        # ax.lines → list of Line2D objects (for line plots)
        for i, line in enumerate(ax.lines):
            color = legend_colors[i % len(legend_colors)]
            line.set_color(color)

        # change legend font size (if it exists)
        legend = ax.get_legend()
        handles, labels = ax.get_legend_handles_labels()
        if legend:
            # labels for the towing agents legend
            new_labels = ["I", "II", "III", "IV", "V"]
            ax.legend(handles, new_labels)
            for text in ax.get_legend().get_texts():
                text.set_fontsize(30)

    # optionally show or save again
    # plt.show()
    fig.savefig("new_figure.png", dpi=300, bbox_inches="tight")

def edit_obj_capture():
    """
    Polishes up the graphs produced from the object capture simulations. Edit anything below as needed.
    """
    # load the pickled figure
    with open("./data/figures/object_capture_maintain_line_False_offset0_graph_2025-08-03.png.pkl", "rb") as f:
        fig = pickle.load(f)

    # legend_colors = ["yellow", "red", "purple", "blue"]

    # load files and calculate the std deviations 
    files_5 = glob.glob("./data/8-3-25_to_8-5-25/object_capture_maintain_line_False_trial*_objects5_offset0.csv")
    dfs_5 = [pd.read_csv(f) for f in files_5]
    trial_matrix_5 = np.column_stack([df["# of objects collected"].values for df in dfs_5])
    std_5  = np.std(trial_matrix_5, axis=1)

    files_10 = glob.glob("./data/8-3-25_to_8-5-25/object_capture_maintain_line_False_trial*_objects10_offset0.csv")
    dfs_10 = [pd.read_csv(f) for f in files_10]
    trial_matrix_10 = np.column_stack([df["# of objects collected"].values for df in dfs_10])
    std_10  = np.std(trial_matrix_10, axis=1)

    files_30 = glob.glob("./data/8-3-25_to_8-5-25/object_capture_maintain_line_False_trial*_objects30_offset0.csv")
    dfs_30 = [pd.read_csv(f) for f in files_30]
    trial_matrix_30 = np.column_stack([df["# of objects collected"].values for df in dfs_30])
    std_30  = np.std(trial_matrix_30, axis=1)

    files_50 = glob.glob("./data/8-3-25_to_8-5-25/object_capture_maintain_line_False_trial*_objects50_offset0.csv")
    dfs_50 = [pd.read_csv(f) for f in files_50]
    trial_matrix_50 = np.column_stack([df["# of objects collected"].values for df in dfs_50])
    std_50  = np.std(trial_matrix_50, axis=1)

    std_map = {
        "5 Obj":  std_5,
        "10 Obj": std_10,
        "30 Obj": std_30,
        "50 Obj": std_50,
    }

    # access all axes in the figure
    for ax in fig.axes:
        # change title font size
        ax.set_title("")
        ax.title.set_fontsize(30)

        # change x/y label font sizes
        ax.xaxis.label.set_fontsize(30)
        ax.yaxis.label.set_fontsize(30)

        # change tick size for right side only
        # if this axis has a right y-axis label, modify its ticks
        if ax.yaxis.get_label_position() == 'right':
            # ax.yaxis.set_major_locator(plt.MaxNLocator(5))  # at most 5 ticks
            ax.set_yticks([0, 10, 20, 30, 40, 50])
            for line, lbl in zip(ax.lines, new_labels):
                x = line.get_xdata()
                y = line.get_ydata()
                if lbl in std_map:
                    std = std_map[lbl]
                    print(lbl, len(x), len(y), len(std))  # debugging
                    ax.fill_between(x, y-std, y+std, color=line.get_color(), alpha=0.2) # add std deviation shading
        
        if ax.yaxis.get_label_position() == 'left':
            ax.set_yticks([0, 20000, 40000, 60000, 80000])

        # change tick label font sizes
        for tick in ax.xaxis.get_major_ticks():
            tick.label1.set_fontsize(30)
        for tick in ax.yaxis.get_major_ticks():
            tick.label1.set_fontsize(30)

        # ax.lines → list of Line2D objects (for line plots)
        # for i, line in enumerate(ax.lines):
        #     color = legend_colors[i % len(legend_colors)]
        #     line.set_color(color)

        # change legend font size (if it exists)
        legend = ax.get_legend()
        handles, labels = ax.get_legend_handles_labels()
        if legend:
            new_labels = ["5 Obj", "10 Obj", "30 Obj", "50 Obj"]
            ax.legend(handles, new_labels, loc='upper right', fontsize=24)

    # optionally show or save again
    # plt.show()
    fig.savefig("new_figure.png", dpi=300, bbox_inches="tight")

def main():
    edit_obj_capture()

if __name__ == "__main__":
    main()
