"""
edit_graphs.py

This file contains code to load and edit pickled matplotlib figures.
"""

import pickle
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import glob
import numpy as np

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

        ax.grid(False)

        # change tick label font sizes
        for tick in ax.xaxis.get_major_ticks():
            tick.label1.set_fontsize(30)
        for tick in ax.yaxis.get_major_ticks():
            tick.label1.set_fontsize(30)

        ax.axvline(500, linestyle='solid', color='black')

        # ax.lines → list of Line2D objects (for line plots)
        for i, line in enumerate(ax.lines[:-1]):
            color = legend_colors[i % len(legend_colors)]
            line.set_color(color)

        # change legend font size (if it exists)
        legend = ax.get_legend()
        handles, labels = ax.get_legend_handles_labels()
        if legend:
            # labels for the towing agents legend
            new_labels = ["I", "II", "III", "IV", "V"]
            ax.legend(handles, new_labels, loc='upper left')
            for text in ax.get_legend().get_texts():
                text.set_fontsize(25)

    # optionally show or save again
    # plt.show()
    fig.savefig("new_figure.svg", dpi=300, bbox_inches="tight")

def edit_obj_capture():
    """
    Polishes up the graphs produced from the object capture simulations. Edit anything below as needed.
    """
    # load the pickled figure
    with open("./data/figures/object_capture_maintain_line_False_offset0_graph_2025-09-14.png.pkl", "rb") as f:
        fig = pickle.load(f)

    new_labels = ["25 Obj", "50 Obj", "75 Obj", "100 Obj"]

    # legend_colors = ["yellow", "red", "purple", "blue"]

    # load files and calculate the std deviations 
    # files_25 = glob.glob("./data/object_capture_maintain_line_True_trial*_objects25_offset0.csv")
    files_25 = glob.glob("./data/object_capture_maintain_line_False_trial*_objects25_offset0.csv")
    dfs_25 = [pd.read_csv(f) for f in files_25]
    trial_matrix_25 = np.column_stack([df["variance"].values for df in dfs_25])
    std_25  = np.std(trial_matrix_25, axis=1)

    # files_50 = glob.glob("./data/object_capture_maintain_line_True_trial*_objects50_offset0.csv")
    files_50 = glob.glob("./data/object_capture_maintain_line_False_trial*_objects50_offset0.csv")
    dfs_50 = [pd.read_csv(f) for f in files_50]
    trial_matrix_50 = np.column_stack([df["variance"].values for df in dfs_50])
    std_50  = np.std(trial_matrix_50, axis=1)

    # files_75 = glob.glob("./data/object_capture_maintain_line_True_trial*_objects75_offset0.csv")
    files_75 = glob.glob("./data/object_capture_maintain_line_False_trial*_objects75_offset0.csv")
    dfs_75 = [pd.read_csv(f) for f in files_75]
    trial_matrix_75 = np.column_stack([df["variance"].values for df in dfs_75])
    std_75  = np.std(trial_matrix_75, axis=1)

    # files_100 = glob.glob("./data/object_capture_maintain_line_True_trial*_objects100_offset0.csv")
    files_100 = glob.glob("./data/object_capture_maintain_line_False_trial*_objects100_offset0.csv")
    dfs_100 = [pd.read_csv(f) for f in files_100]
    trial_matrix_100 = np.column_stack([df["variance"].values for df in dfs_100])
    std_100  = np.std(trial_matrix_100, axis=1)

    std_map = {
        "25 Obj":  std_25,
        "50 Obj": std_50,
        "75 Obj": std_75,
        "100 Obj": std_100,
    }

    # access all axes in the figure
    for ax in fig.axes:
        # change title font size
        ax.set_title("")
        ax.title.set_fontsize(30)

        # change x/y label font sizes
        ax.xaxis.label.set_fontsize(30)
        ax.yaxis.label.set_fontsize(30)

        # ax.set_xlim(0, 15000)
        ax.set_xticks([0, 3000, 6000, 9000, 12000, 15000])
        
        # if ax.yaxis.get_label_position() == 'left':
        ax.set_ylim(100, 2500)

        # std deviation shading
        # for line, lbl in zip(ax.lines, new_labels):
        #     x = line.get_xdata()
        #     y = line.get_ydata()

        #     if lbl in std_map:
        #         std = std_map[lbl]
        #         ax.fill_between(x, y-std, y+std, color=line.get_color(), alpha=0.2) # add std deviation shading

        # change tick label font sizes
        for tick in ax.xaxis.get_major_ticks():
            tick.label1.set_fontsize(30)
        for tick in ax.yaxis.get_major_ticks():
            tick.label1.set_fontsize(30)

        # change legend font size (if it exists)
        legend = ax.get_legend()
        handles, labels = ax.get_legend_handles_labels()
        if legend:
            new_labels = ["25 Obj", "50 Obj", "75 Obj", "100 Obj"]
            ax.legend(handles, new_labels, loc='upper right', fontsize=24)

    # optionally show or save again
    # plt.show()
    fig.savefig("new_figure.svg", dpi=300, bbox_inches="tight")
    
def edit_w_to_m_graph():
    """
    Polishes up the graphs produced from the W-to-M simulations. Edit anything below as needed.
    """
    #load the pickled figure
    with open("./data/figures/w_to_m_graph_2025-09-06.png.pkl", "rb") as f:
        fig = pickle.load(f)

    # load files and calculate the std deviations 
    files_agent1 = glob.glob("./data/w_to_m_trial*_agent1.csv")
    dfs_agent1 = [pd.read_csv(f) for f in files_agent1]
    trial_matrix_agent1 = np.column_stack([df["agent error"].values for df in dfs_agent1])
    std_agent1  = np.std(trial_matrix_agent1, axis=1)

    files_agent2 = glob.glob("./data/w_to_m_trial*_agent2.csv")
    dfs_agent2 = [pd.read_csv(f) for f in files_agent2]
    trial_matrix_agent2 = np.column_stack([df["agent error"].values for df in dfs_agent2])
    std_agent2  = np.std(trial_matrix_agent2, axis=1)

    files_agent3 = glob.glob("./data/w_to_m_trial*_agent3.csv")
    dfs_agent3 = [pd.read_csv(f) for f in files_agent3]
    trial_matrix_agent3 = np.column_stack([df["agent error"].values for df in dfs_agent3])
    std_agent3  = np.std(trial_matrix_agent3, axis=1)

    std_map = {
        "Agent 1":  std_agent1,
        "Agent 2": std_agent2,
        "Agent 3": std_agent3,
    }

    new_labels = ["Agent 1", "Agent 2", "Agent 3"]

    # access all axes in the figure
    for ax in fig.axes:
        # change title font size
        ax.set_title("")
        ax.title.set_fontsize(30)
        ax.set_xlabel("Time Step (Normalized)")

        # change x/y label font sizes
        ax.xaxis.label.set_fontsize(30)
        ax.yaxis.label.set_fontsize(30)

        ax.set_xlim(0, 1)
        # ax.set_ylim(0, 210)

        for line, lbl in zip(ax.lines, new_labels):
            line.set_label(lbl)

            x = line.get_xdata()
            y = line.get_ydata()

            x_norm = x / 1500

            if lbl in std_map:
                std = std_map[lbl]
                print(lbl, len(x), len(y), len(std))  # debugging
                ax.fill_between(x_norm, y-std, y+std, color=line.get_color(), alpha=0.2) # add std deviation shading

            line.set_xdata(x_norm)
        
        # change tick label font sizes
        for tick in ax.xaxis.get_major_ticks():
            tick.label1.set_fontsize(30)
        for tick in ax.yaxis.get_major_ticks():
            tick.label1.set_fontsize(30)

        # change legend font size (if it exists)
        legend = ax.get_legend()
        handles, labels = ax.get_legend_handles_labels()
        if legend:
            ax.legend(handles, new_labels, loc='upper right', fontsize=24)
    
    # graph hardware experimental data on top of the simulated data
    # time = [0,0.03571428571,0.07142857143,0.1071428571,0.1428571429,0.1785714286,0.2142857143,0.25,0.2857142857,0.3214285714,0.3571428571,0.3928571429,0.4285714286,0.4642857143,0.5,0.5357142857,0.5714285714,0.6071428571,0.6428571429,0.6785714286,0.7142857143,0.75,0.7857142857,0.8214285714,0.8571428571,0.8928571429,0.9285714286,0.9642857143,1]
    # Agent2 = [171.5714286,166.2857143,161.5714286,150.8571429,140.8571429,131.1428571,121.7142857,115.5714286,109,106.5714286,98.42857143,94.57142857,84.42857143,82,75,65,52.85714286,42.28571429,31.14285714,30,28.14285714,26.14285714,25.85714286,23.57142857,23.85714286,24.14285714,20.57142857,20.42857143,20.85714286]
    # Agent3 = [177.7142857,174.5714286,169.4285714,174,152.8571429,144.5714286,136,125,122.8571429,120.7142857,115,109.2857143,105,102,95.57142857,81.28571429,69.57142857,56.42857143,44.14285714,37.14285714,28.42857143,23,22.57142857,20,21,23.85714286,19,19.14285714,20.85714286]
    # Agent4 = [176,172.7142857,164.7142857,155.8571429,148.1428571,141.2857143,124.5714286,111.1428571,105.2857143,101,93.57142857,84.42857143,81,74.85714286,67.28571429,59.14285714,52.57142857,48.14285714,44.42857143,38.28571429,30.85714286,28,27,21.57142857,22.42857143,20.71428571,16,14.28571429,12.28571429]
    # std_2_low = [160.9749332,145.8022715,130.076153,108.0574989,94.59546479,80.29527912,65.95673231,62.38353163,55.75841225,56.31589109,51.29098154,52.66734308,40.92049883,40.18293809,37.81577395,33.17757604,30.85389634,25.52550063,13.7414341,12.5835327,14.05469933,14.01261159,14.11791274,10.70212216,11.50901788,10.67154724,5.286826921,2.476253632,4.512637446]
    # std_2_high = [182.1679239,186.7691571,193.0667041,193.6567868,187.1188209,181.9904352,177.4718391,168.7593255,162.2415878,156.8269661,145.5661613,136.4755141,127.936644,123.8170619,112.184226,96.82242396,74.86038937,59.04592794,48.54428018,47.4164673,42.23101495,38.27310269,37.59637298,36.44073499,36.20526784,37.61416704,35.85603022,38.38088923,37.20164827]
    # std_3_low = [167.7524538,161.8322875,152.0039036,154.1589651,104.7689303,87.79744056,65.82165576,56.42400809,53.5139659,54.30717192,51.68860029,49.4366979,47.47609193,41.42497764,35.56349259,23.84671973,18.10669261,11.08596033,2.805107763,-5.940119276,-0.9418056139,-5.172090208,-3.085650652,-3.187640386,-0.2994522621,7.563702751,12.26699671,6.435528923,3.599984232]
    # std_3_high = [187.6761176,187.3105696,186.8532392,193.8410349,200.9453555,201.3454166,206.1783442,193.5759919,192.2003198,187.1213995,178.3113997,169.1347307,162.5239081,162.5750224,155.5793646,138.7247088,121.0361645,101.7711825,85.48060652,80.22583356,57.79894847,51.17209021,48.2285078,43.18764039,42.29945226,40.15058296,25.73300329,31.85018536,38.11430148]
    # std_4_low = [163.8619057,160.6766463,151.9508707,142.7241238,129.0865496,119.7317952,93.32052382,70.01529716,67.14250045,55.32725101,42.82087146,29.88850315,23.06612965,14.75326453,7.715827358,5.098342862,1.278618345,3.910785176,4.308513507,2.042488318,5.18336524,5.736427361,5.755392841,9.741281448,4.485539886,4.634129354,4.789885519,3.914049589,2.874474805]
    # std_4_high = [188.1380943,184.7519251,177.4777007,168.9901619,167.1991647,162.8396334,155.8223333,152.2704171,143.4289281,146.672749,144.3219857,138.9686397,138.9338703,134.9610212,126.8556012,113.1873714,103.8642388,92.37492911,84.54862935,74.52894025,56.53092047,50.26357264,48.24460716,33.40157569,40.37160297,36.79444207,27.21011448,24.65737898,21.69695377]

    # for ax in fig.axes:
    #     ax.plot(time, Agent2, label="Agent 1 (Hardware)", color="blue")
    #     ax.fill_between(time, std_2_low, std_2_high, color="blue", alpha=0.2)

    #     ax.plot(time, Agent3, label="Agent 2 (Hardware)", color="yellow")
    #     ax.fill_between(time, std_3_low, std_3_high, color="green", alpha=0.2)

    #     ax.plot(time, Agent4, label="Agent 3 (Hardware)", color="black")
    #     ax.fill_between(time, std_4_low, std_4_high, color="red", alpha=0.2)

    #     # Styling
    #     ax.set_xlim(0, 1)
    #     ax.set_ylim(0, 210)
    #     ax.tick_params(axis='both', labelsize=30)

    #     legend = ax.get_legend()
    #     handles, labels = ax.get_legend_handles_labels()
    #     ax.legend(handles, labels, loc='upper right', fontsize=24)

    # optionally show or save again
    # plt.show()
    fig.savefig("new_figure.svg", dpi=300, bbox_inches="tight")

def make_empty_graph(rows=1, cols=1):
    """
    Create an empty graph with subplots arranged horizontally 
    for object capture snapshots.
    """
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5, rows * 3))

    if not isinstance(axes, np.ndarray):
        axes = np.array([axes])
    axes = axes.ravel()

    # Flatten axes to make them easy to iterate
    # axes = axes.ravel()

    labels = ["100 Objects"]

    for ax, lbl in zip(axes, labels):
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(False)         # no gridlines
        ax.set_xlim(0, 1)      # placeholder
        ax.set_ylim(0, 1)

        ax.set_xlabel(lbl, fontsize=12)

    plt.tight_layout()
    
    fig.savefig("new_figure.svg", dpi=300, bbox_inches="tight")
    return fig, axes

def main():
    # edit_towing_agents()
    edit_obj_capture()
    # edit_w_to_m_graph()
    # make_empty_graph()

if __name__ == "__main__":
    main()
