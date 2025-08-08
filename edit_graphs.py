import pickle
import matplotlib.pyplot as plt

# load the pickled figure
with open("./data/figures/towing_agents_graph_2025-08-03.png.pkl", "rb") as f:
# with open("./data/figures/object_capture_maintain_line_False_offset0_graph_2025-08-03.png.pkl", "rb") as f:
    fig = pickle.load(f)

# change graph colors of towing agents
legend_colors = ["red", "yellow", "deepskyblue", "blue", "lime"]

# access all axes in the figure
for ax in fig.axes:
    # change title font size
    # ax.set_title("Failed Agent X-Position Over Time")
    # ax.set_title("Collective Radius + # of Objects Captured Over Time, No Offset")
    ax.title.set_fontsize(30)
    ax.set_title("")

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
        # legend.remove()
        # new_labels = ["5 Obj", "10 Obj", "30 Obj", "50 Obj"]
        # ax.legend(handles, new_labels, loc='center right', fontsize=30)
        for text in ax.get_legend().get_texts():
            text.set_fontsize(30)

# optionally show or save again
# plt.show()
fig.savefig("new_figure.png", dpi=300, bbox_inches="tight")
