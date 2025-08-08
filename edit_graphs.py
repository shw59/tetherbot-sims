import pickle
import matplotlib.pyplot as plt

# load the pickled figure
with open("./data/figures/towing_agents_graph_2025-08-03.png.pkl", "rb") as f:
    fig = pickle.load(f)

legend_colors = ["red", "yellow", "skyblue", "blue", "lightgreen"]

# access all axes in the figure
for ax in fig.axes:
    # change title font size
    # ax.title.set_fontsize(35)
    ax.set_title("")
    ax.set_ylabel("Failed Agent X-Position")

    # change x/y label font sizes
    ax.xaxis.label.set_fontsize(30)
    ax.yaxis.label.set_fontsize(30)

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
    if legend:
        new_labels = ["I", "II", "III", "IV", "V"]
        ax.legend(new_labels)
        for text in ax.get_legend().get_texts():
            text.set_fontsize(30)

# optionally show or save again
# plt.show()
fig.savefig("new_figure.png", dpi=300, bbox_inches="tight")
