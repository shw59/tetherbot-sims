import pickle
import matplotlib.pyplot as plt

# load the pickled figure
with open("./data/figures/towing_agents_graph_2025-08-03.png.pkl", "rb") as f:
    fig = pickle.load(f)

# access all axes in the figure
for ax in fig.axes:
    # change title font size
    ax.title.set_fontsize(35)

    # change x/y label font sizes
    ax.xaxis.label.set_fontsize(30)
    ax.yaxis.label.set_fontsize(30)

    # change tick label font sizes
    for tick in ax.xaxis.get_major_ticks():
        tick.label1.set_fontsize(20)
    for tick in ax.yaxis.get_major_ticks():
        tick.label1.set_fontsize(20)

    # change legend font size (if it exists)
    legend = ax.get_legend()
    if legend:
        for text in legend.get_texts():
            text.set_fontsize(30)

# optionally show or save again
plt.show()
# fig.savefig("new_figure.png")
