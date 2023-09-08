import sys

import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":
    # print(f"Arguments count: {len(sys.argv)}")
    if len(sys.argv) != 9:
        print(
            "Invalid Arguments. Usage: visualization.py path/to/txt/file0 path/to/txt/file1 path/to/txt/file2 PlotName low high firstLabel secondLabel"
        )
        exit()
    originalFilePath = sys.argv[1]
    firstFilePath = sys.argv[2]
    secondFilePath = sys.argv[3]
    plotName = sys.argv[4]
    frameRangeLow = int(sys.argv[5])
    frameRangeHigh = int(sys.argv[6])
    firstLabel = sys.argv[7]
    secondLabel = sys.argv[8]

    originalData = []
    firstData = []
    secondData = []

    f = open(originalFilePath, "r")
    for line in f:
        originalData.append(float(line))
    f.close()

    f = open(firstFilePath, "r")
    for line in f:
        firstData.append(float(line))
    f.close()

    f = open(secondFilePath, "r")
    for line in f:
        secondData.append(float(line))
    f.close()

    fig = plt.figure(facecolor="black")
    ax = plt.axes()
    ax.set_facecolor("black")
    ax.set(xlabel="Frame", ylabel="Joint Rotation", title=plotName)

    ax.tick_params(axis="x", colors="white")
    ax.tick_params(axis="y", colors="white")

    ax.yaxis.label.set_color("white")
    ax.xaxis.label.set_color("white")
    ax.title.set_color("white")
    plt.rcParams["axes.edgecolor"] = "white"

    x = np.linspace(frameRangeLow, frameRangeHigh, frameRangeHigh - frameRangeLow)
    plt.plot(
        x,
        originalData[frameRangeLow:frameRangeHigh],
        color="blue",
        linestyle="solid",
        label="Original Input",
    )
    plt.plot(
        x,
        firstData[frameRangeLow:frameRangeHigh],
        color="yellow",
        linestyle="--",
        label=firstLabel,
    )
    plt.plot(
        x,
        secondData[frameRangeLow:frameRangeHigh],
        color="red",
        linestyle="-.",
        label=secondLabel,
    )
    plt.legend()
    # plt.show()
    figName = plotName + ".png"
    # plt.savefig(figName)
    plt.show()
    plt.close()
