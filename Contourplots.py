import numpy as np
import matplotlib.pyplot as plt
from UAVModelClass import UAVStrikeModel  # Assuming your class is in uav_model.py
from gurobipy import *
from matplotlib.colors import LogNorm

def plot_heatmap(weighted_array, title):
    """
    Plot a heatmap from a weighted array with labels and a color bar.
    """
    # Replace NaN values with a small positive value
    weighted_array = np.nan_to_num(weighted_array, nan=1e-5)
    # Replace any non-positive values with a small positive value
    weighted_array[weighted_array <= 0] = 1e-5

    fig, ax = plt.subplots(figsize=(8, 6))
    # Use LogNorm for logarithmic color scale
    cax = ax.matshow(weighted_array, cmap='Blues', interpolation='nearest',
                     norm=LogNorm(vmin=weighted_array.min(), vmax=weighted_array.max()))
    fig.colorbar(cax, label='Trade-off Value')

    for i in range(weighted_array.shape[0]):
        for j in range(weighted_array.shape[1]):
            if not np.isnan(weighted_array[i, j]):
                ax.text(j, i, f'{weighted_array[i, j]:.2f}', ha='center', va='center', color='black')

    plt.title(title)
    plt.xlabel('Number of UAVs')
    plt.ylabel('Number of Targets')
    ax.set_xticks(range(weighted_array.shape[1]))
    ax.set_xticklabels(range(1, weighted_array.shape[1] + 1))
    ax.set_yticks(range(weighted_array.shape[0]))
    ax.set_yticklabels(range(1, weighted_array.shape[0] + 1))
    plt.show()


def generate_heatmap_data(min_targets, max_targets, min_uavs, max_uavs, endurance, delay=1):
    """
    Generate heatmap data by varying the number of targets and UAVs.
    """
    heatmap_data = np.zeros((max_targets - min_targets + 1, max_uavs - min_uavs + 1))

    for i, n_targets in enumerate(range(min_targets, max_targets + 1)):
        for j, n_uavs in enumerate(range(min_uavs, max_uavs + 1)):
            model = UAVStrikeModel(n_targets=n_targets, n_uavs=n_uavs, endurance=endurance, delay=delay)
            model.optimize()
            if model.m.status == GRB.OPTIMAL:
                heatmap_data[i, j] = model.elapsed_time
            else:
                heatmap_data[i, j] = np.nan  # Assign NaN if no optimal solution found

    return heatmap_data


if __name__ == "__main__":
    min_targets = 1
    max_targets = 6
    min_uavs = 1
    max_uavs = 10
    endurance = 100
    delay = 1

    heatmap_data = generate_heatmap_data(min_targets, max_targets, min_uavs, max_uavs, endurance, delay)
    plot_heatmap(heatmap_data, "Optimization Time Heatmap for Varying Number of UAVs and Targets")


