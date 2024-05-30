from f_sensitivity import plot_heatmap, generate_heatmap_data

if __name__ == "__main__":
    min_targets = 1
    max_targets = 3
    min_uavs = 1
    max_uavs = 4
    endurance = 100
    delay = 1

    heatmap_data = generate_heatmap_data(min_targets, max_targets, min_uavs, max_uavs, endurance, delay)
    plot_heatmap(heatmap_data, "Optimization Time Heatmap for Varying Number of UAVs and Targets")
    