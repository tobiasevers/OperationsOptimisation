from f_sensitivity import plot_heatmap, generate_heatmap_data
import pickle

if __name__ == "__main__":
    min_targets = 1
    max_targets = 5
    min_uavs = 1
    max_uavs = 10
    endurance = 100
    delay = 1

    heatmap_data = generate_heatmap_data(min_targets, max_targets, min_uavs, max_uavs, endurance, delay)
    with open('contour.pickle', 'wb') as handle:
        pickle.dump(heatmap_data, handle, protocol=pickle.HIGHEST_PROTOCOL)

    for i in range(len(heatmap_data)):
        for j in range(len(heatmap_data[0])):
            if i == j:
                heatmap_data[i, j] = 'nan'

    plot_heatmap(heatmap_data, "Optimization Time Heatmap for Varying Number of UAVs and Targets")

