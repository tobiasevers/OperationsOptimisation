from f_sensitivity import *

# Example usage
if __name__ == "__main__":
    # Parameters
    max_targets = 2
    max_drones = 5
    max_endurance = 150
    delay = 5
    num_matrices = 10

    # Combined sensitivity analysis
    # combined_results = combined_sensitivity_analysis(max_targets=max_targets, max_drones=max_drones, endurance=max_endurance, delay=delay, num_matrices=num_matrices)

    # Sensitivity analysis for endurance
    endurance_results = sensitivity_analysis_endurance(n_targets=max_targets, max_drones=max_drones, max_endurance=max_endurance, delay=delay, num_matrices=num_matrices)

    # Construct the filename with parameters
    # csv_file_combined = f'Sensitivity_data/sensitivity_analysis_results_max_targets_{max_targets}_max_drones_{max_drones}_endurance_{max_endurance}_delay_{delay}_num_matrices_{num_matrices}.csv'
    csv_file_endurance = f'Sensitivity_data/sensitivity_analysis_endurance_max_targets_{max_targets}_max_drones_{max_drones}_max_endurance_{max_endurance}_delay_{delay}_num_matrices_{num_matrices}.csv'

    # # Save results to CSV
    # # save_results_to_csv(combined_results, csv_file_combined)
    save_results_to_csv(endurance_results, csv_file_endurance)
    #
    # # Heatmap for Targets vs Drones
    # plot_heatmap_targets_drones(csv_file_combined)
    #
    # # Boxplot for Time Matrix
    # plot_boxplot_time_matrix(csv_file_combined)
    #
    # # Line Plot for Delays
    # plot_line_delay(csv_file_combined)
    #
    # # Scatter Plot for Elapsed Time vs Objective Value
    # plot_scatter_elapsed_time(csv_file_combined)

    # Heatmap for Endurance vs Number of Drones
    plot_heatmap_endurance_drones(csv_file_endurance)