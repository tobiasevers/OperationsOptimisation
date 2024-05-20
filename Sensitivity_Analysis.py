import pandas as pd
from UAVModelClass import UAVStrikeModel
import random as rd
import seaborn as sns
import matplotlib.pyplot as plt
from gurobipy import *

def create_random_time_matrix(n_targets, n_drones, max_time=30):
    time_matrix = {}
    lst_i = range(1, n_targets + n_drones + 1)
    lst_j = range(1, n_targets + 1)
    lst_v = range(1, n_drones + 1)
    lst_k = range(1, 4)
    for i in lst_i:
        for j in lst_j:
            for v in lst_v:
                for k in lst_k:
                    time_matrix[i, j, v, k] = rd.randint(1, max_time)
    return time_matrix

def sensitivity_analysis_targets_drones(max_targets, max_drones, endurance, delay=1):
    results = []
    for n_targets in range(1, max_targets + 1):
        for n_drones in range(1, max_drones + 1):
            model = UAVStrikeModel(n_targets=n_targets, n_uavs=n_drones, endurance=endurance, delay=delay)
            model.optimize()
            if model.m.status == GRB.OPTIMAL:
                obj_val = model.m.objVal
            else:
                obj_val = None
            results.append({
                'n_targets': n_targets,
                'n_drones': n_drones,
                'objective_value': obj_val,
                'elapsed_time': model.elapsed_time
            })
            print(f"Targets: {n_targets}, Drones: {n_drones}, Objective: {obj_val}, Time: {model.elapsed_time}s")
    return results

def sensitivity_analysis_time_matrix(n_targets, n_drones, endurance, delay=1, num_matrices=5, max_time=30):
    results = []
    for _ in range(num_matrices):
        time_matrix = create_random_time_matrix(n_targets, n_drones, max_time)
        model = UAVStrikeModel(n_targets=n_targets, n_uavs=n_drones, endurance=endurance, delay=delay)
        model.time = time_matrix
        model.optimize()
        if model.m.status == GRB.OPTIMAL:
            obj_val = model.m.objVal
        else:
            obj_val = None
        results.append({
            'time_matrix': time_matrix,
            'objective_value': obj_val,
            'elapsed_time': model.elapsed_time
        })
        print(f"Time Matrix Objective: {obj_val}, Time: {model.elapsed_time}s")
    return results

def sensitivity_analysis_delay(n_targets, n_drones, endurance, max_delay):
    results = []
    for delay in range(1, max_delay + 1):
        model = UAVStrikeModel(n_targets=n_targets, n_uavs=n_drones, endurance=endurance, delay=delay)
        model.optimize()
        if model.m.status == GRB.OPTIMAL:
            obj_val = model.m.objVal
        else:
            obj_val = None
        results.append({
            'delay': delay,
            'objective_value': obj_val,
            'elapsed_time': model.elapsed_time
        })
        print(f"Delay: {delay}, Objective: {obj_val}, Time: {model.elapsed_time}s")
    return results

def combined_sensitivity_analysis(max_targets, max_drones, endurance, delay, num_matrices=5, max_time=30):
    results = []
    for n_targets in range(1, max_targets + 1):
        for n_drones in range(n_targets, max_drones + 1):
            for _ in range(num_matrices):
                time_matrix = create_random_time_matrix(n_targets, n_drones, max_time)
                model = UAVStrikeModel(n_targets=n_targets, n_uavs=n_drones, endurance=endurance, delay=delay)
                model.time = time_matrix
                model.optimize()
                if model.m.status == GRB.OPTIMAL:
                    obj_val = model.m.objVal
                else:
                    obj_val = None
                results.append({
                    'n_targets': n_targets,
                    'n_drones': n_drones,
                    'delay': delay,
                    'time_matrix': time_matrix,
                    'objective_value': obj_val,
                    'elapsed_time': model.elapsed_time
                })
                print(f"Targets: {n_targets}, Drones: {n_drones}, Delay: {delay}, Objective: {obj_val}, Time: {model.elapsed_time}s")
    return results

def save_results_to_csv(results, filename='sensitivity_analysis_results.csv'):
    df = pd.DataFrame(results)
    df.to_csv(filename, index=False)

def plot_heatmap_targets_drones(csv_file):
    df = pd.read_csv(csv_file)
    pivot_table = df.pivot_table(index="n_targets", columns="n_drones", values="objective_value")
    plt.figure(figsize=(10, 8))
    sns.heatmap(pivot_table, annot=True, fmt=".2f", cmap="coolwarm", cbar_kws={'label': 'Objective Value'})
    plt.title("Heatmap of Objective Values for Targets vs. Drones")
    plt.xlabel("Number of Drones")
    plt.ylabel("Number of Targets")
    plt.show()

def plot_boxplot_time_matrix(csv_file):
    df = pd.read_csv(csv_file)
    plt.figure(figsize=(10, 8))
    sns.boxplot(x="n_targets", y="objective_value", data=df)
    plt.title("Boxplot of Objective Values for Different Time Matrices")
    plt.xlabel("Number of Targets")
    plt.ylabel("Objective Value")
    plt.show()

def plot_line_delay(csv_file):
    df = pd.read_csv(csv_file)
    plt.figure(figsize=(10, 8))
    sns.lineplot(x="delay", y="objective_value", data=df, marker='o')
    plt.title("Line Plot of Objective Values for Different Delays")
    plt.xlabel("Delay Between Tasks")
    plt.ylabel("Objective Value")
    plt.show()

def plot_scatter_elapsed_time(csv_file):
    df = pd.read_csv(csv_file)
    plt.figure(figsize=(10, 8))
    sns.scatterplot(x="elapsed_time", y="objective_value", data=df)
    plt.title("Scatter Plot of Elapsed Time vs. Objective Value")
    plt.xlabel("Elapsed Time (s)")
    plt.ylabel("Objective Value")
    plt.show()

# Example usage
if __name__ == "__main__":
    # # Sensitivity analysis for number of targets and drones
    # results_targets_drones = sensitivity_analysis_targets_drones(max_targets=2, max_drones=5, endurance=100)
    #
    # # Sensitivity analysis for time matrix
    # results_time_matrix = sensitivity_analysis_time_matrix(n_targets=2, n_drones=5, endurance=100, num_matrices=100)
    #
    # # Sensitivity analysis for delay
    # results_delay = sensitivity_analysis_delay(n_targets=2, n_drones=5, endurance=100, max_delay=5)

    # Combined sensitivity analysis
    combined_results = combined_sensitivity_analysis(max_targets=3, max_drones=10, endurance=100, delay=1, num_matrices=25)

    # Save results to CSV
    save_results_to_csv(combined_results)

    csv_file = 'sensitivity_analysis_results.csv'

    # Heatmap for Targets vs Drones
    plot_heatmap_targets_drones(csv_file)

    # Boxplot for Time Matrix
    plot_boxplot_time_matrix(csv_file)

    # Line Plot for Delays
    plot_line_delay(csv_file)

    # Scatter Plot for Elapsed Time vs Objective Value
    plot_scatter_elapsed_time(csv_file)
