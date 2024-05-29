import numpy as np
import matplotlib.pyplot as plt
from UAVModelClass import UAVStrikeModel  # Assuming your class is in uav_model.py
from gurobipy import *
from matplotlib.colors import LogNorm
from f_helper import create_time_dictionary
import pandas as pd
import random as rd
import seaborn as sns



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

def sensitivity_analysis_endurance(n_targets, max_drones, max_endurance, delay=1, num_matrices=5, max_time=30):
    results = []
    for n_drones in range(n_targets, max_drones + 1):
        for endurance in range(10, max_endurance + 1, 1):
            obj_vals = []
            elapsed_times = []
            for _ in range(num_matrices):
                time_matrix = create_random_time_matrix(n_targets, n_drones, max_time)
                model = UAVStrikeModel(n_targets=n_targets, n_uavs=n_drones, endurance=endurance, delay=delay)
                model.time = time_matrix
                model.optimize()
                if model.m.status == GRB.OPTIMAL:
                    obj_vals.append(model.m.objVal)
                    elapsed_times.append(model.elapsed_time)
                else:
                    obj_vals.append(None)
                    elapsed_times.append(None)
            avg_obj_val = sum(filter(None, obj_vals)) / len(obj_vals) if obj_vals else None
            avg_elapsed_time = sum(filter(None, elapsed_times)) / len(elapsed_times) if elapsed_times else None
            results.append({
                'n_drones': n_drones,
                'endurance': endurance,
                'objective_value': avg_obj_val,
                'elapsed_time': avg_elapsed_time
            })
            print(f"Drones: {n_drones}, Endurance: {endurance}, Average Objective: {avg_obj_val}, Average Time: {avg_elapsed_time}s")
    return results

def combined_sensitivity_analysis(max_targets, max_drones, endurance, delay, num_matrices=5, max_time=30):
    results = []
    for n_targets in range(1, max_targets + 1):
        for n_drones in range(n_targets + 1, max_drones + 1):
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

def plot_heatmap_endurance_drones(csv_file):
    df = pd.read_csv(csv_file)
    pivot_table = df.pivot_table(index="endurance", columns="n_drones", values="objective_value")
    plt.figure(figsize=(10, 8))
    sns.heatmap(pivot_table, annot=True, fmt=".2f", cmap="coolwarm", cbar_kws={'label': 'Objective Value'})
    plt.title("Heatmap of Objective Values for Endurance vs. Number of Drones")
    plt.xlabel("Number of Drones")
    plt.ylabel("Endurance")
    plt.show()


def sens_speed(drone_speed, obj, starting_locations, target_locations, n_targets=3, n_UAVS=6, endurance=240, delay=1):
    time_dictionary = create_time_dictionary(starting_locations, target_locations, drone_speed)
    model = UAVStrikeModel(n_targets, n_UAVS, endurance, delay, time_dictionary, obj=obj)
    model.optimize()

    if model.m.status == GRB.OPTIMAL:
        obj_val = model.m.objVal
    else:
        return np.nan
    
    return obj_val

def sens_endurance(endurance, obj, starting_locations, target_locations, n_targets=3, n_UAVS=6, drone_speed=500, delay=1):
    time_dictionary = create_time_dictionary(starting_locations, target_locations, drone_speed)
    model = UAVStrikeModel(n_targets, n_UAVS, endurance, delay, time_dictionary, obj=obj)
    model.optimize()

    if model.m.status == GRB.OPTIMAL:
        obj_val = model.m.objVal
    else:
        return np.nan

    return obj_val

def plot_speed(min_speed, max_speed, starting_locations, target_locations, n_targets=3, n_UAVS=6, endurance=240, delay=1):
    speed_range = range(min_speed, max_speed, 5)
    # Lists to store results
    lst_sens_speed_1 = []
    lst_sens_speed_2 = []

    # Speed sensitivity analysis
    for speed in speed_range:
        obj_val_1 = sens_speed(speed, 1, starting_locations, target_locations, n_targets, n_UAVS, endurance, delay)
        obj_val_2 = sens_speed(speed, 2, starting_locations, target_locations, n_targets, n_UAVS, endurance, delay)
        
        lst_sens_speed_1.append(obj_val_1)
        lst_sens_speed_2.append(obj_val_2)

    # Plotting speed sensitivity with secondary y-axis
    fig, ax1 = plt.subplots(figsize=(5, 4))

    color = 'tab:blue'
    ax1.set_xlabel('Drone Speed (km/h)')
    ax1.set_ylabel('Objective Value 1', color=color)
    ax1.plot(speed_range, lst_sens_speed_1, color=color, label='Summed flight time of UAVs (min)')
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color = 'tab:red'
    ax2.set_ylabel('Objective Value 2', color=color)  # we already handled the x-label with ax1
    ax2.plot(speed_range, lst_sens_speed_2, color=color, label='Mission time (min)')
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.title('Sensitivity Analysis of Objective Value to Drone Speed')
    plt.grid(True)
    plt.xlim(speed_range.start, speed_range.stop - speed_range.step)
    plt.show()

def plot_endurance(min_end, max_end, starting_locations, target_locations, n_targets=3, n_UAVS=6, drone_speed=100, delay=1):
    endurance_range = range(min_end, max_end, 5)

    lst_endurance_1 = []
    lst_endurance_2 = []

    # Endurance sensitivity analysis
    for endurance in endurance_range:
        obj_val_1 = sens_endurance(endurance, 1, starting_locations, target_locations, n_targets, n_UAVS, drone_speed, delay)
        obj_val_2 = sens_endurance(endurance, 2, starting_locations, target_locations, n_targets, n_UAVS, drone_speed, delay)
        lst_endurance_1.append(obj_val_1)
        lst_endurance_2.append(obj_val_2)

    # Plotting endurance sensitivity with secondary y-axis
    fig, ax1 = plt.subplots(figsize=(5, 4))

    color = 'tab:blue'
    ax1.set_xlabel('Endurance (minutes)')
    ax1.set_ylabel('Objective Value 1', color=color)
    ax1.plot(endurance_range, lst_endurance_1, color=color, label='Summed flight time of UAVs (min)')
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color = 'tab:red'
    ax2.set_ylabel('Objective Value 2', color=color)  # we already handled the x-label with ax1
    ax2.plot(endurance_range, lst_endurance_2, color=color, label='Mission time (min)')
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.title('Sensitivity Analysis of Objective Value to Endurance')
    plt.grid(True)
    plt.xlim(endurance_range.start, endurance_range.stop - endurance_range.step)
    plt.show()