import matplotlib.pyplot as plt
from UAVModelClass import UAVStrikeModel
import pickle
from matplotlib.patches import FancyArrowPatch

# Coordinates
starting_locations = {
    'Manila': (14.5995, 120.9842),
    'Cebu City': (10.3157, 123.8854),
    'Davao City': (7.1907, 125.4553),
    'Baguio': (16.4023, 120.5960),
    'Iloilo City': (10.7202, 122.5621),
    'Zamboanga City': (6.9214, 122.0790)
}

target_locations = {
    'Legazpi': (13.1339, 123.7332),
    'Cagayan de Oro': (8.4542, 124.6319),
    'Puerto Princesa': (9.7392, 118.7353)
}


def plot_locations(starting_locations, target_locations, x1, x2, center_location):
    plt.figure(figsize=(10, 8))

    # Plot starting locations
    for city, (lat, lon) in starting_locations.items():
        plt.scatter(lon, lat, c='blue', marker='o', label='Starting Location' if city == 'Manila' else "")
        plt.text(lon + 0.1, lat + 0.1, city, fontsize=9)

    # Plot target locations
    for city, (lat, lon) in target_locations.items():
        plt.scatter(lon, lat, c='red', marker='x', label='Target Location' if city == 'Legazpi' else "")
        plt.text(lon + 0.1, lat + 0.1, city, fontsize=9)

    # Combine all locations into a single list
    all_locations = list(target_locations.values()) + list(starting_locations.values())

    # Plot the drone paths based on the optimization results
    for (i, j, v, k), value in x1.items():
        if value > 0:
            start_lat, start_lon = all_locations[i - 1]
            end_lat, end_lon = all_locations[j - 1]
            plt.plot([start_lon, end_lon], [start_lat, end_lat], label=f'Drone {v} Task {k}')
            plt.text((start_lon + end_lon) / 2, (start_lat + end_lat) / 2, f'D{v}T{k}', fontsize=8, color='green')

            # Add round arrow if the task is 2 (delivery)
            if k == 2:
                arrow = FancyArrowPatch((end_lon, end_lat), (end_lon, end_lat), connectionstyle="arc3,rad=1",
                                        color="orange", arrowstyle='->', mutation_scale=15)
                plt.gca().add_patch(arrow)
                plt.text(end_lon + 0.2, end_lat, f'D{v}T{k}', fontsize=8, color='orange')

    # Labels and legend
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Drone Paths and Tasks in the Philippines')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()

import math

# Define a function to calculate the distance between two geographic coordinates
def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the Earth in km
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance

# Define a function to compute travel time in minutes
def compute_travel_time(distance, speed):
    return (distance / speed) * 60  # Travel time in minutes

# Define a function to populate the time dictionary
def create_time_dictionary(starting_locations, target_locations, drone_speed):
    # Combine target and starting locations into a list
    all_locations = list(target_locations.values()) + list(starting_locations.values())
    n = len(target_locations)
    w = len(starting_locations)
    lst_i = range(1, n + w + 1)
    lst_j = range(1, n + w + 1)
    lst_v = range(1, w + 1)
    lst_k = range(1, 4)

    # Task times in minutes
    task_times = {
        1: 1,  # Classification takes 1 minute
        2: 2,  # Delivery takes 2 minutes
        3: 1  # Verification takes 1 minute
    }

    time = {}

    for i in lst_i:
        for j in lst_j:
            for v in lst_v:
                for k in lst_k:
                    if i == j and k == 2:  # Task 2 (delivery) at the same location
                        total_time = task_times[k]
                    elif i != j:  # Traveling between different locations
                        loc_i = all_locations[i - 1]
                        loc_j = all_locations[j - 1]
                        distance = haversine(loc_i[0], loc_i[1], loc_j[0], loc_j[1])
                        travel_time = compute_travel_time(distance, drone_speed)
                        total_time = travel_time + task_times[k]
                    else:
                        continue  # Skip other tasks at the same location

                    time[i, j, v, k] = total_time
                    # print(f'Time from node {i} to target {j} by drone {v} for task {k}: {total_time:.2f} minutes')

    return time

# Define the locations
starting_locations = {
    'Manila': (14.5995, 120.9842),
    'Cebu City': (10.3157, 123.8854),
    'Davao City': (7.1907, 125.4553),
    'Baguio': (16.4023, 120.5960),
    'Iloilo City': (10.7202, 122.5621),
    'Zamboanga City': (6.9214, 122.0790)
}

target_locations = {
    'Legazpi': (13.1339, 123.7332),
    'Cagayan de Oro': (8.4542, 124.6319),
    'Puerto Princesa': (9.7392, 118.7353)
}

drone_speed = 100  # in km/h

# Create the time dictionary
time_dictionary = create_time_dictionary(starting_locations, target_locations, drone_speed)

# # Print the time dictionary
# for key, value in time_dictionary.items():
#     print(f'Time from node {key[0]} to target {key[1]} by drone {key[2]} for task {key[3]}: {value:.2f} minutes')

model = UAVStrikeModel(3, 6, 500, 1, time_dictionary)
model.optimize()
print(f'TIME ELAPSED: {model.elapsed_time} s')
model.print_solution()
model.save()


# Load the optimization results
with open('Results/3_6', 'rb') as f:
    optimization_results = pickle.load(f)

center_location = (14.5995, 120.9842)  # Geographic center of the Philippines

# Define x1 and x2 dictionaries
x1 = optimization_results.get('x1', {})
x2 = optimization_results.get('x2', {})
print('X2:', x2)

# Plot the locations and paths
plot_locations(starting_locations, target_locations, x1, x2, center_location)
