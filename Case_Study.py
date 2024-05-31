from UAVModelClass import UAVStrikeModel
import pickle
from f_visualisation import plot_time_space_network, NetworkMap, plot_locations
from f_helper import create_time_dictionary
from coordinates import starting_locations, target_locations, center_location
from gurobipy import *
from f_sensitivity import plot_speed, plot_endurance, plot_heatmap_speed_endurance

# Define the speed of the drone in kilometers per hour
drone_speed = 89  # in km/h
endurance = 360 # in seconds

# Create the time dictionary using the starting and target locations with the given drone speed
time_dictionary = create_time_dictionary(starting_locations, target_locations, drone_speed)

# Create an instance of the UAVStrikeModel with the following parameters:
# - Drone speed: 89 km/h
# - Number of drones: 3
# - Number of targets: 6
# - Maximum flight endurance: 360 minutes (6 hours)
# - Objective type: 1 (minimizing time, cost, or another objective as defined in the model)
# - Time dictionary: precomputed dictionary of travel times between locations
# - Objective weight: obj=3 (custom parameter as per the model's definition)
model = UAVStrikeModel(3, 6, endurance, 1, time_dictionary, obj=3)

# Optimize the model to find the best solution
model.optimize()

# Print the time elapsed during optimization
print(f'TIME ELAPSED: {model.elapsed_time} s')

# Print the solution details found by the optimizer
model.print_solution()

# Save the model's results to a file named 'Phillipines' for later use
model.save('Phillipines')

# Load the optimization results from the saved file
with open('Results/Phillipines', 'rb') as f:
    optimization_results = pickle.load(f)

# Extract the decision variables x1 and x2 from the optimization results
x1 = optimization_results.get('x1', {})
x2 = optimization_results.get('x2', {})
print('X2:', x2)

# Plot the locations and paths using the decision variables
#plot_locations(starting_locations, target_locations, x1, x2, center_location)

# Plot the time-space network based on the optimization results
#plot_time_space_network(optimization_results)

# Create a network map visualization for the given model
#NetworkMap(model)


# Plot contour maps for different speeds and endurance levels
plot_heatmap_speed_endurance(60, 150, 200, 400, starting_locations, target_locations, n_targets=3, n_UAVS=6, delay=1)

# Perform sensitivity analysis for different speeds (120 km/h to 180 km/h)
plot_speed(80, 110, starting_locations, target_locations, endurance=endurance)


# Perform sensitivity analysis for different endurance levels (300 km to 650 km)
plot_endurance(300, 600, starting_locations, target_locations, drone_speed=drone_speed)