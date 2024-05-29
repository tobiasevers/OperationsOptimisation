from UAVModelClass import UAVStrikeModel
import pickle
from f_visualisation import plot_time_space_network, NetworkMap, plot_locations
from f_helper import create_time_dictionary
from coordinates import starting_locations, target_locations, center_location
from gurobipy import *
from f_sensitivity import plot_speed, plot_endurance

drone_speed = 100  # in km/h

# Create the time dictionary
time_dictionary = create_time_dictionary(starting_locations, target_locations, drone_speed)

model = UAVStrikeModel(3, 6, 500, 1, time_dictionary, obj=3)
model.optimize()
print(f'TIME ELAPSED: {model.elapsed_time} s')
model.print_solution()
model.save('Phillipines')

# Load the optimization results
with open('Results/Phillipines', 'rb') as f:
    optimization_results = pickle.load(f)

# Define x1 and x2 dictionaries
x1 = optimization_results.get('x1', {})
x2 = optimization_results.get('x2', {})
print('X2:', x2)

# Plot the locations and paths
plot_locations(starting_locations, target_locations, x1, x2, center_location)
plot_time_space_network(optimization_results)
NetworkMap(model)

#Perfrom the sensitivity analysis for speed and endurance
plot_speed(120, 180, starting_locations, target_locations)
plot_endurance(300, 650, starting_locations, target_locations)