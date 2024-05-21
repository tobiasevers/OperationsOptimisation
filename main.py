import matplotlib.pyplot as plt
from UAVModelClass import UAVStrikeModel
import pickle
from matplotlib.patches import FancyArrowPatch
from Time_Space import plot_time_space_network
from CaseStudy import starting_locations, target_locations, plot_locations, create_time_dictionary
from NetworkMap import NetworkMap

drone_speed = 100  # in km/h

# Create the time dictionary
time_dictionary = create_time_dictionary(starting_locations, target_locations, drone_speed)

model = UAVStrikeModel(3, 6, 500, 1, time_dictionary, obj=3)
model.optimize()
print(f'TIME ELAPSED: {model.elapsed_time} s')
model.print_solution()
model.save('1')

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
plot_time_space_network(optimization_results)
NetworkMap(model)

model.sensitivity_analysis()