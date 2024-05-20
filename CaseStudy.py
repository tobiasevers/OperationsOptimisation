import matplotlib.pyplot as plt

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

def plot_locations(starting_locations, target_locations):
    plt.figure(figsize=(10, 8))

    # Plot starting locations
    for city, (lat, lon) in starting_locations.items():
        plt.scatter(lon, lat, c='blue', marker='o', label='Starting Location' if city == 'Manila' else "")
        plt.text(lon + 0.1, lat + 0.1, city, fontsize=9)

    # Plot target locations
    for city, (lat, lon) in target_locations.items():
        plt.scatter(lon, lat, c='red', marker='x', label='Target Location' if city == 'Legazpi' else "")
        plt.text(lon + 0.1, lat + 0.1, city, fontsize=9)

    # Labels and legend
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Drone Starting and Target Locations in the Philippines')
    plt.legend(loc='upper right')
    plt.grid(True)
    plt.show()


import math


# Define a function to calculate the distance between two geographic coordinates
def haversine(lat1, lon1, lat2, lon2):
    R = 6371  # Radius of the Earth in km
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    return distance


# Define a function to compute travel time
def compute_travel_time(distance, speed):
    return distance / speed


# Define a function to populate the time dictionary
def create_time_dictionary(starting_locations, target_locations, drone_speed):
    all_locations = {**starting_locations, **target_locations}
    n = len(target_locations)
    w = len(starting_locations)
    lst_i = range(1, n + w + 1)
    lst_j = range(1, n + 1)
    lst_v = range(1, w + 1)
    lst_k = range(1, 4)

    task_times = {
        1: 1,  # Classification takes 1 minute
        2: 2,  # Delivery takes 2 minutes
        3: 1  # Verification takes 1 minute
    }

    time = {}

    for i, loc_i in enumerate(all_locations.values(), 1):
        for j, loc_j in enumerate(target_locations.values(), 1):
            for v in lst_v:
                for k in lst_k:
                    distance = haversine(loc_i[0], loc_i[1], loc_j[0], loc_j[1])
                    travel_time = compute_travel_time(distance, drone_speed)
                    time[i, j, v, k] = travel_time * 60 + task_times[k]

    return time

# Plot the locations
plot_locations(starting_locations, target_locations)
time = create_time_dictionary(starting_locations, target_locations, drone_speed=100)

print(time)