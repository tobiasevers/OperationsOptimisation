import math

#Define a function to calculate the distance between two geographic coordinates
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