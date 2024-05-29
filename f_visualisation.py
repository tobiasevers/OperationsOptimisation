import matplotlib.pyplot as plt
import math
import matplotlib.pyplot as plt
from gurobipy import *
import networkx as nx
from matplotlib.patches import FancyArrowPatch

def plot_time_space_network(results):
    model = results['Model']
    decision_variables = results
    x1 = decision_variables.get('x1', {})
    x2 = decision_variables.get('x2', {})
    t1 = decision_variables.get('t1', {})
    t2 = decision_variables.get('t2', {})
    n = model['n']  # Number of target nodes
    w = model['w']  # Number of UAVs (source nodes)
    t_max = math.ceil(max(t1.values())) + 2

    time_space = {}

    for v in range(1, w + 1):
        time_space[v] = {t2[v]: (v + n - 1, None)}

    # Define nodes: target nodes, UAV starting nodes, and sink node
    nodes = range(1, n + w + 1)

    # Create the plot
    fig, ax = plt.subplots(figsize=(6, 6))

    # Plot nodes on the y-axis
    node_positions = {node: idx for idx, node in enumerate(nodes)}
    for node, pos in node_positions.items():
        ax.plot(range(t_max), [pos] * t_max, 'k--', alpha=0.5)

    # Define the colors in the order specified: orange, green, red, purple
    colors = ['orange', 'green', 'red', 'purple']

    # Plot UAV movements for x1
    for (i, j, v, k), value in x1.items():
        if value > 0:
            end_time = t1[(j, k)]
            time_space[v][end_time] = (j - 1, k)

    for idx, (v, timespace) in enumerate(time_space.items()):
        sorted_time = sorted(timespace.keys())
        sorted_place = [timespace[key][0] for key in sorted_time]
        sorted_tasks = [timespace[key][1] for key in sorted_time]
        color = colors[idx % len(colors)]  # Cycle through the colors list
        ax.plot(sorted_time, sorted_place, label=f'UAV {v}', marker='o', color=color)

        for idx in range(len(sorted_time) - 1):
            start_time = sorted_time[idx]
            end_time = sorted_time[idx + 1]
            start_place = sorted_place[idx]
            end_place = sorted_place[idx + 1]
            task_k = sorted_tasks[idx + 1]
            if task_k is not None:
                ax.text((start_time + end_time) / 2, (start_place + end_place) / 2, f'{task_k}',
                        horizontalalignment='center', verticalalignment='bottom', color=color)

    for (j, k), value in t1.items():
        if k == 2:
            ax.scatter(value, j - 1, marker='x', c='r', s=200)

    # Customize plot
    ax.set_yticks(range(len(nodes)))
    ax.set_yticklabels(nodes)
    ax.set_ylabel('Nodes')
    ax.set_xlabel('Time')
    ax.set_xlim(0, t_max)
    ax.set_title('Time-Space Network with UAV Movements and Task Completions')
    ax.legend()

    plt.show()


    import networkx as nx


# Run the MILP model (code provided above)
# Assuming m is the model and x1, x2 are the decision variables as defined earlier

def NetworkMap(model):
    time = model.time
    n = model.n
    w = model.w
    lst_v = model.lst_v
    lst_i = model.lst_i
    lst_j = model.lst_j
    lst_k = model.lst_k
    x1 = model.x1
    x2 = model.x2

    # Create a MultiDiGraph
    G = nx.MultiDiGraph()

    # Add nodes
    for node in lst_i:
        G.add_node(node, label=f'Node {node}')

    # Add sink node
    sink_node = n + w + 1
    G.add_node(sink_node, label='Sink')

    # Initialize a colormap
    cmap = plt.get_cmap('tab10')  # Use 'tab10' colormap which has 10 distinct colors

    # Only add edges that are active in the MILP solution and store their UAV index
    edges = []
    for i in lst_i:
        for j in lst_j:
            if i != j:
                for v in lst_v:
                    for k in lst_k:
                        if (i, j, v, k) in x1 and x1[i, j, v, k].X > 0.5:
                            G.add_edge(i, j, weight=time[i, j, v, k], UAV=v, task=k)

    # Add edges from nodes to the sink node if active
    for i in lst_i:
        for v in lst_v:
            if (i, sink_node, v) in x2 and x2[i, sink_node, v].X > 0.5:
                G.add_edge(i, sink_node, weight=0, UAV=v)  # Using last task and target for weight

    # Add self-loops for target nodes if active
    for j in lst_j:
        for v in lst_v:
            if (j, j, v, 2) in x1 and x1[j, j, v, 2].X > 0.5:
                G.add_edge(j, j, weight=time[j, j, v, 2], UAV=v)

    # Set positions manually
    pos = {}

    # Sink node on the right
    pos[sink_node] = (1, 0.5)

    # Target nodes in the middle
    for idx, node in enumerate(lst_j, start=1):
        pos[node] = (0.5, (idx - 1) / (n - 1) if n > 1 else 0)

    # Source nodes on the left
    for idx, node in enumerate(range(n + 1, n + w + 1), start=1):
        pos[node] = (0, (idx - 1) / (w - 1) if w > 1 else 0)

    # Plot the graph
    plt.figure(figsize=(6, 6))

    # Draw nodes with white color and black edges
    nx.draw_networkx_nodes(G, pos, node_size=700, node_color='white', edgecolors='black')

    # Draw labels
    nx.draw_networkx_labels(G, pos, font_size=16)

    # Draw arcs for edges with colors corresponding to each UAV
    ax = plt.gca()
    for (i, j, key, data) in G.edges(keys=True, data=True):
        UAV = data['UAV']
        rad = 0.1 * (key + 1)  # Offset the radius for each multiple edge
        arrow = nx.draw_networkx_edges(G, pos, edgelist=[(i, j)], width=2.0, alpha=0.7, edge_color=[cmap(UAV % 10)], connectionstyle=f"arc3,rad={rad}")
        ax.add_patch(arrow[0])

    # Manually add edge labels for multi-edges with slight offsets to avoid overlap
    edge_labels = {(i, j, key): f'{round(data["weight"])}' for i, j, key, data in G.edges(keys=True, data=True)}
    for (i, j, key), label in edge_labels.items():
        UAV = G.edges[i, j, key]['UAV']
        color = cmap(UAV % 10)
        offset = 0.05 * key  # Slight offset for each label based on the edge key
        if i == j:
            # Self-loop label position
            x, y = pos[i]
            pos_label = (x, y + 0.1 + offset)  # Slightly above the node with additional offset
            plt.text(pos_label[0], pos_label[1], label, fontsize=14, color=color, horizontalalignment='center')
        else:
            x, y = pos[i]
            dx, dy = pos[j][0] - x, pos[j][1] - y
            pos_label = (x + dx / 3 + offset, y + dy / 3 + offset)
            plt.text(pos_label[0], pos_label[1], label, fontsize=14, color=color, horizontalalignment='center')

    plt.title("UAV Mission Map with Active Links")
    plt.show()

def plot_locations(starting_locations, target_locations, x1, x2, center_location):
    plt.figure(figsize=(5, 7))  # Adjusted figure size for better visibility

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
            if k==1:
                plt.plot([start_lon, end_lon], [start_lat, end_lat], label=f'Drone {v} Task {k} + 2')
            elif k==3:
                plt.plot([start_lon, end_lon], [start_lat, end_lat], label=f'Drone {v} Task {k}')
            
    # Labels and legend
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.title('Drone Paths and Tasks in the Philippines')

    # Combine unique labels for starting and target locations
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(), loc='center left', bbox_to_anchor=(1, 0.5))

    plt.grid(True)
    plt.show()