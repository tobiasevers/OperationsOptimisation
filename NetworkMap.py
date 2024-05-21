import networkx as nx
import matplotlib.pyplot as plt
from gurobipy import *
# Ensure the UAVModel import is the MILP code above. This is for reference.
from UAVModelClass import UAVStrikeModel

model = UAVStrikeModel(3, 4, 100, delay=1, obj=2)
model.optimize()
model.print_solution()
model.save('Visualveri')

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
    plt.figure(figsize=(10, 6))

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
    edge_labels = {(i, j, key): f'{data["weight"]}' for i, j, key, data in G.edges(keys=True, data=True)}
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

# NetworkMap(model)