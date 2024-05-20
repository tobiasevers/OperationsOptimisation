import pandas as pd
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import random as rd
from UAVModel import time, n, w, lst_v, lst_j, lst_k, lst_i

# Create a graph
G = nx.DiGraph()

# Add nodes
for node in lst_i:
    G.add_node(node, label=f'Node {node}')

# Add sink node
sink_node = n + w + 1
G.add_node(sink_node, label='Sink')

# Add edges with weights (time data)
for i in lst_i:
    for j in lst_j:
        if i != j:
            for v in lst_v:
                for k in lst_k:
                    G.add_edge(i, j, weight=time[i, j, v, k])

# Add edges from all nodes to the sink node
for i in lst_i:
    for v in lst_v:
        for k in lst_k:
            G.add_edge(i, sink_node, weight=time[i, lst_j[-1], v, k])  # Using the last target as a representative for the sink node edge weights

# Add self-loops for target nodes
for j in lst_j:
    for v in lst_v:
        G.add_edge(j, j, weight=time[j, j, v, 2])

# Set positions manually
pos = {}

# Sink node on the right
pos[sink_node] = (1, 0)

# Target nodes in the middle
for idx, node in enumerate(lst_j, start=1):
    pos[node] = (0.5, (idx - 1) / (n - 1) if n > 1 else 0)

# Source nodes on the left
for idx, node in enumerate(range(n + 1, n + w + 1), start=1):
    pos[node] = (0, (idx - 1) / (w - 1) if w > 1 else 0)

# Plot the graph
plt.figure(figsize=(10, 6))

# Draw nodes
nx.draw_networkx_nodes(G, pos, node_size=700)

# Draw edges
weights = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edges(G, pos, width=1.0, alpha=0.5)

# Draw labels
nx.draw_networkx_labels(G, pos, font_size=14)
edge_labels = {(i, j): f'{weights[i, j]}' for i, j in weights}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)

plt.title("UAV Mission Map")
plt.show()
