import matplotlib.pyplot as plt
import pickle
test
def plot_time_space_network(results):
    model = results['Model']
    decision_variables = results
    x1 = decision_variables.get('x1', {})
    x2 = decision_variables.get('x2', {})
    t1 = decision_variables.get('t1', {})
    t2 = decision_variables.get('t2', {})
    print(t1)




    n = model['n']  # Number of target nodes
    w = model['w']  # Number of UAVs (source nodes)
    T = model['T']  # Maximum time/endurance for visualization
    delay = model['delay']  # Delay parameter for visualization

    time_space= {}

    for v in range(1, w+1):
        time_space[v] = {t2[v]: v+n - 1}


    # Define nodes: target nodes, UAV starting nodes, and sink node
    nodes = range(1, n + w + 1 + 1)

    # Create the plot
    fig, ax = plt.subplots(figsize=(14, 8))

    # Plot nodes on the y-axis
    node_positions = {node: idx for idx, node in enumerate(nodes)}
    for node, pos in node_positions.items():
        ax.plot(range(T), [pos] * T, 'k--', alpha=0.5)

        # # ax.plot([start_time, end_time], [node_positions[i], node_positions[j]], label=f'UAV {v} Task {k}',
        # marker = 'o')

   # Plot UAV movements for x1
    for (i, j, v, k), value in x1.items():
        if value > 0:
            end_time = t1[(j, k)]
            time_space[v][end_time] = j - 1

    for v, timespace in time_space.items():
        sorted_time = sorted(timespace.keys())
        sorted_place = [timespace[key] for key in sorted_time]
        ax.plot(sorted_time, sorted_place, label=f'UAV {v} ', marker = 'o')


    #
    # # Plot UAV movements for x2 (source to sink)
    # for (i, v, k), value in x2.items():
    #     if value > 0:
    #         start_time = t1.get((k, i), 0)
    #         end_time = t2.get(k, 0)
    #         ax.plot([start_time, end_time], [node_positions[i], node_positions[n + w + 1]],
    #                 label=f'UAV {v} to Sink Task {k}', marker='x')
    #
    # # Plot task completion times
    # for (k, j), completion_time in t1.items():
    #     ax.plot(completion_time, node_positions[j], 'rx', label=f'Task {k} Completion at Node {j}')

    # Customize plot
    ax.set_yticks(range(len(nodes)))
    ax.set_yticklabels(nodes)
    ax.set_ylabel('Nodes')
    ax.set_xlabel('Time')
    ax.set_title('Time-Space Network with UAV Movements and Task Completions')
    ax.legend()

    plt.show()