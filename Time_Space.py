import matplotlib.pyplot as plt
import pickle


def plot_time_space_network(results):
    model = results['Model']
    decision_variables = results
    x1 = decision_variables.get('x1', {})
    x2 = decision_variables.get('x2', {})
    t1 = decision_variables.get('t1', {})
    t2 = decision_variables.get('t2', {})

    print([x for x in x2.values() if x == 1.0])

    n = model['n']  # Number of target nodes
    w = model['w']  # Number of UAVs (source nodes)
    T = model['T']  # Maximum time/endurance for visualization
    t_max = 20
    delay = model['delay']  # Delay parameter for visualization
    time = model['time']

    time_space = {}

    for v in range(1, w + 1):
        time_space[v] = {t2[v]: (v + n - 1, None)}

    # Define nodes: target nodes, UAV starting nodes, and sink node
    nodes = range(1, n + w + 1 + 1)

    # Create the plot
    fig, ax = plt.subplots(figsize=(14, 8))

    # Plot nodes on the y-axis
    node_positions = {node: idx for idx, node in enumerate(nodes)}
    for node, pos in node_positions.items():
        ax.plot(range(t_max), [pos] * t_max, 'k--', alpha=0.5)

    # Plot UAV movements for x1
    for (i, j, v, k), value in x1.items():
        if value > 0:
            end_time = t1[(j, k)]
            time_space[v][end_time] = (j - 1, k)

    for v, timespace in time_space.items():
        sorted_time = sorted(timespace.keys())
        sorted_place = [timespace[key][0] for key in sorted_time]
        sorted_tasks = [timespace[key][1] for key in sorted_time]
        ax.plot(sorted_time, sorted_place, label=f'UAV {v}', marker='o')

        print('time', sorted_time)
        print('task', sorted_tasks)

        for idx in range(len(sorted_time) - 1):
            start_time = sorted_time[idx]
            end_time = sorted_time[idx + 1]
            start_place = sorted_place[idx]
            end_place = sorted_place[idx + 1]
            task_k = sorted_tasks[idx + 1]
            if task_k is not None:
                ax.text((start_time + end_time) / 2, (start_place + end_place) / 2, f'{task_k}',
                        horizontalalignment='center', verticalalignment='bottom')

    for (j, k), value in t1.items():
        if k==2:
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


with open('Results/3_6', 'rb') as f:
    results = pickle.load(f)

plot_time_space_network(results)