from gurobipy import Model, GRB, quicksum

# Define parameters (example values, should be replaced with actual problem data)
n = 3  # number of targets
w = 3  # number of UAVs
T = 100  # max endurance
t = {(v, k, i, j): 1 for v in range(1, w+1) for k in range(1, 4) for i in range(1, n+w+1) for j in range(1, n+1)}  # flight times

# Create a new model
m = Model("UAV_Optimization")

# Add binary decision variables
x = m.addVars(w, 3, n+w, n, vtype=GRB.BINARY, name="x")

# Add continuous decision variables for task completion times
t_jk = m.addVars(n, 3, vtype=GRB.CONTINUOUS, name="t_jk")
t_start = m.addVars(w, vtype=GRB.CONTINUOUS, name="t_start")

# Objective: Minimize total completion time of all tasks
m.setObjective(quicksum(t_jk[j, 2] for j in range(n)), GRB.MINIMIZE)

# Constraints
for j in range(n):
    for k in [0, 2]:  # for k = 1 (classification) and k = 3 (verification)
        m.addConstr(quicksum(x[v, k, i, j] for v in range(w) for i in range(n+w) if i != j) == 1)
    m.addConstr(quicksum(x[v, 1, i, j] for v in range(w) for i in range(n+w)) == 1)  # for k = 2 (attack)

for v in range(w):
    for j in range(n):
        m.addConstr(quicksum(x[v, k, i, j] for k in [0, 2] for i in range(n+w) if i != j) <= 1)
        m.addConstr(quicksum(x[v, 1, i, j] for i in range(n+w)) <= 1)

for j in range(n):
    m.addConstr(quicksum(x[v, 1, i, j] for v in range(w) for i in range(n+w)) == 1)

for v in range(w):
    for j in range(n):
        m.addConstr(quicksum(x[v, k, i, j] for k in [0, 2] for i in range(n+w) if i != j) ==
                    quicksum(x[v, k, j, l] for k in [0, 2] for l in range(n) if l != j) +
                    quicksum(x[v, 1, j, j]) +
                    quicksum(x[v, j, n+w+1]))

# Timing constraints
for i in range(n):
    for j in range(n):
        if i != j:
            for v in range(w):
                for k in [0, 2]:
                    m.addConstr(t_jk[j, k] <= t_jk[i, 0] + t[v, k, i, j] + (2 - x[v, k, i, j] - quicksum(x[v, 0, l, i] for l in range(n+w) if l != i)) * T)
                    m.addConstr(t_jk[j, k] >= t_jk[i, 0] + t[v, k, i, j] - (2 - x[v, k, i, j] - quicksum(x[v, 0, l, i] for l in range(n+w) if l != i)) * T)

# Additional constraints for continuity and timing
for v in range(w):
    for j in range(n):
        m.addConstr(quicksum(x[v, 2, i, j] for i in range(n+w) if i != j) <= 1 - quicksum(x[v, 2, i, j] for i in range(n+w) if i != j))
        m.addConstr(t_jk[j, 1] <= t_start[v] + t[v, 1, n+v, j] + (1 - x[v, 1, n+v, j]) * T)
        m.addConstr(t_jk[j, 1] >= t_start[v] + t[v, 1, n+v, j] - (1 - x[v, 1, n+v, j]) * T)

# Add end constraints to ensure tasks are completed in sequence
for j in range(n):
    m.addConstr(t_jk[j, 0] <= t_jk[j, 1])
    m.addConstr(t_jk[j, 1] <= t_jk[j, 2])

# Optimize the model
m.optimize()

# Print solution
if m.status == GRB.OPTIMAL:
    print("Optimal solution found:")
    for v in range(w):
        for k in range(3):
            for i in range(n+w):
                for j in range(n):
                    if x[v, k, i, j].X > 0.5:
                        print(f"UAV {v} assigned from {i} to {j} for task {k+1}")
    for j in range(n):
        for k in range(3):
            print(f"Task {k+1} on target {j+1} completed at time {t_jk[j, k].X}")
else:
    print("No optimal solution found.")

