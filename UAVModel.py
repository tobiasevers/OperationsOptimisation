import pandas as pd
from gurobipy import *
import numpy as np
import xlsxwriter as xlsx
import networkx as nx
import matplotlib.pyplot as plt
import random as rd

#--------------------------------------------------#
### DATA (self-made)
#tijvk, time vehicle v to fly from i to j to perform k at node j
test233
#--------------------------------------------------#
####### MODEL            #######

m = Model('UAVstrike')

## Information
# i = start node index
# j = arrival node index
# J = Cost function
# k = Taks index, 1 = classification, 2 = attack, 3 = verification
# n = number of targets
# tijvk = Time required for air vehicle v to fly from node i to node j to perform task k at node j
# tjk = Time of completion of task k on target j
T = 100 #Maximum endurance of any UAV
# Tij = Flight time between nodes i and j
# Tv = Endurance of UAV v
# v = air vehicle index
# w = Number of UAVs
# xijvk = The binary decision variable xijvk = 1 if AV v is assigned to fly from node i to node j and perform task k at node , 0 otherwise
# xin+w+1v = The binary decision variable xijv = 1 if AV v is assigned to fly from node i to the sink node n+w+1 , 0 otherwise


### PARAMETERS
n = 2 # Number of targets
w = 3 # Number of UAVs
lst_i = range(1, n+w+1)
lst_j = range(1, n+1)
lst_v = range(1, w+1)
lst_k = range(1, 4)

### DATA ###
time = {}
for i in lst_i:
    for j in lst_j:
        for v in lst_v:
            for k in lst_k:
                time[i, j, v, k] = rd.randint(0, 30)

## Decision variables
# Discrete DVs
x1 = {} # xijvk, total = wn(3n+1)
x2 = {} # xin+w+1v, total = (n+1)w
# Continuous DVs
t1 = {} # tjk, total = 3n
t2 = {} # tv, total = w


## Add variables and objective function to the model


# # Binary decision variables
# for i in lst_i:
#     for v in lst_v:
#         for j in lst_j:
#             for k in lst_k:
#                 # TODO: add rules at top right of page 518 paper
#                 x1[i, j, v, k] = m.addVar(obj = time[i, j, v, k], lb=0,vtype=GRB.BINARY)
#         x2[i, n+w+1, v] = m.addVar(obj = 5, lb=0, #TODO: other objective for this DV
#                            vtype=GRB.BINARY)

# !!!! These commented out DVs are not in the objective function now

# # Continuous decision variables
# for j in lst_j:
#     for k in lst_k:
#         t1[j, k] = m.addVar(obj = quicksum(time[i, j, v, k] * x1[i, j, v, k] for k in lst_k for v in lst_v for i in lst_i for j in lst_j), lb=0,
#                     vtype=GRB.CONTINUOUS)
# for v in lst_v:
#     t2[v] = m.addVar(obj = quicksum(time[i, j, v, k] * x1[i, j, v, k] for k in lst_k for v in lst_v for i in lst_i for j in lst_j), lb=0,
#                     vtype=GRB.CONTINUOUS)


# Add binary decision variables
x1 = m.addVars(n+w+1, n+1, w+1, 3+1, vtype=GRB.BINARY, name="x1")
x2 = m.addVars(n+w+1, n+w+2, w+1, vtype=GRB.BINARY, name="x2")

# Add continuous decision variables for task completion times
t1 = m.addVars(n+1, 3+1, vtype=GRB.CONTINUOUS, name="t_1")
t2 = m.addVars(w+1, vtype=GRB.CONTINUOUS, name="t_2")

# Objective: Minimize total completion time of all tasks
#m.setObjective(quicksum(t1[j, 2] for j in range(n)), GRB.MINIMIZE)

m.setObjective(quicksum(time[i, j, v, k] * x1[i, j, v, k] for k in lst_k for v in lst_v for i in lst_i for j in lst_j), GRB.MAXIMIZE)



# Set objective
m.update()
# m.setObjective(m.getObjective(), GRB.MINIMIZE)


## Add constraints
# Equality constraints
for v in lst_v:
    for i in lst_i:
        # C1.1 (4) If more targets than vehicles, don't include C1
        m.addLConstr(quicksum(x1[i,j,v,k] for k in [1,3] for j in lst_j if j != i), GRB.EQUAL, 1)
        # C1.2 (5)
        m.addLConstr(quicksum(x1[i,j,v,k] for k in [2] for j in lst_j), GRB.EQUAL, 1)

#Inequality constraints
#for i in lst_i:
    #C2.1 (6) If more targets than vehicles, C2 is interesting
    #m.addLConstr(quicksum(x1[i,j,v,k] for k in [1,3] for v in lst_v for j in lst_j if j != i), GRB.LESS_EQUAL, 1)
    # C2.2 (7)
    # m.addLConstr(quicksum(x1[i,j,v,2] for v in lst_v for j in lst_j), GRB.LESS_EQUAL, 1)

# for k in lst_k:
#     for i in lst_i:
#         # C3.1 (8)
#         m.addLConstr(quicksum(x1[i,j,v,k] for j in lst_j if i != j for v in lst_v), GRB.LESS_EQUAL, 1)

# for i in lst_i: #need other objective for this
#     # C3.2 (9)
#     m.addLConstr(quicksum(x2[i,n+w+1, v] for v in lst_v), GRB.LESS_EQUAL, 1)

# for k in lst_k:
#     for i in lst_j: # important that range for i is different in paper here!
#         # C4.1 (10)
#         m.addLConstr(quicksum(x1[i,j,v,k] + x2[j, n+w+1, v] for v in lst_v for j in lst_j if j != i), GRB.LESS_EQUAL, 1)

# for j in lst_j:
#     for i in lst_i:
#         # C5.1 (11)
#         m.addLConstr(quicksum(x1[i,j,v,2] for v in lst_v),  GRB.LESS_EQUAL, 1)

# for i in lst_i:
#     # C6.1 (12)
#     m.addLConstr(quicksum(x1[i,j,v,2] for v in lst_v for j in lst_j if j != i), GRB.LESS_EQUAL, 1 - quicksum(x1[i,j,v,3] for v in lst_v for j in lst_j if j != i))

# Continuity constraints

for k in lst_k:
    # C7.1 (13)
    m.addLConstr(quicksum(x1[i,j,v,3] for i in lst_i for j in lst_j if j != i for v in lst_v), GRB.LESS_EQUAL, quicksum(x1[i,j,v,k] + x2[j,n+w+1,v] for i in lst_j for j in lst_j if j != i for v in lst_v))
    # C7.2 (14)
    m.addLConstr(quicksum(x1[i,j,v,1] for i in lst_i for j in lst_j if j != i for v in lst_v), GRB.LESS_EQUAL, quicksum([x1[j,i,v,k] + x1[j,j,v,2] + x2[j,n+w+1,v] for i in lst_j for v in lst_v for j in lst_j if j != i ]))

# for k in lst_k:
#     for i in lst_j:
#         # C7.3 (15)
#         m.addLConstr(quicksum(x1[j, i, v, k] + x2[j,n+w+1,v] for v in lst_v for j in lst_j if j != i), GRB.LESS_EQUAL,
#                      1 - quicksum(x1[i, j, v, 2] for v in lst_v for j in lst_j if j != i))
#         # C7.4 (16)
#         m.addLConstr(quicksum(x1[j,i,v,k] + x2[j,n+w+1,v] for j in lst_j if j != i for v in lst_v), GRB.LESS_EQUAL, quicksum(x1[i,j,v,k] for j in lst_j if j != i for v in lst_v))
#
        # C7.5 (17)
        # m.addLConstr(quicksum(x1[n+v, j, v, k] + x2[n+v, n + w + 1, v] for j in lst_j if j != i for v in lst_v),
        #              GRB.EQUAL, 1)

for j in lst_i:
    # C7.6 (18)
    m.addLConstr(quicksum(x1[i,i,v,2] for i in lst_j for v in lst_v), GRB.LESS_EQUAL, quicksum(x1[j,i,v,1] for i in lst_j for v in lst_v))

# Timing constraints
for i in lst_j:
    for j in lst_j:
        if i != j:
            for v in lst_v:
                for k in [1, 3]:
                    m.addConstr(t1[j, k] <= t1[i, 0] + time[i, j, v, k] + (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 0] for l in lst_i if l != i)) * T)
                    m.addConstr(t1[j, k] >= t1[i, 0] + time[i, j, v, k] - (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 0] for l in lst_i if l != i)) * T)

# for i in lst_j:
#     for j in lst_j:
#         if i != j:
#             for v in lst_j:
#                 for k in [1, 3]:
#                     m.addConstr(t1[j, k] <= t1[i, 2] + time[i, j, v, k] + (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 2] for l in lst_i if l != i)) * T)
#                     m.addConstr(t1[j, k] >= t1[i, 2] + time[i, j, v, k] - (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 2] for l in lst_i if l != i)) * T)

for j in lst_j:
    for v in lst_v:
        for k in lst_k:
            m.addConstr(t1[j, k] <= t2[v] + time[n+v, j, v, 1] + (1 - x1[n+v, j, v, 1]) * T)
            m.addConstr(t1[j, k] >= t2[v] + time[n+v, j, v, 1] - (1 - x1[n+v, j, v, 1]) * T)
#
for j in lst_j:
    m.addConstr(t1[j, 1] <= t1[j, 2])
    m.addConstr(t1[j, 2] <= t1[j, 3])
#
# # Additional constraints for task sequence
# for j in lst_j:
#     for i in lst_j:
#         if i != j:
#             for v in lst_v:
#                 m.addConstr(t1[j, 1] <= t1[i, 0] + time[i, j, v, 1] + (2 - x1[i, j, v, 1] - quicksum(x1[l, i, v, 0] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 1] >= t1[i, 0] + time[i, j, v, 1] - (2 - x1[i, j, v, 1] - quicksum(x1[l, i, v, 0] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 1] <= t1[i, 2] + time[i, j, v, 1] + (2 - x1[i, j, v, 1] - quicksum(x1[l, i, v, 2] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 1] >= t1[i, 2] + time[i, j, v, 1] - (2 - x1[i, j, v, 1] - quicksum(x1[l, i, v, 2] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 2] <= t1[i, 0] + time[i, j, v, 2] + (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 0] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 2] >= t1[i, 0] + time[i, j, v, 2] - (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 0] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 2] <= t1[i, 2] + time[i, j, v, 2] + (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 2] for l in range(n+w) if l != i)) * T)
#                 m.addConstr(t1[j, 2] >= t1[i, 2] + time[i, j, v, 2] - (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 2] for l in range(n+w) if l != i)) * T)

# Optimize the model
m.optimize()

# Print solution
if m.status == GRB.OPTIMAL:
    print("Optimal solution found:")
    for i in range(n+w):
        for j in range(n):
            for v in range(w):
                for k in range(3):
                    if x1[i, j, v, k].X > 0.5:
                        print(f"UAV {v} assigned from {i} to {j} for task {k+1}")
    for j in range(n):
        for k in range(3):
            print(f"Task {k+1} on target {j+1} completed at time {t1[j, k].X}")
else:
    print("No optimal solution found.")

