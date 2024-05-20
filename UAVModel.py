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

delay = 1

### PARAMETERS
n = 5 # Number of targets
w = 10  # Number of UAVs
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
                time[i, j, v, k] = rd.randint(1, 30)
                # time[i, j, v, k] = 10

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


# # Add binary decision variables
# x1 = m.addVars(n+w+1, n+1, w+1, 3+1, vtype=GRB.BINARY, name="x1")
# x2 = m.addVars(n+w+1, n+w+2, w+1, vtype=GRB.BINARY, name="x2")

# Add continuous decision variables for task completion times
# t1 = m.addVars(n+1, 3+1, vtype=GRB.CONTINUOUS, name="t_1")
# t2 = m.addVars(w+1, vtype=GRB.CONTINUOUS, name="t_2")

for i in lst_i:
    for j in lst_j:
        if i == j:
            for v in lst_v:
                x1[i,j,v,2] = m.addVar(vtype=GRB.BINARY)
        elif i in lst_j:
            for v in lst_v:
                for k in lst_k:
                    x1[i,j,v,k] = m.addVar(vtype=GRB.BINARY)
        else:
            for k in [1,2,3]:
                x1[i,j,i-n,k] = m.addVar(vtype=GRB.BINARY)

for i in lst_i:
    if i in lst_j:
        for v in lst_v:
            x2[i,n+w+1,v] = m.addVar(vtype=GRB.BINARY)
    else:
        x2[i,n+w+1, i-n] = m.addVar(vtype=GRB.BINARY)

for v in lst_v:
    t2[v] = m.addVar(vtype=GRB.CONTINUOUS)

for j in lst_j:
    for k in lst_k:
        t1[j,k] = m.addVar(vtype=GRB.CONTINUOUS)

t = m.addVar(vtype=GRB.CONTINUOUS)

# Objective: Minimize total completion time of all tasks
#m.setObjective(quicksum(t1[j, 2] for j in range(n)), GRB.MINIMIZE)

#m.setObjective(quicksum(time[i, j, v, k] * x1[i, j, v, k] for k in lst_k for v in lst_v for i in lst_i for j in lst_j if (i,j,v,k) in x1), GRB.MAXIMIZE)

m.setObjective(0.1*quicksum(t1[j,k] for j in lst_j for k in lst_k) + t)


# Set objective
m.update()
# m.setObjective(m.getObjective(), GRB.MINIMIZE)


## Add constraints
# Equality constraints
''' Mission completion requires that all three tasks are performed on each target exactly one time. Similar to linear assignment problems'''
''' If more targets than vehicles, don't include C1.1 and C1.2'''
''' 3n constraints'''
for k in [1,3]:
    for j in lst_j:
        # C1.1 (4)
        m.addLConstr(quicksum(x1[i,j,v,k] for i in lst_i if i!=j for v in lst_v if (i,j,v,k) in x1), GRB.EQUAL, 1)
for k in [2]:
    for j in lst_j:
        # C1.2 (5)
        m.addLConstr(quicksum(x1[i,j,v,k] for i in lst_i for v in lst_v if (i,j,v,k) in x1), GRB.EQUAL, 1)

'''Not more than one AV is assigned to perform a specific task k on a specified target j'''
''' wnk constraints'''
#Inequality constraints
for k in [1,3]:
    for j in lst_j:
        # C2.1 (6) If more targets than vehicles, C2 is interesting
        m.addLConstr(quicksum(x1[i,j,v,k] for v in lst_v for i in lst_i if i != j and (i,j,v,k) in x1), GRB.LESS_EQUAL, 1)

for j in lst_j:
    # C2.2 (7)
    m.addLConstr(quicksum(x1[i,j,v,2] for v in lst_v for i in lst_i if (i,j,v,2) in x1), GRB.LESS_EQUAL, 1)

'''An AV v, coming from the outside, can visit target j at most once:'''
''' (n+1)w constraints'''
for v in lst_v:
    for j in lst_j:
        # C3.1 (8)
        m.addLConstr(quicksum(x1[i,j,v,k] for k in lst_k for i in lst_i if i!=j and (i,j,v,k) in x1), GRB.LESS_EQUAL, 1)
'''each AV v can only enter the sink once'''
for v in lst_v:
    # C3.2 (9)
    m.addLConstr(quicksum(x2[i,n+w+1,v] for i in lst_i if (i,n+w+1,v) in x2), GRB.LESS_EQUAL, 1)

''' AV v leaves node j at most once '''
''' nw constraints '''
for v in lst_v:
    for j in lst_j:
        # C4.1 (10)
        m.addLConstr(quicksum(x1[i,j,v,k] + x2[i, n+w+1, v] for v in lst_v for k in lst_k if (i,j,v,k) in x1 and (i,n+w+1,v) in x2), GRB.LESS_EQUAL, 1)

''' A munition is perishable. Thus, an AV v can be assigned to attack at most one target. '''
''' w constraints'''
for v in lst_v:
    # C5.1 (11)
    m.addLConstr(quicksum(x1[i,j,v,2] for j in lst_j for i in lst_i if (i,j,v,2) in x1),  GRB.LESS_EQUAL, 1)

''' If AV v is assigned to fly to target j for verification, it cannot possibly be assigned to attack target j:'''
''' wn constraints'''
for v in lst_v:
    for j in lst_j:
        # C6.1 (12)
        m.addLConstr(quicksum(x1[i,j,v,2] for i in lst_i if i!=j and (i,j,v,2) in x1), GRB.LESS_EQUAL, 1 - quicksum(x1[i,j,v,3] for i in lst_i if i!=j and (i,j,v,3) in x1))

# Continuity constraints
''' If AV v enters target (node) j for the purpose of performing task 3, it must also exit target j:'''
''' 2 * wn constraints'''
for j in lst_j:
    for v in lst_v:
        # C7.1 (13)
        m.addLConstr(quicksum(x1[i,j,v,3] for i in lst_i if i!=j and (i,j,v,3) in x1), GRB.LESS_EQUAL, quicksum(x1[j,i,v,k] for k in lst_k for i in lst_j if i!=j and (j,i,v,k) in x1 and (j,n+w+1,v) in x2) + x2[j,n+w+1,v])
        # C7.2 (14)
        m.addLConstr(quicksum(x1[i,j,v,1] for i in lst_i if i!=j and (i,j,v,1) in x1), GRB.LESS_EQUAL, quicksum(x1[j,i,v,k] for k in lst_k for i in lst_j if i!=j and (j,i,v,k) in x1 and (j,j,v,2) in x1 and (j,n+w+1,v) in x2) + x1[j,j,v,2] + x2[j,n+w+1,v])

''' Thus, if AV v is assigned to fly to target (node) j to perform task k 1⁄4 2, then, at any other point in time, AV v cannot also be assigned to fly from target j to a target i, iaj, to perform any other tastk at target i;'''
''' 2 * wn constraints'''
for j in lst_j:
    for v in lst_v:
        # C7.3 (15)
        m.addLConstr(quicksum(x1[j, i, v, k] for i in lst_j if i!=j for k in lst_k if (j, i, v, k) in x1 and (j,n+w+1,v) in x2) + x2[j,n+w+1,v], GRB.LESS_EQUAL,
                     1 - quicksum(x1[i, j, v, 2] for i in lst_i if (i, j, v, 2) in x1))
        ''' If AV v is not assigned to visit node j, then it cannot possibly be assigned to fly out of node j.'''
        # C7.4 (16)
        m.addLConstr(quicksum(x1[j,i,v,k] for i in lst_j if i!=j for k in lst_k if (j,i,v,k) in x1 and (j,n+w+1,v) in x2) + x2[j,n+w+1,v], GRB.LESS_EQUAL, quicksum(x1[i,j,v,k] for i in lst_i if i!=j for k in lst_k if (i,j,v,k) in x1))

''' All AVs leave the source nodes. An AV leaves the source node even if this entails a direct assignment to the sink.'''
''' w constraints'''
for v in lst_v:
    # C7.5 (17)
    m.addLConstr(quicksum(x1[n+v, j, v, k] for j in lst_j for k in lst_k if (n+v, j, v, k) in x1 and (n+v, n + w + 1, v) in x2) + x2[n+v, n + w + 1, v], GRB.EQUAL, 1)

''' An AV cannot attack target (node) i, coming from target (node) i, unless it entered target (node) i to perform a classification.'''
''' wn constraints'''
for i in lst_j:
    for v in lst_v:
        # C7.6 (18)
        m.addLConstr(x1[i, i, v, 2], GRB.LESS_EQUAL, quicksum(x1[j, i, v, 1] for j in lst_i if (j, i, v, 1) in x1))


# Timing constraints
for i in lst_j:
    for j in lst_j:
        if i != j:
            for v in lst_v:
                for k in [1, 3]:
                    # 8.1 (20) #TODO: test
                    m.addLConstr(t1[j, k] <= t1[i, 1] + time[i, j, v, k] + (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 1] for l in lst_i if l != i if (l, i, v, 1) in x1 and (i, j, v, k) in x1)) * w * T)
                    # 8.2 (21) #TODO: test
                    m.addLConstr(t1[j, k] >= t1[i, 1] + time[i, j, v, k] - (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 1] for l in lst_i if l != i if (l, i, v, 1) in x1 and (i, j, v, k) in x1)) * w * T)
                    # 8.3 (22) #TODO: test
                    m.addLConstr(t1[j, k] <= t1[i, 3] + time[i, j, v, k] + (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 3] for l in lst_i if l != i if (l, i, v, 3) in x1 and (i, j, v, k) in x1)) * w * T)
                    # 8.4 (23) #TODO: test
                    m.addLConstr(t1[j, k] >= t1[i, 3] + time[i, j, v, k] - (2 - x1[i, j, v, k] - quicksum(x1[l, i, v, 3] for l in lst_i if l != i if (l, i, v, 3) in x1 and (i, j, v, k) in x1)) * w * T)

# Timing constraints
for i in lst_j:
    for j in lst_j:
        if i != j:
            for v in lst_v:
                # 8.5 (24) #TODO: test
                m.addLConstr(t1[j, 2] <= t1[i, 1] + time[i, j, v, 2] + (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 1] for l in lst_i if l != i if (l, i, v, 1) in x1 and (i, j, v, 2) in x1)) * w * T)
                # 8.6 (25) #TODO: test
                m.addLConstr(t1[j, 2] >= t1[i, 1] + time[i, j, v, 2] - (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 1] for l in lst_i if l != i if (l, i, v, 1) in x1 and (i, j, v, 2) in x1)) * w * T)
                #8.7 (26) #TODO: test
                m.addLConstr(t1[j, 2] <= t1[i, 3] + time[i, j, v, 2] + (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 3] for l in lst_i if l != i if (l, i, v, 3) in x1 and (i, j, v, 2) in x1)) * w * T)
                # 8.8 (27) #TODO: test
                m.addLConstr(t1[j, 2] >= t1[i, 3] + time[i, j, v, 2] - (2 - x1[i, j, v, 2] - quicksum(x1[l, i, v, 3] for l in lst_i if l != i if (l, i, v, 3) in x1 and (i, j, v, 2) in x1)) * w * T)


for j in lst_j:
    for v in lst_v:
        for k in lst_k:
            m.addConstr(t1[j, k] <= t2[v] + time[n+v, j, v, k] + (1 - x1[n+v, j, v, k]) * T)
            m.addConstr(t1[j, k] >= t2[v] + time[n+v, j, v, k] - (1 - x1[n+v, j, v, k]) * T)
#
for j in lst_j:
    m.addConstr(t1[j, 1] + delay <= t1[j, 2])
    m.addConstr(t1[j, 2] + delay <= t1[j, 3])

for v in lst_v:
    m.addLConstr(quicksum(time[i,j,v,k] * x1[i,j,v,k] for k in lst_k for i in lst_i for j in lst_j if j!=i and (i,j,v,k) in x1), GRB.LESS_EQUAL, T)

m.update()
m.write('test2.lp')

# Optimize the model
m.optimize()

# Print solution
if m.status == GRB.OPTIMAL:
    print("Optimal solution found:")
    for i in lst_i:
        for j in lst_j:
            for v in lst_v:
                for k in lst_k:
                    if (i,j,v,k) in x1:
                        if x1[i, j, v, k].X > 0.5:
                            print(f"UAV {v} assigned from {i} to {j} for task {k}")
    for i in lst_i:
        for v in lst_v:
            if (i, n+w+1, v) in x2:
                if x2[i, n+w+1, v].X > 0.5:
                    print(f'UAV {v} flew to sinknode from node {i}')

    for j in lst_j:
        for k in lst_k:
            print(f"Task {k} on target {j} completed at time {t1[j, k].X}")
else:
    print("No optimal solution found.")


