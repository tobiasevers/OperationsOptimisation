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
test2
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
# T = Maximum endurance of any UAV
# Tij = Flight time between nodes i and j
# Tv = Endurance of UAV v
# v = air vehicle index
# w = Number of UAVs
# xijvk = The binary decision variable xijvk = 1 if AV v is assigned to fly from node i to node j and perform task k at node , 0 otherwise
# xin+w+1v = The binary decision variable xijv = 1 if AV v is assigned to fly from node i to the sink node n+w+1 , 0 otherwise


### PARAMETERS
n = 5 # Number of targets
w = 5 # Number of UAVs
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


# Binary decision variables
for i in lst_i:
    for v in lst_v:
        for j in lst_j:
            for k in lst_k:
                # TODO: add rules at top right of page 518 paper
                x1[i, j, v, k] = m.addVar(obj = time[i, j, v, k], lb=0,vtype=GRB.INTEGER)
        #x2[i, n+w+1, v] = m.addVar(obj = other_objective, lb=0,
                           #vtype=GRB.INTEGER)

# !!!! These commented out DVs are not in the objective function now

# # Continuous decision variables
# for j in lst_j:
#     for k in lst_k:
#         t1[j, k] = m.addVar(obj = quicksum(time[i, j, v, k] * x1[i, j, v, k] for k in lst_k for v in lst_v for i in lst_i for j in lst_j), lb=0,
#                     vtype=GRB.CONTINUOUS)
# for v in lst_v:
#     t2[v] = m.addVar(obj = quicksum(time[i, j, v, k] * x1[i, j, v, k] for k in lst_k for v in lst_v for i in lst_i for j in lst_j), lb=0,
#                     vtype=GRB.CONTINUOUS)


# Set objective
m.update()
m.setObjective(m.getObjective(), GRB.MINIMIZE)

## Add constraints
# Equality constraints
for v in lst_v:
    for i in lst_i:
        # C1.1 (4) If more targets than vehicles, don't include C1
        m.addLConstr(quicksum(x1[i,j,v,k] for k in [1,3] for j in lst_j if j != i), GRB.EQUAL, 1)
        # C1.2 (5)
        m.addLConstr(quicksum(x1[i,j,v,k] for k in [2] for j in lst_j), GRB.EQUAL, 1)

# Inequality constraints
for i in lst_i:
    # C2.1 (6) If more targets than vehicles, C2 is interesting
    m.addLConstr(quicksum(x1[i,j,v,k] for k in [1,3] for v in lst_v for j in lst_j if j != i), GRB.LESS_EQUAL, 1)
    # C2.2 (7)
    m.addLConstr(quicksum(x1[i,j,v,k] for k in [2] for v in lst_v for j in lst_j), GRB.LESS_EQUAL, 1)

# for k in lst_k:
#     for i in lst_i:
#         # C3.1 (8)
#         m.addLConstr()
#
# for i in lst_i:
#     # C3.2 (9)
#     m.addLConstr()
#
# for k in lst_k:
#     for i in lst_j: # important that range for i is different in paper here!
#         # C4.1 (10)
#         m.addLConstr()
#
# for j in lst_j:
#     for i in lst_i:
#         # C5.1 (11)
#         m.addLConstr()
#
# for i in lst_i:
#     # C6.1 (12)
#     m.addLConstr()
#
# # Continuity constraints
# for i in lst_i:
#     # C7.1 (13)
#     m.addLConstr()
#     # C7.2 (14)
#     m.addLConstr()
#
# for k in lst_k:
#     for i in lst_j:
#         # C7.3 (15)
#         m.addLConstr()
#         # C7.4 (16)
#         m.addLConstr()
#         # C7.5 (17)
#         m.addLConstr()
#
# # C7.6 (18)
# m.addLConstr()
#
# # Timing constraints
# # C8.1 (19)
# m.addLConstr()
# # C8.2 (20)
# m.addLConstr()
# # C8.3 (21)
# m.addLConstr()
# # C8.4 (22)
# m.addLConstr()
# # C8.5 (23)
# m.addLConstr()
# # C8.6 (24)
# m.addLConstr()
# # C8.7 (25)
# m.addLConstr()
# # C8.8 (26)
# m.addLConstr()
# # C8.9 (27)
# m.addLConstr()
# # C8.10 (28)
# m.addLConstr()
# # C8.11 (29)
# m.addLConstr()
#
# # C8.12 (30)
# m.addLConstr()
# # C8.13 (31)
# m.addLConstr()
#
# # Extensions
# # C9.1 (32)
# m.addLConstr()
# # C10.1 (33)
# m.addLConstr()


## Run model ###
m.update()
m.write('test.lp')
# Set time constraint for optimization (5minutes)
m.setParam('TimeLimit', 1)
# m.setParam('MIPgap', 0.009)
m.optimize()
# m.write("testout.sol")
status = m.status

if status == GRB.Status.UNBOUNDED:
    print('The model cannot be solved because it is unbounded')

elif status == GRB.Status.OPTIMAL or True:
    f_objective = m.objVal
    print('***** RESULTS ******')
    print('\nObjective Function Value: \t %g' % f_objective)

elif status != GRB.Status.INF_OR_UNBD and status != GRB.Status.INFEASIBLE:
    print('Optimization was stopped with status %d' % status)




