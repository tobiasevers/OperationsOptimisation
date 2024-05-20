import pandas as pd
from gurobipy import *
import numpy as np
import xlsxwriter as xlsx
import networkx as nx
import matplotlib.pyplot as plt
import random as rd
import time as tm
import pickle

class UAVStrikeModel:
    def __init__(self, n_targets, n_uavs, endurance, delay=1):
        self.n = n_targets
        self.w = n_uavs
        self.filename = f'Results/{self.n}_{self.w}'
        self.T = endurance
        self.delay = delay
        self.lst_i = range(1, self.n + self.w + 1)
        self.lst_j = range(1, self.n + 1)
        self.lst_v = range(1, self.w + 1)
        self.lst_k = range(1, 4)
        self.time = {}
        self.m = Model('UAVstrike')
        self.x1 = {}
        self.x2 = {}
        self.t1 = {}
        self.t2 = {}
        self.t = None
        self.elapsed_time = None
        self.setup_data()
        self.setup_variables()
        self.setup_constraints()
        self.setup_objective()

    def setup_data(self):
        for i in self.lst_i:
            for j in self.lst_j:
                for v in self.lst_v:
                    for k in self.lst_k:
                        self.time[i, j, v, k] = rd.randint(1, 30)

    def setup_variables(self):
        for i in self.lst_i:
            for j in self.lst_j:
                if i == j:
                    for v in self.lst_v:
                        self.x1[i, j, v, 2] = self.m.addVar(vtype=GRB.BINARY)
                elif i in self.lst_j:
                    for v in self.lst_v:
                        for k in self.lst_k:
                            self.x1[i, j, v, k] = self.m.addVar(vtype=GRB.BINARY)
                else:
                    for k in [1, 2, 3]:
                        self.x1[i, j, i - self.n, k] = self.m.addVar(vtype=GRB.BINARY)

        for i in self.lst_i:
            if i in self.lst_j:
                for v in self.lst_v:
                    self.x2[i, self.n + self.w + 1, v] = self.m.addVar(vtype=GRB.BINARY)
            else:
                self.x2[i, self.n + self.w + 1, i - self.n] = self.m.addVar(vtype=GRB.BINARY)

        for v in self.lst_v:
            self.t2[v] = self.m.addVar(vtype=GRB.CONTINUOUS)

        for j in self.lst_j:
            for k in self.lst_k:
                self.t1[j, k] = self.m.addVar(vtype=GRB.CONTINUOUS)

        self.t = self.m.addVar(vtype=GRB.CONTINUOUS)

    def setup_objective(self):
        self.m.setObjective(0.1 * quicksum(self.t1[j, k] for j in self.lst_j for k in self.lst_k) + self.t, GRB.MINIMIZE)
        self.m.update()

    def setup_constraints(self):
        ## Add constraints
        # Equality constraints
        ''' Mission completion requires that all three tasks are performed on each target exactly one time. Similar to linear assignment problems'''
        ''' If more targets than vehicles, don't include C1.1 and C1.2'''
        ''' 3n constraints'''
        for k in [1, 3]:
            for j in self.lst_j:
                # C1.1 (4)
                self.m.addLConstr(quicksum(
                    self.x1[i, j, v, k] for i in self.lst_i if i != j for v in self.lst_v if (i, j, v, k) in self.x1),
                                  GRB.EQUAL, 1)
        for k in [2]:
            for j in self.lst_j:
                # C1.2 (5)
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, k] for i in self.lst_i for v in self.lst_v if (i, j, v, k) in self.x1),
                    GRB.EQUAL, 1)

        '''Not more than one AV is assigned to perform a specific task k on a specified target j'''
        ''' wnk constraints'''
        # Inequality constraints
        for k in [1, 3]:
            for j in self.lst_j:
                # C2.1 (6) If more targets than vehicles, C2 is interesting
                self.m.addLConstr(quicksum(
                    self.x1[i, j, v, k] for v in self.lst_v for i in self.lst_i if i != j and (i, j, v, k) in self.x1),
                                  GRB.LESS_EQUAL, 1)

        for j in self.lst_j:
            # C2.2 (7)
            self.m.addLConstr(
                quicksum(self.x1[i, j, v, 2] for v in self.lst_v for i in self.lst_i if (i, j, v, 2) in self.x1),
                GRB.LESS_EQUAL, 1)

        '''An AV v, coming from the outside, can visit target j at most once:'''
        ''' (n+1)w constraints'''
        for v in self.lst_v:
            for j in self.lst_j:
                # C3.1 (8)
                self.m.addLConstr(quicksum(
                    self.x1[i, j, v, k] for k in self.lst_k for i in self.lst_i if i != j and (i, j, v, k) in self.x1),
                                  GRB.LESS_EQUAL, 1)
        '''each AV v can only enter the sink once'''
        for v in self.lst_v:
            # C3.2 (9)
            self.m.addLConstr(quicksum(
                self.x2[i, self.n + self.w + 1, v] for i in self.lst_i if (i, self.n + self.w + 1, v) in self.x2),
                              GRB.LESS_EQUAL, 1)

        ''' AV v leaves node j at most once '''
        ''' nw constraints '''
        for v in self.lst_v:
            for j in self.lst_j:
                # C4.1 (10)
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] for i in self.lst_j if i != j for k in self.lst_k if (i, j, v, k) in self.x1 and (j, self.n + self.w + 1, v) in self.x2) + self.x2[j, self.n + self.w + 1, v], GRB.LESS_EQUAL, 1)

        ''' A munition is perishable. Thus, an AV v can be assigned to attack at most one target. '''
        ''' w constraints'''
        for v in self.lst_v:
            # C5.1 (11)
            self.m.addLConstr(
                quicksum(self.x1[i, j, v, 2] for j in self.lst_j for i in self.lst_i if (i, j, v, 2) in self.x1),
                GRB.LESS_EQUAL, 1)

        ''' If AV v is assigned to fly to target j for verification, it cannot possibly be assigned to attack target j:'''
        ''' wn constraints'''
        for v in self.lst_v:
            for j in self.lst_j:
                # C6.1 (12)
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, 2] for i in self.lst_i if i != j and (i, j, v, 2) in self.x1),
                    GRB.LESS_EQUAL,
                    1 - quicksum(self.x1[i, j, v, 3] for i in self.lst_i if i != j and (i, j, v, 3) in self.x1))

        # Continuity constraints
        ''' If AV v enters target (node) j for the purpose of performing task 3, it must also exit target j:'''
        ''' 2 * wn constraints'''
        for j in self.lst_j:
            for v in self.lst_v:
                # C7.1 (13)
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, 3] for i in self.lst_i if i != j and (i, j, v, 3) in self.x1),
                    GRB.LESS_EQUAL, quicksum(self.x1[j, i, v, k] for k in self.lst_k for i in self.lst_j if
                                             i != j and (j, i, v, k) in self.x1 and (
                                             j, self.n + self.w + 1, v) in self.x2) + self.x2[
                        j, self.n + self.w + 1, v])
                # C7.2 (14)
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, 1] for i in self.lst_i if i != j and (i, j, v, 1) in self.x1),
                    GRB.LESS_EQUAL, quicksum(self.x1[j, i, v, k] for k in self.lst_k for i in self.lst_j if
                                             i != j and (j, i, v, k) in self.x1 and (j, j, v, 2) in self.x1 and (
                                             j, self.n + self.w + 1, v) in self.x2) + self.x1[j, j, v, 2] + self.x2[
                        j, self.n + self.w + 1, v])

        ''' Thus, if AV v is assigned to fly to target (node) j to perform task k 1⁄4 2, then, at any other point in time, AV v cannot also be assigned to fly from target j to a target i, iaj, to perform any other tastk at target i;'''
        ''' 2 * wn constraints'''
        for j in self.lst_j:
            for v in self.lst_v:
                # C7.3 (15)
                self.m.addLConstr(quicksum(self.x1[j, i, v, k] for i in self.lst_j if i != j for k in self.lst_k if
                                           (j, i, v, k) in self.x1 and (j, self.n + self.w + 1, v) in self.x2) +
                                  self.x2[j, self.n + self.w + 1, v], GRB.LESS_EQUAL,
                                  1 - quicksum(self.x1[i, j, v, 2] for i in self.lst_i if (i, j, v, 2) in self.x1))
                ''' If AV v is not assigned to visit node j, then it cannot possibly be assigned to fly out of node j.'''
                # C7.4 (16)
                self.m.addLConstr(quicksum(self.x1[j, i, v, k] for i in self.lst_j if i != j for k in self.lst_k if
                                           (j, i, v, k) in self.x1 and (j, self.n + self.w + 1, v) in self.x2) +
                                  self.x2[j, self.n + self.w + 1, v], GRB.LESS_EQUAL, quicksum(
                    self.x1[i, j, v, k] for i in self.lst_i if i != j for k in self.lst_k if (i, j, v, k) in self.x1))

        ''' All AVs leave the source nodes. An AV leaves the source node even if this entails a direct assignment to the sink.'''
        ''' w constraints'''
        for v in self.lst_v:
            # C7.5 (17)
            self.m.addLConstr(quicksum(self.x1[self.n + v, j, v, k] for j in self.lst_j for k in self.lst_k if
                                       (self.n + v, j, v, k) in self.x1 and (
                                       self.n + v, self.n + self.w + 1, v) in self.x2) + self.x2[
                                  self.n + v, self.n + self.w + 1, v], GRB.EQUAL, 1)

        ''' An AV cannot attack target (node) i, coming from target (node) i, unless it entered target (node) i to perform a classification.'''
        ''' wn constraints'''
        for i in self.lst_j:
            for v in self.lst_v:
                # C7.6 (18)
                self.m.addLConstr(self.x1[i, i, v, 2], GRB.LESS_EQUAL,
                                  quicksum(self.x1[j, i, v, 1] for j in self.lst_i if (j, i, v, 1) in self.x1))

        # Timing constraints
        for i in self.lst_j:
            for j in self.lst_j:
                if i != j:
                    for v in self.lst_v:
                        for k in [1, 3]:
                            # 8.1 (20) #TODO: test
                            self.m.addLConstr(self.t1[j, k] <= self.t1[i, 1] + self.time[i, j, v, k] + (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                    (l, i, v, 1) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)
                            # 8.2 (21) #TODO: test
                            self.m.addLConstr(self.t1[j, k] >= self.t1[i, 1] + self.time[i, j, v, k] - (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                    (l, i, v, 1) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)
                            # 8.3 (22) #TODO: test
                            self.m.addLConstr(self.t1[j, k] <= self.t1[i, 3] + self.time[i, j, v, k] + (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                    (l, i, v, 3) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)
                            # 8.4 (23) #TODO: test
                            self.m.addLConstr(self.t1[j, k] >= self.t1[i, 3] + self.time[i, j, v, k] - (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                    (l, i, v, 3) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)

        # Timing constraints
        for i in self.lst_j:
            for j in self.lst_j:
                if i != j:
                    for v in self.lst_v:
                        # 8.5 (24) #TODO: test
                        self.m.addLConstr(self.t1[j, 2] <= self.t1[i, 1] + self.time[i, j, v, 2] + (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                (l, i, v, 1) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)
                        # 8.6 (25) #TODO: test
                        self.m.addLConstr(self.t1[j, 2] >= self.t1[i, 1] + self.time[i, j, v, 2] - (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                (l, i, v, 1) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)
                        # 8.7 (26) #TODO: test
                        self.m.addLConstr(self.t1[j, 2] <= self.t1[i, 3] + self.time[i, j, v, 2] + (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                (l, i, v, 3) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)
                        # 8.8 (27) #TODO: test
                        self.m.addLConstr(self.t1[j, 2] >= self.t1[i, 3] + self.time[i, j, v, 2] - (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                (l, i, v, 3) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)

        for j in self.lst_j:
            for v in self.lst_v:
                for k in self.lst_k:
                    self.m.addConstr(self.t1[j, k] <= self.t2[v] + self.time[self.n + v, j, v, k] + (
                                1 - self.x1[self.n + v, j, v, k]) * self.T)
                    self.m.addConstr(self.t1[j, k] >= self.t2[v] + self.time[self.n + v, j, v, k] - (
                                1 - self.x1[self.n + v, j, v, k]) * self.T)

        for j in self.lst_j:
            self.m.addConstr(self.t1[j, 1] + self.delay <= self.t1[j, 2])
            self.m.addConstr(self.t1[j, 2] + self.delay <= self.t1[j, 3])

        for v in self.lst_v:
            self.m.addLConstr(quicksum(
                self.time[i, j, v, k] * self.x1[i, j, v, k] for k in self.lst_k for i in self.lst_i for j in self.lst_j
                if j != i and (i, j, v, k) in self.x1), GRB.LESS_EQUAL, self.T)

        self.m.update()

    def optimize(self):
        start_time = tm.time()
        self.m.optimize()
        end_time = tm.time()
        self.elapsed_time = round(end_time - start_time, 2)
        self.m.write('bigboy.lp')

    def save(self):
        dict_dv = {'x1': {}, 'x2': {}, 't1': {}, 't2': {}, 'Model': {}}
        for x1, value1 in self.x1.items():
            dict_dv['x1'][x1] = value1.X
        for x2, value2 in self.x2.items():
            dict_dv['x2'][x2] = value2.X
        for t1, value3 in self.t1.items():
            dict_dv['t1'][t1] = value3.X
        for t2, value4 in self.t2.items():
                dict_dv['t2'][t2] = value4.X
        dict_dv['Model'] = {'n': self.n, 'w': self.w, 'T': self.T, 'delay': self.delay, 'finaltime': self.t}
        with open(self.filename, 'wb') as f:
            pickle.dump(dict_dv, f)

    def print_solution(self):
        if self.m.status == GRB.OPTIMAL:
            print("Optimal solution found:")
            for i in self.lst_i:
                for j in self.lst_j:
                    for v in self.lst_v:
                        for k in self.lst_k:
                            if (i, j, v, k) in self.x1 and self.x1[i, j, v, k].X > 0.5:
                                print(f"UAV {v} assigned from {i} to {j} for task {k}")
            for i in self.lst_i:
                for v in self.lst_v:
                    if (i, self.n + self.w + 1, v) in self.x2 and self.x2[i, self.n + self.w + 1, v].X > 0.5:
                        print(f'UAV {v} flew to sinknode from node {i}')

            for j in self.lst_j:
                for k in self.lst_k:
                    print(f"Task {k} on target {j} completed at time {self.t1[j, k].X}")
        else:
            print("No optimal solution found.")

if __name__ == "__main__":
    model = UAVStrikeModel(n_targets=4, n_uavs=8, endurance=100)
    model.optimize()
    print(f'TIME ELAPSED: {model.elapsed_time} s')
    model.print_solution()
    model.save()

