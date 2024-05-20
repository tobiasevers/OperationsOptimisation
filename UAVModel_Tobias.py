import pandas as pd
from gurobipy import *
import numpy as np
import xlsxwriter as xlsx
import networkx as nx
import matplotlib.pyplot as plt
import random as rd
import time as tm

class UAVStrikeModel:
    def __init__(self, n_targets, n_uavs, endurance, delay=1):
        self.n = n_targets
        self.w = n_uavs
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
        for k in [1, 3]:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] for i in self.lst_i if i != j for v in self.lst_v if (i, j, v, k) in self.x1), GRB.EQUAL, 1)
        for k in [2]:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] for i in self.lst_i for v in self.lst_v if (i, j, v, k) in self.x1), GRB.EQUAL, 1)

        for k in [1, 3]:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] for v in self.lst_v for i in self.lst_i if i != j and (i, j, v, k) in self.x1), GRB.LESS_EQUAL, 1)

        for j in self.lst_j:
            self.m.addLConstr(quicksum(self.x1[i, j, v, 2] for v in self.lst_v for i in self.lst_i if (i, j, v, 2) in self.x1), GRB.LESS_EQUAL, 1)

        for v in self.lst_v:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] for k in self.lst_k for i in self.lst_i if i != j and (i, j, v, k) in self.x1), GRB.LESS_EQUAL, 1)
        for v in self.lst_v:
            self.m.addLConstr(quicksum(self.x2[i, self.n + self.w + 1, v] for i in self.lst_i if (i, self.n + self.w + 1, v) in self.x2), GRB.LESS_EQUAL, 1)

        for v in self.lst_v:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] + self.x2[i, self.n + self.w + 1, v] for i in self.lst_i if i!=j for k in self.lst_k if (i, j, v, k) in self.x1 and (i, self.n + self.w + 1, v) in self.x2), GRB.LESS_EQUAL, 1)

        for v in self.lst_v:
            self.m.addLConstr(quicksum(self.x1[self.n + v, j, v, k] for j in self.lst_j for k in self.lst_k if (self.n + v, j, v, k) in self.x1 and (self.n + v, self.n + self.w + 1, v) in self.x2) + self.x2[self.n + v, self.n + self.w + 1, v], GRB.EQUAL, 1)

        for i in self.lst_j:
            for v in self.lst_v:
                self.m.addLConstr(self.x1[i, i, v, 2], GRB.LESS_EQUAL, quicksum(self.x1[j, i, v, 1] for j in self.lst_i if (j, i, v, 1) in self.x1))

        for i in self.lst_j:
            for j in self.lst_j:
                if i != j:
                    for v in self.lst_v:
                        for k in [1, 3]:
                            self.m.addLConstr(self.t1[j, k] <= self.t1[i, 1] + self.time[i, j, v, k] + (2 - self.x1[i, j, v, k] - quicksum(self.x1[l, i, v, 1] for l in self.lst_i if l != i and (l, i, v, 1) in self.x1)) * self.w * self.T)
                            self.m.addLConstr(self.t1[j, k] >= self.t1[i, 1] + self.time[i, j, v, k] - (2 - self.x1[i, j, v, k] - quicksum(self.x1[l, i, v, 1] for l in self.lst_i if l != i and (l, i, v, 1) in self.x1)) * self.w * self.T)
                            self.m.addLConstr(self.t1[j, k] <= self.t1[i, 3] + self.time[i, j, v, k] + (2 - self.x1[i, j, v, k] - quicksum(self.x1[l, i, v, 3] for l in self.lst_i if l != i and (l, i, v, 3) in self.x1)) * self.w * self.T)
                            self.m.addLConstr(self.t1[j, k] >= self.t1[i, 3] + self.time[i, j, v, k] - (2 - self.x1[i, j, v, k] - quicksum(self.x1[l, i, v, 3] for l in self.lst_i if l != i and (l, i, v, 3) in self.x1)) * self.w * self.T)

        for i in self.lst_j:
            for j in self.lst_j:
                for v in self.lst_v:
                    self.m.addLConstr(self.t1[j, 2] <= self.t1[i, 1] + self.time[i, j, v, 2] + (2 - self.x1[i, j, v, 2] - quicksum(self.x1[l, i, v, 1] for l in self.lst_i if l != i and (l, i, v, 1) in self.x1)) * self.w * self.T)
                    self.m.addLConstr(self.t1[j, 2] >= self.t1[i, 1] + self.time[i, j, v, 2] - (2 - self.x1[i, j, v, 2] - quicksum(self.x1[l, i, v, 1] for l in self.lst_i if l != i and (l, i, v, 1) in self.x1)) * self.w * self.T)
                    if i != j:
                        self.m.addLConstr(self.t1[j, 2] <= self.t1[i, 3] + self.time[i, j, v, 2] + (2 - self.x1[i, j, v, 2] - quicksum(self.x1[l, i, v, 3] for l in self.lst_i if l != i and (l, i, v, 3) in self.x1)) * self.w * self.T)
                        self.m.addLConstr(self.t1[j, 2] >= self.t1[i, 3] + self.time[i, j, v, 2] - (2 - self.x1[i, j, v, 2] - quicksum(self.x1[l, i, v, 3] for l in self.lst_i if l != i and (l, i, v, 3) in self.x1)) * self.w * self.T)

        for j in self.lst_j:
            for v in self.lst_v:
                for k in self.lst_k:
                    self.m.addConstr(self.t1[j, k] <= self.t2[v] + self.time[self.n + v, j, v, k] + (1 - self.x1[self.n + v, j, v, k]) * self.T)
                    self.m.addConstr(self.t1[j, k] >= self.t2[v] + self.time[self.n + v, j, v, k] - (1 - self.x1[self.n + v, j, v, k]) * self.T)

        for j in self.lst_j:
            self.m.addConstr(self.t1[j, 1] + self.delay <= self.t1[j, 2])
            self.m.addConstr(self.t1[j, 2] + self.delay <= self.t1[j, 3])

        for v in self.lst_v:
            self.m.addLConstr(quicksum(self.time[i, j, v, k] * self.x1[i, j, v, k] for k in self.lst_k for i in self.lst_i for j in self.lst_j if j != i and (i, j, v, k) in self.x1), GRB.LESS_EQUAL, self.T)

        self.m.update()

    def optimize(self):
        start_time = tm.time()
        self.m.optimize()
        end_time = tm.time()
        self.elapsed_time = round(end_time - start_time, 2)
        self.m.write('bigboy.lp')

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

# Usage example
model = UAVStrikeModel(n_targets=2, n_uavs=5, endurance=100)
model.optimize()
print(f'TIME ELAPSED: {model.elapsed_time} s')
model.print_solution()
