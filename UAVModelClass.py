from gurobipy import *
import random as rd
import time as tm
import pickle

class UAVStrikeModel:
    def __init__(self, n_targets, n_uavs, endurance, delay=1, timedict=None, obj=2):
        self.n = n_targets  # Number of targets
        self.w = n_uavs  # Number of UAVs
        self.obj = obj
        self.filename = f'Results/{self.n}_{self.w}'  # Filename for saving results
        self.T = endurance  # UAV endurance
        self.delay = delay  # Delay parameter
        self.lst_i = range(1, self.n + self.w + 1)  # All nodes (targets + UAVs)
        self.lst_j = range(1, self.n + 1)  # Target nodes
        self.lst_v = range(1, self.w + 1)  # UAV nodes
        self.lst_k = range(1, 4)  # Task types (1, 2, 3)
        self.time = {}  # Dictionary for time data
        self.m = Model('UAVstrike')  # Gurobi model
        self.x1 = {}  # Decision variables for task assignments
        self.x2 = {}  # Decision variables for UAVs returning to sink
        self.t1 = {}  # Time variables for tasks at targets
        self.t2 = {}  # Time variables for UAVs
        self.t = None  # Final time variable
        self.elapsed_time = None  # To store optimization elapsed time
        self.time = timedict  # Optional time dictionary input
        if self.time is None:
            self.time = {}
            self.setup_data()  # Generate random time data if none provided
        self.setup_variables()  # Setup decision variables
        self.setup_constraints()  # Setup constraints
        self.setup_objective()  # Setup objective function

    def setup_data(self):
        # Randomly generate time data
        for i in self.lst_i:
            for j in self.lst_j:
                for v in self.lst_v:
                    for k in self.lst_k:
                        self.time[i, j, v, k] = rd.randint(1, 30)

    def setup_variables(self):
        # Initialize decision variables
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
        # Objective function to minimize
        if self.obj == 1:
            self.m.setObjective(quicksum(self.time[i, j, v, k] * self.x1[i, j, v, k] for i in self.lst_i for j in self.lst_j for v in self.lst_v for k in self.lst_k if (i, j, v, k) in self.x1), GRB.MINIMIZE)
        elif self.obj == 2:
            #self.m.setObjective(0.1 * quicksum(self.t1[j, k] for j in self.lst_j for k in self.lst_k) + self.t, GRB.MINIMIZE)
            self.m.setObjective(self.t, GRB.MINIMIZE)
        elif self.obj == 3:
            self.m.setObjective(quicksum(self.x2[i, self.n + self.w + 1, v] for i in self.lst_i for v in self.lst_v if (i, self.n + self.w + 1, v) in self.x2), GRB.MAXIMIZE)
        self.m.update()

    def setup_constraints(self):
        # Add constraints
        # Mission completion constraints
        for k in [1, 3]:
            for j in self.lst_j:
                # Each task must be performed exactly once on each target
                self.m.addLConstr(quicksum(
                    self.x1[i, j, v, k] for i in self.lst_i if i != j for v in self.lst_v if (i, j, v, k) in self.x1),
                                  GRB.EQUAL, 1)
        for k in [2]:
            for j in self.lst_j:
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, k] for i in self.lst_i for v in self.lst_v if (i, j, v, k) in self.x1),
                    GRB.EQUAL, 1)

        # UAV assignment constraints
        for k in [1, 3]:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(
                    self.x1[i, j, v, k] for v in self.lst_v for i in self.lst_i if i != j and (i, j, v, k) in self.x1),
                                  GRB.LESS_EQUAL, 1)
        for j in self.lst_j:
            self.m.addLConstr(
                quicksum(self.x1[i, j, v, 2] for v in self.lst_v for i in self.lst_i if (i, j, v, 2) in self.x1),
                GRB.LESS_EQUAL, 1)

        # UAV visit constraints
        for v in self.lst_v:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(
                    self.x1[i, j, v, k] for k in self.lst_k for i in self.lst_i if i != j and (i, j, v, k) in self.x1),
                                  GRB.LESS_EQUAL, 1)
        for v in self.lst_v:
            self.m.addLConstr(quicksum(
                self.x2[i, self.n + self.w + 1, v] for i in self.lst_i if (i, self.n + self.w + 1, v) in self.x2),
                              GRB.LESS_EQUAL, 1)

        for v in self.lst_v:
            for j in self.lst_j:
                self.m.addLConstr(quicksum(self.x1[i, j, v, k] for i in self.lst_j if i != j for k in self.lst_k if (i, j, v, k) in self.x1 and (j, self.n + self.w + 1, v) in self.x2) + self.x2[j, self.n + self.w + 1, v], GRB.LESS_EQUAL, 1)

        for v in self.lst_v:
            self.m.addLConstr(
                quicksum(self.x1[i, j, v, 2] for j in self.lst_j for i in self.lst_i if (i, j, v, 2) in self.x1),
                GRB.LESS_EQUAL, 1)

        for v in self.lst_v:
            for j in self.lst_j:
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, 2] for i in self.lst_i if i != j and (i, j, v, 2) in self.x1),
                    GRB.LESS_EQUAL,
                    1 - quicksum(self.x1[i, j, v, 3] for i in self.lst_i if i != j and (i, j, v, 3) in self.x1))

        # Continuity constraints
        for j in self.lst_j:
            for v in self.lst_v:
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, 3] for i in self.lst_i if i != j and (i, j, v, 3) in self.x1),
                    GRB.LESS_EQUAL, quicksum(self.x1[j, i, v, k] for k in self.lst_k for i in self.lst_j if
                                             i != j and (j, i, v, k) in self.x1 and (
                                             j, self.n + self.w + 1, v) in self.x2) + self.x2[
                        j, self.n + self.w + 1, v])
                self.m.addLConstr(
                    quicksum(self.x1[i, j, v, 1] for i in self.lst_i if i != j and (i, j, v, 1) in self.x1),
                    GRB.LESS_EQUAL, quicksum(self.x1[j, i, v, k] for k in self.lst_k for i in self.lst_j if
                                             i != j and (j, i, v, k) in self.x1 and (j, j, v, 2) in self.x1 and (
                                             j, self.n + self.w + 1, v) in self.x2) + self.x1[j, j, v, 2] + self.x2[
                        j, self.n + self.w + 1, v])

        for j in self.lst_j:
            for v in self.lst_v:
                self.m.addLConstr(quicksum(self.x1[j, i, v, k] for i in self.lst_j if i != j for k in self.lst_k if
                                           (j, i, v, k) in self.x1 and (j, self.n + self.w + 1, v) in self.x2) +
                                  self.x2[j, self.n + self.w + 1, v], GRB.LESS_EQUAL,
                                  1 - quicksum(self.x1[i, j, v, 2] for i in self.lst_i if (i, j, v, 2) in self.x1))
                self.m.addLConstr(quicksum(self.x1[j, i, v, k] for i in self.lst_j if i != j for k in self.lst_k if
                                           (j, i, v, k) in self.x1 and (j, self.n + self.w + 1, v) in self.x2) +
                                  self.x2[j, self.n + self.w + 1, v], GRB.LESS_EQUAL, quicksum(
                    self.x1[i, j, v, k] for i in self.lst_i if i != j for k in self.lst_k if (i, j, v, k) in self.x1))

        for v in self.lst_v:
            self.m.addLConstr(quicksum(self.x1[self.n + v, j, v, k] for j in self.lst_j for k in self.lst_k if
                                       (self.n + v, j, v, k) in self.x1 and (
                                       self.n + v, self.n + self.w + 1, v) in self.x2) + self.x2[
                                  self.n + v, self.n + self.w + 1, v], GRB.EQUAL, 1)

        for i in self.lst_j:
            for v in self.lst_v:
                self.m.addLConstr(self.x1[i, i, v, 2], GRB.LESS_EQUAL,
                                  quicksum(self.x1[j, i, v, 1] for j in self.lst_i if (j, i, v, 1) in self.x1))

        # Timing constraints
        for i in self.lst_j:
            for j in self.lst_j:
                if i != j:
                    for v in self.lst_v:
                        for k in [1, 3]:
                            self.m.addLConstr(self.t1[j, k] <= self.t1[i, 1] + self.time[i, j, v, k] + (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                    (l, i, v, 1) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)
                            self.m.addLConstr(self.t1[j, k] >= self.t1[i, 1] + self.time[i, j, v, k] - (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                    (l, i, v, 1) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)
                            self.m.addLConstr(self.t1[j, k] <= self.t1[i, 3] + self.time[i, j, v, k] + (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                    (l, i, v, 3) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)
                            self.m.addLConstr(self.t1[j, k] >= self.t1[i, 3] + self.time[i, j, v, k] - (
                                        2 - self.x1[i, j, v, k] - quicksum(
                                    self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                    (l, i, v, 3) in self.x1 and (i, j, v, k) in self.x1)) * self.w * self.T)

        # More timing constraints
        for i in self.lst_j:
            for j in self.lst_j:
                if i != j:
                    for v in self.lst_v:
                        self.m.addLConstr(self.t1[j, 2] <= self.t1[i, 1] + self.time[i, j, v, 2] + (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                (l, i, v, 1) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)
                        self.m.addLConstr(self.t1[j, 2] >= self.t1[i, 1] + self.time[i, j, v, 2] - (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 1] for l in self.lst_i if l != i if
                                (l, i, v, 1) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)
                        self.m.addLConstr(self.t1[j, 2] <= self.t1[i, 3] + self.time[i, j, v, 2] + (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                (l, i, v, 3) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)
                        self.m.addLConstr(self.t1[j, 2] >= self.t1[i, 3] + self.time[i, j, v, 2] - (
                                    2 - self.x1[i, j, v, 2] - quicksum(
                                self.x1[l, i, v, 3] for l in self.lst_i if l != i if
                                (l, i, v, 3) in self.x1 and (i, j, v, 2) in self.x1)) * self.w * self.T)

        # More timing constraints
        for j in self.lst_j:
            for v in self.lst_v:
                for k in self.lst_k:
                    self.m.addConstr(self.t1[j, k] <= self.t2[v] + self.time[self.n + v, j, v, k] + (
                                1 - self.x1[self.n + v, j, v, k]) * self.w * self.T)
                    self.m.addConstr(self.t1[j, k] >= self.t2[v] + self.time[self.n + v, j, v, k] - (
                                1 - self.x1[self.n + v, j, v, k]) * self.w * self.T)

        # Sequence of tasks
        for j in self.lst_j:
            self.m.addConstr(self.t1[j, 1] + self.delay <= self.t1[j, 2])
            self.m.addConstr(self.t1[j, 2] + self.delay <= self.t1[j, 3])

        # Vehicle's path cannot be longer than endurance
        for v in self.lst_v:
            self.m.addLConstr(quicksum(
                self.time[i, j, v, k] * self.x1[i, j, v, k] for k in self.lst_k for i in self.lst_i for j in self.lst_j
                if j != i and (i, j, v, k) in self.x1), GRB.LESS_EQUAL, self.T)

        # Total time is the longest time
        for j in self.lst_j:
            for k in self.lst_k:
                self.m.addLConstr(self.t >= self.t1[j, k])

        self.m.update()

    def optimize(self):
        # Optimize the model
        start_time = tm.time()  # Start timer
        self.m.optimize()  # Run optimization
        end_time = tm.time()  # End timer
        self.elapsed_time = round(end_time - start_time, 2)  # Calculate elapsed time
        self.m.write('test.lp')  # Write model to file

    def save(self, filename):
        # Save results
        dict_dv = {'x1': {}, 'x2': {}, 't1': {}, 't2': {}, 'Model': {}}  # Initialize dictionary
        for x1, value1 in self.x1.items():
            dict_dv['x1'][x1] = value1.X  # Save x1 variables
        for x2, value2 in self.x2.items():
            dict_dv['x2'][x2] = value2.X  # Save x2 variables
        for t1, value3 in self.t1.items():
            dict_dv['t1'][t1] = value3.X  # Save t1 variables
        for t2, value4 in self.t2.items():
                dict_dv['t2'][t2] = value4.X  # Save t2 variables
        dict_dv['Model'] = {'n': self.n, 'w': self.w, 'T': self.T, 'delay': self.delay, 'finaltime': self.t.X, 'time': self.time}  # Save model parameters
        print('finaltime', self.t.X)  # Print final time
        filename = f'Results/{filename}'
        with open(filename, 'wb') as f:
            pickle.dump(dict_dv, f)  # Save to file

    def sensitivity_analysis(self):
        try:
            if self.m.status == GRB.OPTIMAL:
                # Objective coefficient ranges
                obj_low = self.m.getAttr(GRB.Attr.SAObjLow)
                obj_up = self.m.getAttr(GRB.Attr.SAObjUp)

                # Right-hand side ranges
                rhs_low = self.m.getAttr(GRB.Attr.SARHSLow)
                rhs_up = self.m.getAttr(GRB.Attr.SARHSUp)

                # Variable bound ranges
                lb_low = self.m.getAttr(GRB.Attr.SALBLow)
                lb_up = self.m.getAttr(GRB.Attr.SALBUp)
                ub_low = self.m.getAttr(GRB.Attr.SAUBLow)
                ub_up = self.m.getAttr(GRB.Attr.SAUBUp)

                # Display sensitivity analysis results
                print("Sensitivity Analysis Results:")

                # Objective coefficient ranges
                print("\nObjective Coefficient Ranges:")
                for i in range(self.num_variables):
                    print(f"x[{i}]: ObjLow = {obj_low[i]}, ObjUp = {obj_up[i]}")

                # Right-hand side ranges
                print("\nRight-Hand Side Ranges:")
                for i in range(self.num_constraints):
                    print(f"Constraint {i}: RHS_Low = {rhs_low[i]}, RHS_Up = {rhs_up[i]}")

                # Variable bound ranges
                print("\nVariable Bound Ranges:")
                for i in range(self.num_variables):
                    print(f"x[{i}]: LB_Low = {lb_low[i]}, LB_Up = {lb_up[i]}, UB_Low = {ub_low[i]}, UB_Up = {ub_up[i]}")
            else:
                print("Sensitivity analysis is not available because no optimal solution was found")
        except GurobiError as e:
            print(f"Error during sensitivity analysis: {e}")


    def print_solution(self):
        # Print solution
        if self.m.status == GRB.OPTIMAL:  # Check if optimal solution found
            print("Optimal solution found:")
            for i in self.lst_i:
                for j in self.lst_j:
                    for v in self.lst_v:
                        for k in self.lst_k:
                            if (i, j, v, k) in self.x1 and self.x1[i, j, v, k].X > 0.5:
                                print(f"UAV {v} assigned from {i} to {j} for task {k}")  # Print assignments
            for i in self.lst_i:
                for v in self.lst_v:
                    if (i, self.n + self.w + 1, v) in self.x2 and self.x2[i, self.n + self.w + 1, v].X > 0.5:
                        print(f'UAV {v} flew to sinknode from node {i}')  # Print returns to sink

            for j in self.lst_j:
                for k in self.lst_k:
                    print(f"Task {k} on target {j} completed at time {self.t1[j, k].X}")  # Print task completion times
        else:
            print("No optimal solution found.")  # Print if no solution found


if __name__ == "__main__":
    model = UAVStrikeModel(n_targets=3, n_uavs=5, endurance=100)  # Initialize model
    model.optimize()  # Optimize model
    print(f'TIME ELAPSED: {model.elapsed_time} s')  # Print elapsed time
    model.print_solution()  # Print solution
    model.save('test')  # Save results
