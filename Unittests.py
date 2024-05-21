import unittest
import gurobipy as gp
from collections import defaultdict

from UAVModel import time, n, w, lst_v, lst_j, lst_k, lst_i, x1, x2, t1, t2, T


class TestGurobiModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Load the model
        cls.lp_file_path = 'bigboy.lp'
        cls.model = gp.read(cls.lp_file_path)
        cls.model.optimize()

        # Ensure the model has an optimal solution
        if cls.model.status != gp.GRB.OPTIMAL:
            raise Exception(f"Model not solved to optimality. Status: {cls.model.status}")

    def test_constraints(self):
        # Loop through all the constraints in the model
        for constr in self.model.getConstrs():
            # Calculate the left-hand side of the constraint
            lhs = sum(var.x * self.model.getCoeff(constr, var) for var in self.model.getVars())
            sense = constr.Sense  # Sense of the constraint
            rhs = constr.RHS  # Right-hand side of the constraint

            if sense == gp.GRB.LESS_EQUAL:
                self.assertLessEqual(lhs, rhs, f"Constraint {constr.ConstrName} not satisfied: {lhs} <= {rhs}")
            elif sense == gp.GRB.GREATER_EQUAL:
                self.assertGreaterEqual(lhs, rhs, f"Constraint {constr.ConstrName} not satisfied: {lhs} >= {rhs}")
            elif sense == gp.GRB.EQUAL:
                self.assertAlmostEqual(lhs, rhs, places=5, msg=f"Constraint {constr.ConstrName} not satisfied: {lhs} == {rhs}")

    def test_solution_status(self):
        self.assertEqual(self.model.status, gp.GRB.OPTIMAL, "Model did not reach an optimal solution")

    def test_variable_bounds(self):
        # Verify that all variable bounds are respected
        for var in self.model.getVars():
            self.assertGreaterEqual(var.x, var.lb, f"Variable {var.VarName} lower bound not respected: {var.x} >= {var.lb}")
            self.assertLessEqual(var.x, var.ub, f"Variable {var.VarName} upper bound not respected: {var.x} <= {var.ub}")

    def test_constraint_types(self):
        constraint_types = defaultdict(list)

        # Group constraints by their format
        for constr in self.model.getConstrs():
            lhs_terms = sorted([(self.model.getCoeff(constr, var), var.VarName) for var in self.model.getVars() if self.model.getCoeff(constr, var) != 0])
            constraint_format = (len(lhs_terms), constr.Sense, constr.RHS)
            constraint_types[constraint_format].append(constr)

        # Print information about each type of constraint
        for constr_format, constr_list in constraint_types.items():
            num_terms, sense, rhs = constr_format
            sense_str = {gp.GRB.LESS_EQUAL: "<=", gp.GRB.GREATER_EQUAL: ">=", gp.GRB.EQUAL: "=="}[sense]

            # Print the general format of the constraint
            lhs_terms = sorted([(self.model.getCoeff(constr_list[0], var), var.VarName) for var in self.model.getVars() if self.model.getCoeff(constr_list[0], var) != 0])
            constraint_expression = " + ".join([f"{coeff}*{var}" for coeff, var in lhs_terms])
            constraint_example = f"{constraint_expression} {sense_str} {rhs}"
            print(f"General format of constraint: {constraint_example}")
            print(f"Number of this type of constraint: {len(constr_list)}")

            # Check if at least one example constraint is satisfied
            example_constr = constr_list[0]
            lhs = sum(var.x * self.model.getCoeff(example_constr, var) for var in self.model.getVars())
            if sense == gp.GRB.LESS_EQUAL:
                satisfied = lhs <= rhs
            elif sense == gp.GRB.GREATER_EQUAL:
                satisfied = lhs >= rhs
            elif sense == gp.GRB.EQUAL:
                satisfied = abs(lhs - rhs) < 1e-5

            print(f"Example constraint: {example_constr.ConstrName} is {'satisfied' if satisfied else 'not satisfied'}", '\n')

    def test_redundant_constraints(self):
        # Test Symmetry in Task Assignment
        for i in lst_i:
            for j in lst_i:
                if i != j:
                    for v in lst_v:
                        for k in lst_k:
                            if (i, j, v, k) in x1 and (j, i, v, k) in x1:
                                lhs = x1[i, j, v, k].X + x1[j, i, v, k].X
                                self.assertLessEqual(lhs, 1, f"Redundant constraint (symmetry in task assignment) violated for {i}->{j} and {j}->{i} for UAV {v} and task {k}")

        # Test Task Completion Ordering
        for i in lst_j:
            if (i, 1) in t1 and (i, 3) in t1:
                self.assertLessEqual(t1[i, 1].X, t1[i, 3].X, f"Redundant constraint (task completion ordering) violated for node {i}")

        # Test Number of Drones to Sink Node
        num_drones_to_sink = sum(x2[i, n + w + 1, v].X for i in lst_i for v in lst_v if (i, n + w + 1, v) in x2)
        expected_drones_to_sink = w - n
        self.assertEqual(num_drones_to_sink, expected_drones_to_sink, f"Redundant constraint (number of drones to sink node) violated: {num_drones_to_sink} != {expected_drones_to_sink}")

        # Test Drone Capacity
        max_capacity = n + 1
        for v in lst_v:
            total_tasks = sum(x1[i, j, v, k].X for i in lst_i for j in lst_j for k in lst_k if (i, j, v, k) in x1)
            self.assertLessEqual(total_tasks, max_capacity, f"Redundant constraint (drone capacity) violated for UAV {v}: {total_tasks} > {max_capacity}")

    def test_task_completion(self):
        # For classification and verification tasks (tasks 1 and 3)
        for k in [1, 3]:
            for j in lst_j:
                lhs = sum(x1[i, j, v, k].X for v in lst_v for i in lst_i if (i, j, v, k) in x1)
                self.assertAlmostEqual(lhs, 1, msg=f"Task completion constraint violated for target {j}, task {k}")

        # For attack tasks (task 2)
        for j in lst_j:
            lhs = sum(x1[i, j, v, 2].X for v in lst_v for i in lst_i if (i, j, v, 2) in x1)
            self.assertAlmostEqual(lhs, 1, msg=f"Task completion constraint violated for target {j}, task 2")

    def test_unique_task_assignment(self):
        # For classification and verification tasks (tasks 1 and 3)
        for k in [1, 3]:
            for v in lst_v:
                for j in lst_j:
                    lhs = sum(x1[i, j, v, k].X for i in lst_i if (i, j, v, k) in x1)
                    self.assertLessEqual(lhs, 1, msg=f"Unique task assignment constraint violated for UAV {v}, target {j}, task {k}")

        # For attack tasks (task 2)
        for v in lst_v:
            for j in lst_j:
                lhs = sum(x1[i, j, v, 2].X for i in lst_i if (i, j, v, 2) in x1)
                self.assertLessEqual(lhs, 1, msg=f"Unique task assignment constraint violated for UAV {v}, target {j}, task 2")

    def test_single_visit_to_target(self):
        for v in lst_v:
            for j in lst_j:
                lhs = sum(x1[i, j, v, k].X for k in lst_k for i in lst_i if (i, j, v, k) in x1 and i !=j)
                self.assertLessEqual(lhs, 1, msg=f"Single visit to target constraint violated for UAV {v}, target {j}")

    def test_single_entry_to_sink(self):
        for v in lst_v:
            lhs = sum(x2[i, n + w + 1, v].X for i in lst_i if (i, n + w + 1, v) in x2)
            self.assertLessEqual(lhs, 1, msg=f"Single entry to sink constraint violated for UAV {v}")

    def test_single_attack_per_uav(self):
        for v in lst_v:
            lhs = sum(x1[i, j, v, 2].X for j in lst_j for i in lst_i if (i, j, v, 2) in x1)
            self.assertLessEqual(lhs, 1, msg=f"Single attack per UAV constraint violated for UAV {v}")

    def test_attack_and_verification_exclusive(self):
        for v in lst_v:
            for j in lst_j:
                lhs_attack = sum(x1[i, j, v, 2].X for i in lst_i if (i, j, v, 2) in x1)
                lhs_verify = sum(x1[i, j, v, 3].X for i in lst_i if (i, j, v, 3) in x1)
                self.assertLessEqual(lhs_attack + lhs_verify, 1, msg=f"Attack and verification exclusive constraint violated for UAV {v}, target {j}")

    def test_timing_constraints(self):
        for v in lst_v:
            for i in lst_i:
                for j in lst_j:
                    if i != j:
                        for k in [1, 3]:
                            if (j, k) in t1 and (i, 1) in t1:
                                lhs1 = t1[j, k].X
                                rhs1 = t1[i, 1].X + time[i, j, v, k] + (2 - x1[i, j, v, k].X - sum(
                                    x1[l, i, v, 1].X for l in lst_i if (l, i, v, 1) in x1)) * w * T
                                self.assertLessEqual(lhs1, rhs1,
                                                     msg=f"Timing constraint (classification and verification) violated for UAV {v}, nodes {i}->{j}, task {k}")

                                lhs2 = t1[j, k].X
                                rhs2 = t1[i, 1].X + time[i, j, v, k] - (2 - x1[i, j, v, k].X - sum(
                                    x1[l, i, v, 1].X for l in lst_i if (l, i, v, 1) in x1)) * w * T
                                self.assertGreaterEqual(lhs2, rhs2,
                                                        msg=f"Timing constraint (classification and verification) violated for UAV {v}, nodes {i}->{j}, task {k}")

                        if (j, 2) in t1 and (i, 1) in t1:
                            lhs3 = t1[j, 2].X
                            rhs3 = t1[i, 1].X + time[i, j, v, 2] + (2 - x1[i, j, v, 2].X - sum(
                                x1[l, i, v, 1].X for l in lst_i if (l, i, v, 1) in x1)) * w * T
                            self.assertLessEqual(lhs3, rhs3,
                                                 msg=f"Timing constraint (attack) violated for UAV {v}, nodes {i}->{j}")

                            lhs4 = t1[j, 2].X
                            rhs4 = t1[i, 1].X + time[i, j, v, 2] - (2 - x1[i, j, v, 2].X - sum(
                                x1[l, i, v, 1].X for l in lst_i if (l, i, v, 1) in x1)) * w * T
                            self.assertGreaterEqual(lhs4, rhs4,
                                                    msg=f"Timing constraint (attack) violated for UAV {v}, nodes {i}->{j}")

                        if (j, 2) in t1 and (i, 3) in t1:
                            lhs5 = t1[j, 2].X
                            rhs5 = t1[i, 3].X + time[i, j, v, 2] + (2 - x1[i, j, v, 2].X - sum(
                                x1[l, i, v, 3].X for l in lst_i if (l, i, v, 3) in x1)) * w * T
                            self.assertLessEqual(lhs5, rhs5,
                                                 msg=f"Timing constraint (verification) violated for UAV {v}, nodes {i}->{j}")

                            lhs6 = t1[j, 2].X
                            rhs6 = t1[i, 3].X + time[i, j, v, 2] - (2 - x1[i, j, v, 2].X - sum(
                                x1[l, i, v, 3].X for l in lst_i if (l, i, v, 3) in x1)) * w * T
                            self.assertGreaterEqual(lhs6, rhs6,
                                                    msg=f"Timing constraint (verification) violated for UAV {v}, nodes {i}->{j}")

            for j in lst_j:
                if (j, 1) in t1 and (j, 2) in t1:
                    lhs7 = t1[j, 1].X
                    rhs7 = t1[j, 2].X
                    self.assertLessEqual(lhs7, rhs7, msg=f"Timing constraint (task sequence) violated for node {j}")

                if (j, 2) in t1 and (j, 3) in t1:
                    lhs8 = t1[j, 2].X
                    rhs8 = t1[j, 3].X
                    self.assertLessEqual(lhs8, rhs8, msg=f"Timing constraint (task sequence) violated for node {j}")

    def test_vehicle_endurance_constraint(self):
        for v in lst_v:
            lhs = sum(
                time[i, j, v, k] * x1[i, j, v, k].X for k in lst_k for i in lst_i for j in lst_j if (i, j, v, k) in x1)
            self.assertLessEqual(lhs, T, msg=f"Vehicle endurance constraint violated for UAV {v}")

if __name__ == '__main__':
    unittest.main()
