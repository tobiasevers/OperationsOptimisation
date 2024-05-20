import unittest
import gurobipy as gp
from collections import defaultdict

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

if __name__ == '__main__':
    unittest.main()
