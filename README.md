# Operation Optimization Project

## Introduction

The optimization of military operations, particularly in the domain of Unmanned Aerial Vehicles (UAVs), is critical due to the increasing reliance on UAVs for various tactical missions. This project focuses on optimizing the task allocation for UAVs using a Mixed-Integer Linear Programming (MILP) model. The main objective is to ensure efficient execution of three sequential tasks: classification, attack, and damage assessment, on multiple geographically dispersed targets.

This repository contains the code and files necessary to develop, verify, and analyze the MILP model for UAV task allocation. It also includes a case study where the tasks are used for medical support rather than offensive missions.

## Repository Contents

### UAVModelClass.py
This file contains the `UAVStrikeModel` class, which encapsulates the MILP model. The class includes methods for setting up the model, adding variables and constraints, defining the objective function, and optimizing the model.

### Unittests.py
This file contains unit tests for the `UAVStrikeModel` class using the `unittest` framework. The tests verify that the model constraints are correctly implemented and that the model optimizes as expected.

### Case_Study.py
This script performs a case study where the UAV tasks are adapted for medical support missions. It demonstrates how to use the `UAVStrikeModel` class in a practical scenario and analyzes the results.

### Sensitivity_Analysis.py
This script performs sensitivity analysis on the MILP model. It assesses how changes in model parameters affect the optimal solution and provides insights into the model's robustness.

### Contourplots.py
This script generates contour plots for visualizing the results of the sensitivity analysis. It helps in understanding the impact of different parameters on the model's performance.

### coordinates.py
This file contains helper functions for managing the coordinates of the targets and UAVs. It assists in setting up the geographical layout for the MILP model.

### f_helper.py
This file includes additional helper functions used throughout the project. These functions support various tasks such as data processing and model setup.

### f_sensitivity.py
This file contains functions specifically for performing sensitivity analysis. It is used in conjunction with `Sensitivity_Analysis.py` to evaluate the model's sensitivity to parameter changes.

### f_visualisation.py
This file includes functions for visualizing the results of the MILP model and sensitivity analysis. It helps in creating charts and plots for better interpretation of the results.

### requirements.txt
This file lists all the dependencies required to run the project. It includes specific versions of libraries such as `pandas`, `gurobipy`, `numpy`, and `matplotlib` to ensure compatibility.

## Getting Started

1. **Clone the repository:**
   ```sh
   git clone https://github.com/yourusername/operation-optimization-project.git
   cd operation-optimization-project
   ```

2. **Install dependencies:**
   ```sh
   pip install -r requirements.txt
   ```

3. **Run the unit tests to ensure everything is set up correctly:**
   ```sh
   python Unittests.py
   ```

4. **Execute the case study:**
   ```sh
   python Case_Study.py
   ```

5. **Perform sensitivity analysis:**
   ```sh
   python Sensitivity_Analysis.py
   ```

## Contributing

Contributions to this project are welcome. Please fork the repository, create a new branch, and submit a pull request with your changes.
