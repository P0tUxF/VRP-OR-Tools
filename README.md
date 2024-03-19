# Testing Solomon Instances for Vehicle Routing Problem with Time Windows and Capacity Constraints

This repository contains scripts and tools for testing Solomon instances with the Vehicle Routing Problem (VRP) that incorporates time windows and capacity constraints. The tests are conducted using the Google OR-Tools Solver.

## Background

The Solomon instances are a set of benchmark problems commonly used to evaluate algorithms for the VRP. They consist of a variety of instances with different numbers of customers, vehicle capacities, and time window constraints. Incorporating time windows and capacity constraints adds complexity to the problem, making it more realistic and applicable to real-world scenarios.

Google OR-Tools Solver is a powerful optimization library developed by Google that provides various algorithms and tools for solving combinatorial optimization problems. It offers efficient solutions for VRP and supports the incorporation of constraints such as time windows and capacity limits.

## Purpose

The purpose of this repository is to provide a framework for testing the performance of Google OR-Tools Solver on Solomon instances with time windows and capacity constraints. By running these tests, users can evaluate the effectiveness and efficiency of the solver on different problem instances and compare its performance against other algorithms or solvers.

## Usage

To use this repository, follow these steps:

1. Clone the repository to your local machine.
2. Install Google OR-Tools library following the instructions provided [here](https://developers.google.com/optimization/install).
3. Run the provided scripts to generate and solve Solomon instances with time windows and capacity constraints using the Google OR-Tools Solver.
4. Analyze the results to evaluate the solver's performance.

TLDR:

```sh
python3 -m venv venv
source venv/bin/activate
python -m pip install -r requirements.txt
./main.py
```

## Repository Structure

- `solomon_25/`: Contains the Solomon instances files (.txt) with time window and capacity constraints for 25 nodes.
- `main.py/`: Contains Python scripts for generating and solving Solomon instances using Google OR-Tools Solver.
- `README.md`: This README file providing an overview of the repository and instructions for usage.

## Contributing

Contributions to this repository are welcome. If you have suggestions for improvements, new features, or bug fixes, please feel free to open an issue or submit a pull request.

## License

This repository is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Disclaimer

This repository is provided as-is, without any warranties or guarantees. The users are responsible for reviewing and testing the scripts and tools provided here in their own environment before using them in production or critical systems.

## Acknowledgments

- Solomon, M. M. (1987). Algorithms for the vehicle routing and scheduling problems with time window constraints. Operations Research, 35(2), 254-265.
- Google OR-Tools development team for providing the powerful optimization library.
- Contributors to this repository for their efforts in testing and improving the scripts and tools.
