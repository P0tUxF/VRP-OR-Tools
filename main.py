#!/usr/bin/env python3
import numpy as np
import pandas as pd

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial import distance_matrix
import vrplib


# Read Solomon formatted instances
instance = vrplib.read_instance("solomon_25/C101.txt", instance_format="solomon")

coords = instance["node_coord"].tolist()
# Calculate distance matrix
dist_matrix = distance_matrix(coords, coords)
dist_matrix = dist_matrix * 500
data = {}
data["time_matrix"] = dist_matrix.astype(int)
data["num_vehicles"] = 25
data["depot"] = 0
data["time_windows"] = [
    tuple((tw[0] * 500, tw[1] * 500)) for tw in instance["time_window"].tolist()
]
data["demands"] = instance["demand"].tolist()
data["vehicle_capacities"] = [200] * 25

manager = pywrapcp.RoutingIndexManager(
    len(data["time_matrix"]), data["num_vehicles"], data["depot"]
)

routing = pywrapcp.RoutingModel(manager)


def demand_callback(from_index):
    """Returns the demand of the node."""
    # Convert from routing variable Index to demands NodeIndex.
    from_node = manager.IndexToNode(from_index)
    return data["demands"][from_node]


demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_callback_index,
    0,  # null capacity slack
    data["vehicle_capacities"],  # vehicle maximum capacities
    True,  # start cumul to zero
    "Capacity",
)


# Create and register a transit callback.
def time_callback(from_index, to_index):
    """Returns the travel time between the two nodes."""
    # Convert from routing variable Index to time matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data["time_matrix"][from_node][to_node]


transit_callback_index = routing.RegisterTransitCallback(time_callback)

# Define cost of each arc.
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Add Time Windows constraint.
time = "Time"
routing.AddDimension(
    transit_callback_index,
    1000000000000000,  # allow waiting time
    1000000000000000,  # maximum time per vehicle
    False,  # Don't force start cumul to zero.
    time,
)
time_dimension = routing.GetDimensionOrDie(time)
# Add time window constraints for each location except depot.
for location_idx, time_window in enumerate(data["time_windows"]):
    if location_idx == data["depot"]:
        continue
    index = manager.NodeToIndex(location_idx)
    print(f"Add TW: {location_idx}, [{time_window[0]},{time_window[1]}]")
    time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
# Add time window constraints for each vehicle start node.
depot_idx = data["depot"]
for vehicle_id in range(data["num_vehicles"]):
    index = routing.Start(vehicle_id)
    time_dimension.CumulVar(index).SetRange(
        data["time_windows"][depot_idx][0], data["time_windows"][depot_idx][1]
    )

# Instantiate route start and end times to produce feasible times.
for i in range(data["num_vehicles"]):
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
    routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

# Setting first solution heuristic.
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
# search_parameters.first_solution_strategy = (
# routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
# )
search_parameters.local_search_metaheuristic = (
    routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
)
search_parameters.time_limit.seconds = 15
# search_parameters.log_search = True

# Solve the problem.
print("Start solving...")
solution = routing.SolveWithParameters(search_parameters)


def print_solution(manager, routing, solution):
    """Prints solution on console."""
    print(f"Objective: {solution.ObjectiveValue()}")
    time_dimension = routing.GetDimensionOrDie("Time")
    total_time = 0
    for vehicle_id in range(manager.GetNumberOfVehicles()):
        index = routing.Start(vehicle_id)
        plan_output = f"Route for vehicle {vehicle_id}:\n"
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += (
                f"{manager.IndexToNode(index)}"
                f" Time({solution.Min(time_var)},{solution.Max(time_var)})"
                " -> "
            )
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += (
            f"{manager.IndexToNode(index)}"
            f" Time({solution.Min(time_var)},{solution.Max(time_var)})\n"
        )
        plan_output += f"Time of the route: {solution.Min(time_var)}min\n"
        print(plan_output)
        total_time += solution.Min(time_var)
    print(f"Total time of all routes: {total_time}min")


if solution:
    print_solution(manager, routing, solution)
