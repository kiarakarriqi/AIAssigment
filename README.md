1. Introduction
  The objective of this assignment was to design a search agent capable of solving a custom puzzle domain using both uninformed and informed search strategies. Instead of standard examples like the 8-puzzle or classical mazes, we designed a battery-constrained multi-goal grid with recharge stations and obstacles.
  The challenge involves an agent starting at a fixed position, visiting all target cells, managing limited battery capacity, and optionally using recharge stations to restore energy. This domain introduces realistic planning constraints and requires strategic decision-making.
2. Problem Formulation
	•	Grid: 2D grid of size rows × cols (default 10×10).
	•	Start: The agent begins at position (0,0).
	•	Targets: Multiple target cells (n_targets) must be visited.
	•	Recharge Stations: Recharge locations restore battery to maximum.
	•	Obstacles: Randomly placed obstacles prevent movement.
	•	Battery Constraint: Agent has limited moves (max_battery) before needing recharge.
	•	Actions: Move to a neighbor or recharge (if on a station).
  State Representation:
  State(pos, mask, battery) where mask is a bitmask indicating which targets have been visited.
  Goal: Visit all targets while managing battery efficiently.
  This domain is original and non-trivial, introducing constraints beyond classical grid problems.
3. Search Algorithms
   3.1 Uniform-Cost Search (UCS)
	•	Type: Uninformed search.
	•	Implementation:
	•	Expands nodes in increasing order of cumulative cost.
	•	Uses dominance pruning: skips states if a better cost with higher battery was already found.
	•	Properties: Complete and optimal but can be slow due to exhaustive search.
   3.2 A Search*
	•	Type: Informed search.
	•	Heuristic:
	•	h(state) = distance_to_nearest_target + MST(remaining_targets)
	•	MST (minimum spanning tree) uses Manhattan distances.
	•	Admissible and consistent.
	•	Implementation: Similar to UCS, but priority = g + h.
	•	Properties: Reduces node expansion and runtime compared to UCS while maintaining optimality.
4. Experimental Evaluation
  Setup:
	•	6 instances generated with varying target and recharge positions.
	•	Metrics recorded: nodes expanded, runtime (seconds), solution cost, max fringe size.
	•	All experiments executed on Python 3.8+, using matplotlib and pandas.
  4.1 Nodes Expanded
	•	UCS expands more nodes than A* due to uninformed search.
	•	A* effectively reduces search space with heuristic guidance.
  4.2 Runtime
	•	UCS runtime is higher for all instances.
	•	A* runtime is consistently lower, demonstrating the efficiency of informed search.
  4.3 Solution Cost
	•	Both UCS and A* find the same minimal solution cost for all solvable instances.
	•	Confirms correctness and optimality of the algorithms.
5. Analysis & Discussion
	•	Trade-offs:
	•	UCS guarantees optimality but at higher computational cost.
	•	A* reduces nodes expanded and runtime while maintaining optimality using the MST-based heuristic.
	•	Domain Design:
	•	Recharge stations and battery constraints force strategic planning.
	•	Random obstacles make each instance unique, avoiding trivial solutions.
	•	Code Quality:
	•	Modular functions for instance generation, search algorithms, evaluation, and plotting.
	•	Fully reproducible: random seed ensures repeatable experiments.
6. Conclusion
  This assignment demonstrates the creation of an original multi-goal grid puzzle domain and the implementation of two search algorithms. Comparative evaluation shows A* is more efficient while UCS guarantees optimality. The results illustrate trade-offs between completeness, runtime, and nodes expanded.
