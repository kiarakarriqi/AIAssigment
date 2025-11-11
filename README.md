Battery-Constrained Multi-Goal Grid

1. Introduction
	The objective of this assignment was to design a custom search problem and implement two search strategies to solve it: one uninformed and one informed. Instead of using standard domains such as the 8-puzzle or classical mazes, I developed a battery-constrained multi-goal grid environment. In this domain, an agent must visit multiple target locations on a grid while managing limited battery energy and optionally recharging at specific stations. This introduces realistic planning constraints and requires strategic optimization rather than simple pathfinding.
2. Problem Formulation
	The environment is represented as a 2D grid. The agent begins at a fixed starting position and must visit all designated target cells. However, the agent has limited battery capacity and can only take a certain number of moves before needing to recharge. Recharge stations located on the grid restore the battery to full capacity. Additionally, obstacles placed on the grid prevent traversal through certain cells.
	Environment Components:
Component           Description
Grid                2D grid (e.g., 10×10)
Start Position      (0, 0)
Targets             Set of cells that must all be visited
Recharge Stations   Restore battery to full capacity
Obstacles           Cells that cannot be traversed
Battery             Maximum step capacity before recharging is required
Actions             Move Up/Down/Left/Right, or Recharge (if on station)

State Representation:
State(position, visited_targets_mask, battery_remaining)
The goal is to reach a state where all targets have been visited without running out of battery.
This domain is non-trivial because it requires planning under resource constraints, unlike classical pathfinding problems.

3. Search Algorithms
   3.1 Uniform-Cost Search (UCS)
UCS is used as the uninformed search strategy. It expands states in order of increasing path cost. A dominance pruning rule is applied so that states are skipped if the same position and visited-target set were reached previously with a better cost and higher remaining battery. UCS is complete and optimal but can be computationally expensive due to exploring a large state space.
  3.2 A* Search
A* is used as the informed search strategy. The heuristic function is defined as:

h(state) = \text{distance to nearest unvisited target} + \text{MST cost over remaining targets}

The MST (Minimum Spanning Tree) uses Manhattan distances, making the heuristic admissible and consistent. A* expands states based on the priority g + h, guiding the search toward promising regions of the grid. It maintains optimality but greatly reduces the number of expanded nodes compared to UCS.
4. Experimental Evaluation
  Experiments were performed on six randomly generated grid instances with varying target and recharge placements. The following performance metrics were recorded: number of nodes expanded, runtime, solution cost, and maximum fringe size. All tests were performed using Python with matplotlib and pandas for data analysis.

Results:
	•	Nodes Expanded: UCS consistently expanded more nodes than A*, demonstrating A*’s advantage in guided search.
	•	Runtime: UCS required more time to complete each problem. A* showed significantly faster execution due to heuristic direction.
	•	Solution Cost: Both UCS and A* always found the same minimal-cost solution, verifying correctness and optimality.
	
5. Analysis & Discussion
	UCS guarantees optimality but is computationally expensive due to exploring many states before finding the solution. A*, using the MST-based heuristic, reduces both the number of expanded nodes and runtime while maintaining optimality. The domain design, including battery constraints and recharge points, forces strategic planning and prevents trivial greedy navigation. The use of random obstacles and target placement ensures variation and prevents overfitting to fixed patterns. The codebase was structured modularly, and controlled random seeds provided reproducible experiments.
6. Conclusion
 This assignment successfully demonstrates the design of a novel search problem and the implementation of two search algorithms to solve it. The evaluation showed that while UCS is complete and optimal, it is inefficient in both time and space. A* achieves the same solution with significantly fewer node expansions and lower runtime, showcasing the effectiveness of informed search in constrained environments. The experiment highlights the importance of heuristic design in improving planning efficiency.

 References:
	1.	Russell, S., & Norvig, P. (2020). Artificial Intelligence: A Modern Approach (4th Edition). Pearson.
	2.	Cormen, T., Leiserson, C., Rivest, R., & Stein, C. (2009). Introduction to Algorithms (3rd Edition). MIT Press.
