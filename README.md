# MAPF
Multi Agent Path Finding

0 Task 0: Preparing for the Project
0.1 Installing Python 3
This project requires a Python 3 installation with the numpy and matplotlib packages. On Ubuntu
Linux, download python by using:
sudo apt install python3 python3-numpy python3-matplotlib
On Mac OS X, download Anaconda 2019.03 with Python 3.7 from https://www.anaconda.com/
distribution/#download-section and follow the installer. You can verify your installation by
using:
python3 --version
It may be necessary to use pythonw instead depending on the particular version of Mac OS X.
On Windows, download Anaconda 2019.03 with Python 3.7 from https://www.anaconda.com/
distribution/#download-section.
On Ubuntu Linux and Mac OS X, use python3 to run python. On Windows, use python instead.
You can use a plain text editor for the project. If you would like to use an IDE, we recommend that
you download PyCharm from https://www.jetbrains.com/pycharm/. The free community edition
suces fully, but you can get the professional edition for free as well, see https://www.jetbrains.
com/student/ for details.
0.2 Installing the MAPF Software
Download the archive with the provided MAPF software and extract it on your computer.
0.3 Learning about MAPF
Read the provided textbook-style overview of MAPF.
0.4 Understanding Independent Planning
Execute the independent MAPF solver by using:
python run_experiments.py --instance instances/exp0.txt --solver Independent
If you are successful, you should see an animation:
The independent MAPF solver plans for all agents independently. Their paths do not collide with
the environment but are allowed to collide with the paths of the other agents. Thus, there is a
2
collision when the blue agent 1 stays at its goal cell while the green agent 0 moves on top of it. In
your animation, both agents turn red when this happens, and a warning is printed on the terminal
notifying you about the details of the collision.
Try to understand the independent MAPF solver in independent.py. The rst part denes the
class IndependentSolver and its constructor:
class IndependentSolver(object):
def __init__(self, my_map, starts, goals):
# some parts are omitted here for brevity
# compute heuristic values for the A* search
self.heuristics = []
for goal in self.goals:
self.heuristics.append(compute_heuristics(my_map, goal))
The function compute_heuristics receives as input the representation of the environment and the
goal cell of the agent and computes a look-up table with heuristic values (or, synonymously, h-values)
for the A* search that nds a path for the agent, by executing a Dijkstra search starting at the goal
cell.
The second part performs one A* search per agent:
def find_solution(self):
for i in range(self.num_of_agents): # Find path for each agent
path = a_star(self.my_map, self.starts[i], self.goals[i],
self.heuristics[i], i, [])↪→
if path is None:
raise BaseException('No solutions')
result.append(path)
return result
The function a_star receives as input the representation of the environment, the start cell of the
agent, the goal cell of the agent, the heuristic values computed in the constructor, the unique agent
id of the agent, and a list of constraints and performs an A* search to nd a path for the agent.
The independent MAPF solver does not use constraints.
1 Task 1: Implementing Space-Time A* 6/20
You now change the single agent solver to perform a space-time A* search that searches in cell-time
space and returns a shortest path that satises a given set of constraints. Such constraints are
essential for MAPF solvers such as prioritized planning and CBS.
1.1 Searching in the Space-Time Domain (1pt)
The existing A* search in the function a_star in single_agent_planner.py only searches over
cells. Since we want to support temporal constraints, we also need to search over time steps. Use
the following steps to change the search dimension:
1. Your variables root and child are dictionaries with various key/value pairs such as the g-
value, h-value, and cell. Add a new key/value pair for the time step. The time step of the root
node is zero. The time step of each node is one larger than the one of its parent node.
3
2. The variable closed_list contains the processed (that is, expanded) nodes. Currently, this
is a dictionary indexed by cells. Use tuples of (cell, time step) instead.
3. When generating child nodes, do not forget to add a child node where the agent waits in its
current cell instead of moving to a neighbouring cell.
You can test your code by using:
python run_experiments.py --instance instances/exp1.txt --solver Independent
and should observe identical behaviour. Include the output in your report.
1.2 Handling Vertex Constraints (1pt)
We rst consider (negative) vertex constraints, that prohibit a given agent from being in a given cell
at a given time step.
Each constraint is a Python dictionary. The following code creates a (negative) vertex constraint
that prohibits agent 2 from occupying cell (3, 4) at time step 5:
{'agent': 2,
'loc': [(3,4)],
'timestep': 5}
In order to add support for constraints, change the code to check whether the new node satises the
constraints passed to the a_star function and prune it if it does not.
An ecient way to check for constraint violations is to create, in a pre-processing step, a con-
straint table, which indexes the constraints by their time steps. At runtime, a lookup in the
table is used to verify whether a constraint is violated. Example function headers for the
functions build_constraint_table and is_constrained are already provided. You can call
build_constraint_table before generating the root node in the a_star function.
You can test your code by adding a constraint in prioritized.py that prohibits agent 0 from being
at its goal cell (1, 5) at time step 4 and then using:
python run_experiments.py --instance instances/exp1.txt --solver Prioritized
Agent 0 should wait for one time step (but when and where it waits depends on the tie-breaking).
Include the output in your report.
1.3 Adding Edge Constraints (1pt)
We now consider (negative) edge constraints, that prohibit a given agent from moving from a given
cell to another given cell at a given time step.
The following code creates a (negative) edge constraint that prohibits agent 2 from moving from cell
(1, 1) to cell (1, 2) from time step 4 to time step 5:
{'agent': 2,
'loc': [(1,1), (1,2)],
'timestep': 5}
4
Implement constraint handling for edge constraints in the function is_constrained.
You can test your code by adding a constraint in prioritized.py that prohibits agent 1 from
moving from its start cell (1, 2) to the neighbouring cell (1, 3) from time step 0 to time step 1.
1.4 Handling Goal Constraints (1.5pt)
Run your code with a constraint that prohibits agent 0 from being at its goal cell (1, 5) at time step
10. Where is agent 0 at time step 10 in your solution? To make the algorithm work properly, you
might have to change the goal test condition. In your report, explain what changes you made to the
goal test condition. (The solution of both agents could have collisions.)
1.5 Designing Constraints (1.5pt)
Design a set of constraints by hand that allows your algorithm to nd collision-free paths with a
minimal sum of path lengths. Run your code with the set of constraints. Within your report,
document this set of constraints, the solution, and the sum of path lengths.
2 Task 2: Implementing Prioritized Planning (6.5+0.5)/20
The independent MAPF solver nds paths for all agents, simultaneously or one after the other, that
do not collide with the environment but are allowed to collide with the paths of the other agents.
The prioritized MAPF solver nds paths for all agents, one after the other, that do not collide
with the environment or the already planned paths of the other agents. To ensure that the path of
an agent does not collide with the already planned paths of the other agents, the function a_star
receives as input a list of (negative) constraints compiled from their paths.
2.1 Adding Vertex Constraints (1pt)
Add code to prioritized.py that adds all necessary vertex constraints. You need two loops, namely
one to iterate over the path of the current agent and one to add vertex constraints for all future
agents (since constraints apply only to the specied agent). You can test your code by using:
python run_experiments.py --instance instances/exp2_1.txt --solver Prioritized
Now, the blue agent 2 does not stay at its goal cell when it reaches that cell for the rst time:
Unfortunately, there is still a collision because both agents move to the cell of the other agent at
the same time step. We thus need to add (negative) edge constraints as well.
2.2 Adding Edge Constraints (1pt)
Add code to prioritized.py that adds all necessary edge constraints, and test your code as before.
There are no more collisions.
5
