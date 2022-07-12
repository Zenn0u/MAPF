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
