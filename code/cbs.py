import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    n_p1 = len(path1)
    n_p2 = len(path2)
    n = 0
    if n_p1 > n_p2:
        n = n_p1
    else:
        n = n_p2

    for i in range(n):
        # Vertex Collision
        if get_location(path1, i) == get_location(path2, i):
            collision = {"loc":[get_location(path1, i)], "timestep":i}
            return collision
        # Edge Collision
        if i != 0:
            if get_location(path1, i) == get_location(path2, i-1) and get_location(path2, i) == get_location(path1, i-1):
                collision = {"loc":[get_location(path1, i), get_location(path1, i-1)], "timestep":i}
                return collision

    return None


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    
    collisions = list()
    n = len(paths)

    for i in range(n-1):
        for j in range(i+1, n):
            collision = detect_collision(paths[i], paths[j])
            if collision != None:
                collisions.append({"a1":i, "a2":j, "loc":collision["loc"], "timestep":collision["timestep"]})

    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    cons = list()
    loc = collision["loc"]

    # Vertex Collision
    if len(loc) == 1:
        # First Constraint
        cons.append({"agent":collision["a1"], "loc":loc, "timestep":collision["timestep"]})
        # Second Constraint
        cons.append({"agent":collision["a2"], "loc":loc, "timestep":collision["timestep"]})
        return cons

    elif len(loc) == 2:
        # First Constraint
        cons.append({"agent":collision["a1"], "loc":[loc[1], loc[0]], "timestep":collision["timestep"]})
        # Second Constraint
        cons.append({"agent":collision["a2"], "loc":[loc[0], loc[1]], "timestep":collision["timestep"]})
        return cons

    return cons


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    tmp = random.randint(0,1)
    if tmp == 0:
        r_agent = "a1"
    else:
        r_agent = "a2"

    cons = list()
    loc = collision["loc"]

    # Vertex Collision
    if len(loc) == 1:
        # First Constraint
        cons.append({"agent":collision[r_agent], "loc":loc, "timestep":collision["timestep"], "positive":True})
        # Second Constraint
        cons.append({"agent":collision[r_agent], "loc":loc, "timestep":collision["timestep"], "positive":False})
        return cons

    # Edge Collision
    if len(loc) == 2:
        if r_agent == "a1":
            # First Constraint
            cons.append({"agent":collision[r_agent], "loc":[loc[1], loc[0]], "timestep":collision["timestep"], "positive":True})
            # Second Constraint
            cons.append({"agent":collision[r_agent], "loc":[loc[1], loc[0]], "timestep":collision["timestep"], "positive":False})
            return cons
        if r_agent == "a2":
            # First Constraint
            cons.append({"agent":collision[r_agent], "loc":[loc[0], loc[1]], "timestep":collision["timestep"], "positive":True})
            # Second Constraint
            cons.append({"agent":collision[r_agent], "loc":[loc[0], loc[1]], "timestep":collision["timestep"], "positive":False})
            return cons

    return cons


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            p = self.pop_node()
            if len(p["collisions"]) == 0:
                return p["paths"]
            
            loc = p["collisions"]
            collision = loc[0]
            #constraints = standard_splitting(collision)
            constraints = disjoint_splitting(collision)

            for constraint in constraints:
                q = {"cost": 0,
                    "constraints": [],
                    "paths": [],
                    "collisions": []}

                for cons in p["constraints"]:
                    q["constraints"].append(cons)
                q["constraints"].append(constraint)

                for pth in p["paths"]:
                    q["paths"].append(pth)
                
                a = constraint["agent"]

                path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a],
                          a, q["constraints"])
                        
                if len(path) != 0:
                    q["paths"][a] = path

                    ##################
                    # Task 4.3
                    path_vio = list()
                    if constraint["positive"] == True:
                        path_vio = paths_violate_constraint(constraint, q["paths"])
                    
                    for a2 in path_vio:
                        con = {"agent":a2, "loc":constraint["loc"], "timestep":constraint["timestep"], "positive":True}

                        q["constraints"].append(con)
                        n_path = a_star(self.my_map, self.starts[a2], self.goals[a2], self.heuristics[a2],a2, q["constraints"])
                        
                        if n_path is not None:
                            q["paths"][a2] = n_path
                    ##################

                    q["collisions"] = detect_collisions(q["paths"])
                    q["cost"] = get_sum_of_cost(q["paths"])

                    self.push_node(q) 
            
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))