#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

# SOKOBAN HEURISTICS
def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    # EXPLAIN YOUR HEURISTIC IN THE COMMENTS. Please leave this function (and your explanation) at the top of your solution file, to facilitate marking.


    '''EXPLAIN!!!'''

    walls = []
    for x in range(state.width):
        walls.append((x,-1))
        walls.append((x,state.height))
    for y in range(state.height):
        walls.append((-1,y))
        walls.append((state.width,y))
    obstacles = state.obstacles.union(frozenset(walls)).union(frozenset(state.boxes))

    sub_obstacles = state.obstacles.union(frozenset(walls)) # used for dead_cuz_multiple_boxes

    not_stored = [box for box in state.boxes if (box not in state.storage)]
    for box in not_stored:
        
        if in_corner(box, obstacles):
            return float('inf')
        elif along_edge_without_storage(box, state):
            return float('inf')
        # elif dead_cuz_multiple_boxes(box, state, sub_obstacles):
        #     return float('inf')
    
    result = 0

    # having the *2 and then -1 resulted in 18/22 passed!!! -- don't delete this, just comment out and make new copy!!!
    # for box in not_stored:
    #     closest_dist = float('inf')
    #     for robot in state.robots:
    #         man_dist = abs(robot[0]-box[0]) + abs(robot[1]-box[1])
    #         closest_dist = min(man_dist + num_obstacles_between(robot, box, state)*2, closest_dist)
    #     result += closest_dist

    #     closest_dist = float('inf')
    #     for storage in state.storage:
    #         man_dist = abs(storage[0]-box[0]) + abs(storage[1]-box[1])
    #         closest_dist = min(man_dist + num_obstacles_between(box, storage, state)-1, closest_dist)
    #     result += closest_dist

    # new weights --> 19/22 passed!!!
    for box in not_stored:
        closest_dist = float('inf')
        for robot in state.robots:
            man_dist = abs(robot[0]-box[0]) + abs(robot[1]-box[1])
            closest_dist = min(man_dist + num_obstacles_between(robot, box, state)*2, closest_dist)
        result += closest_dist

        closest_dist = float('inf')
        for storage in state.storage:
            man_dist = abs(storage[0]-box[0]) + abs(storage[1]-box[1])
            closest_dist = min(man_dist + num_obstacles_between(box, storage, state)-1.2, closest_dist)
        result += closest_dist   
    
    return result

    # return heur_manhattan_distance(state)


    # return 0  # CHANGE THIS

def num_obstacles_between(A, B, state):

    '''Returns the number of obstacles within the grid formed by point A and B.'''

    result = 0
    (left, right) = (min(A[0], B[0]), max(A[0], B[0]))
    (upper, lower) = (min(A[1], B[1]), max(A[1], B[1]))
    for obstacle in state.obstacles:
        if left < obstacle[0] < right and upper < obstacle[0] < lower:
            result += 1
    return result

# Functions that check for dead states
def in_corner(box, obstacles):
    '''Returns True if two adjacent sides of the box is either an obstacle or a wall. False otherwise.'''

    # combine obstacles and walls

    (x, y) = box
    # check top left of box
    if (x-1,y) in obstacles and (x,y-1) in obstacles:
        return True
    # check top right of box
    elif (x+1,y) in obstacles and (x,y-1) in obstacles:
        return True
    # check bottom right of box
    elif (x+1,y) in obstacles and (x,y+1) in obstacles:
        return True
    # check bottom left of box
    elif (x-1,y) in obstacles and (x,y+1) in obstacles:
        return True
    
    return False

def along_edge_without_storage(box, state):
    '''Returns True if box is along a wall without a storage spot. False otherwise'''

    (x, y) = box
    # Check if box is on a wall
    if (0 < x < state.width-1) and (0 < y < state.height-1):
        return False # box is not on wall
    
    x_coords = []
    y_coords = []

    # available_spots = [storage for storage in state.storage if (storage not in state.boxes)]:

    for spot in [storage for storage in state.storage if (storage not in state.boxes)]:
        x_coords.append(spot[0])
        y_coords.append(spot[1])
    if (x == 0 or x == state.width-1) and x not in x_coords:
        return True
    elif (y == 0 or y == state.height-1) and y not in y_coords:
        return True
 
    return False

def dead_cuz_multiple_boxes(box, state, obstacles):

    (x, y) = box
    up = (x, y + 1)
    down = (x, y - 1)
    left = (x - 1, y)
    right = (x + 1, y)
    surround_boxes = set((up, down, left, right)).union(state.boxes)
    if len(surround_boxes) == 0:
        return False
    
    for neighbour_box in surround_boxes:
        x1 = neighbour_box[0]
        y1 = neighbour_box[1]
        if x1 == x:
            # same column
            if (((x+1,y),(x1+1,y1)) in obstacles) or (((x-1,y),(x1+1,y1)) in obstacles) or (((x-1,y),(x1-1,y1)) in obstacles) or (((x+1,y),(x1-1,y1)) in obstacles):
                return True
        elif y1 == y:
            # same row
            if (((x,y+1),(x1,y1+1)) in obstacles) or (((x,y-1),(x1,y1+1)) in obstacles) or (((x,y-1),(x1,y1-1)) in obstacles) or (((x,y+1),(x1,y1-1)) in obstacles):
                return True

    return False



def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):

    # DONE!

    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    '''
    Pseudocode
    # ignoring obstacles in calculation
    # assume many boxes can be stored at same storage spot (not actually legal)
    # assume 'nearest' implies shortest Euclidean distance
    for each box that is not stored:
        find storage spot with shortest Euclidean distance to the box
        calculate Manhattan distance between box and the storage spot
    return sum of Manhattan distances calculated
    '''

    sum = 0
    boxes = state.boxes
    storage_spots = state.storage

    not_stored = [box for box in boxes if (box not in storage_spots)]
    for box in not_stored:
        # print("Checking box:", box)
        euclid_dists = []
        manhat_dists = []
        for spot in storage_spots:
            euclid_dists.append( ((box[0]-spot[0])**2 + (box[1]-spot[1])**2)**(1/2) )
            manhat_dists.append( abs(box[0]-spot[0]) + abs(box[1]-spot[1]) )

        # print("Euclid dists:", euclid_dists)
        # print("Manhattan dists:", manhat_dists)
        # print("Selected Manhattan dist:", manhat_dists[euclid_dists.index(min(euclid_dists))])
        # print("\n")

        sum += manhat_dists[euclid_dists.index(min(euclid_dists))]

    return sum
    # return 0  # CHANGE THIS

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval

    # return 0 #CHANGE THIS

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    # IMPLEMENT    
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''


    end_time = os.times()[0] + timebound
    time_remaining = timebound

    multiplier = 0.6 # can change

    se = SearchEngine('custom', 'full')
    # custom search strategy --> need to specify fval function
    # full --> full cycle checking
    
    result = None, None
    best_cost = float('inf') # set no best cost initially
    time_remaining = end_time - os.times()[0]

    while time_remaining > 0:
        se.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
        final = se.search(time_remaining - 0.1, (float('inf'), float('inf'), best_cost))
        weight = weight*multiplier # decrease weight for next iteration to find better solution
        time_remaining = end_time - os.times()[0]
        
        goal, stats = final

        if goal:
            result = final
            best_cost = goal.gval + heur_fn(goal) - 1
        else:
            break
    
    return result



    # return None, None  # CHANGE THIS

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    result = weighted_astar(initial_state, heur_fn, weight, timebound)
    return result
    # return None, None #CHANGE THIS

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''

    end_time = os.times()[0] + timebound
    time_remaining = timebound

    se = SearchEngine('best_first', 'full')
    # custom search strategy --> need to specify fval function
    # full --> full cycle checking
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    
    result = None, None
    best_cost = float('inf') # set no best cost initially
    time_remaining = end_time - os.times()[0]

    '''costbound is defined as a list of three values. costbound[0] is used to prune
    states based on their g-values; any state with a g-value higher than costbound[0] will not be
    expanded. costbound[1] is used to prune states based on their h-values;
    any state with an hvalue higher than costbound[1] will not be expanded.
    Finally, costbound[2] is used to prune states based on their f-values;
    any state with an f-value higher than costbound[2] will not be expanded.'''
    # costbound = (float('inf'), float('inf'), float('inf')) # initially, don't set any costbound
    # final_state = se.search(timebound, costbound)

    while time_remaining > 0:
        final = se.search(time_remaining - 0.1, (best_cost, float('inf'), float('inf')))
        time_remaining = end_time - os.times()[0]

        goal, stats = final

        if goal:
            result = final
            # print("found a goal:", goal)
            # print("gval:", goal.gval)
            # best_cost = goal.gval + heur_fn(goal) - 1
            best_cost = goal.gval  - 1
        else:
            break
    
    return result

    return None, None #CHANGE THIS



