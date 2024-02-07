#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

from sokoban import UP, RIGHT, DOWN, LEFT

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
    result = 0

    walls = []
    for x in range(state.width):
        walls.append((x,-1))
        walls.append((x,state.height))
    for y in range(state.height):
        walls.append((-1,y))
        walls.append((state.width,y))

    obstacles = state.obstacles.union(frozenset(walls))

    not_stored = [box for box in state.boxes if (box not in state.storage)]
    for box in not_stored:
        
        if in_corner(box, obstacles): #!!!!
            return float('inf')
        elif along_edge_without_storage(box, state):
            return float('inf')
        dead, num_neighbour_boxes = dead_cuz_multiple_boxes(box, state, obstacles)
        if dead:
            return float('inf')
        else:
            result += 1*num_neighbour_boxes

    used_storages = []
    total_box_storage_cost = 0
    for box in state.boxes:
        min_dist = float('inf')
        current_used_storage = None
        for storage in state.storage:
            if storage not in used_storages:
                dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                if dist < min_dist:
                    current_used_storage = storage
                    min_dist = dist
        used_storages.append(current_used_storage)
        total_box_storage_cost += min_dist

    result += total_box_storage_cost


    '''
    for box in not_stored:
        closest_dist = float('inf')
        for robot in state.robots:
            man_dist = abs(robot[0]-box[0]) + abs(robot[1]-box[1])
            closest_dist = min(man_dist + num_objects_within(robot, box, state.obstacles), closest_dist)
            # print("box:", box, "robot:", robot)
            # print("closest:", closest_dist)
        result += closest_dist

        closest_dist = float('inf')
        for storage in state.storage:
            man_dist = abs(storage[0]-box[0]) + abs(storage[1]-box[1])
            closest_dist = min(man_dist + num_objects_within(box, storage, state.obstacles)*2, closest_dist)
            # print("box:", box, "storage:", storage)
            # print("closest:", closest_dist)
        result += closest_dist 
    ''' 

    '''
    # this gets 19/22 for bfs w/out dead_cuz_multiple_boxes check and in_corner check includes boxes in obstacles
    for box in not_stored:
        closest_dist = float('inf')
        for robot in state.robots:
            man_dist = abs(robot[0]-box[0]) + abs(robot[1]-box[1])
            closest_dist = min(man_dist + num_objects_within(robot, box, state.obstacles)*2, closest_dist)
            # print("box:", box, "robot:", robot)
            # print("closest:", closest_dist)
        result += closest_dist

        closest_dist = float('inf')
        for storage in state.storage:
            man_dist = abs(storage[0]-box[0]) + abs(storage[1]-box[1])
            closest_dist = min(man_dist + num_objects_within(box, storage, state.obstacles)-1.2, closest_dist)
            # print("box:", box, "storage:", storage)
            # print("closest:", closest_dist)
        result += closest_dist
    '''

    '''
    for box in not_stored:
        closest_dist = float('inf')
        for robot in state.robots:
            man_dist = abs(robot[0]-box[0]) + abs(robot[1]-box[1])
            closest_dist = min(man_dist + num_objects_between(robot, box, state.obstacles), closest_dist)
            # print("box:", box, "robot:", robot)
            # print("closest:", closest_dist)
        result += closest_dist

        closest_dist = float('inf')
        for storage in state.storage:
            man_dist = abs(storage[0]-box[0]) + abs(storage[1]-box[1])
            closest_dist = min(man_dist + num_objects_between(box, storage, state.obstacles)*2, closest_dist)
            # print("box:", box, "storage:", storage)
            # print("closest:", closest_dist)
        result += closest_dist  
    ''' 

    return result


''' 
(even) better heuristic ideas:

- find a lowest cost perfect matching bipartite graph between boxes and storage:
    - i.e., find the man_dist+num_obs cost between each pair of (box,storage)
    - treat these costs as edges in a graph, and boxes and storages as vertices
    - reduce graph to perfect matching bipartite graph that has lowest total edge costs
    - boxes in one group, storages in the other group
    - this is the assignment problem --> popular solving algorithm: the Hungarian method


- more deadlock state checks:
    - ...

- 

'''


def num_objects_within(A, B, objects):

    '''Returns the number of objects (obstacles) within the grid formed by point A and B.'''

    result = 0
    (left, right) = (min(A[0], B[0]), max(A[0], B[0]))
    (upper, lower) = (min(A[1], B[1]), max(A[1], B[1]))
    for object in objects:
        if left < object[0] < right and upper < object[0] < lower:
            result += 1
    return result

def num_objects_between(A, B, objects):

    '''Returns the number of objects (obstacles) along the straight line between point A and B'''
    
    if A[0] == B[0] or A[1] == B[1]:
        return num_objects_within(A, B, objects)

    result = 0
    a = B[1] - A[1]
    b = A[0] - B[0]
    c = A[1]*(B[0] - A[0]) - A[0]*(B[1] - A[1])

    (left, right) = (min(A[0], B[0]), max(A[0], B[0]))
    (upper, lower) = (min(A[1], B[1]), max(A[1], B[1]))
    for object in objects:
        if left < object[0] < right and upper < object[0] < lower:
            d = abs(a*object[0] + b*object[1] + c)/(a**2 + b**2)**(1/2)
            if d < 2:
                result += 1
    return result





# Functions that check for dead states
def in_corner(box, obstacles):
    '''Returns True if two adjacent sides of the box is either an obstacle or a wall. False otherwise.'''

    # combine obstacles and walls

    # (x, y) = box
    # check top left of box
    if LEFT.move(box) in obstacles and UP.move(box) in obstacles:
        return True
    # check top right of box
    elif RIGHT.move(box) in obstacles and UP.move(box) in obstacles:
        return True
    # check bottom right of box
    elif RIGHT.move(box) in obstacles and DOWN.move(box) in obstacles:
        return True
    # check bottom left of box
    elif LEFT.move(box) in obstacles and DOWN.move(box) in obstacles:
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

    # obstacles contains original obstacles and walls

    (x, y) = box
    up = UP.move(box)
    down = DOWN.move(box)
    left = LEFT.move(box)
    right = RIGHT.move(box)
    surround_boxes = set((up, down, left, right)).intersection(state.boxes)
    if len(surround_boxes) == 0:
        return (False, 0)
    
    '''If two boxes are next to each other, it is only dead if:
    [obs][box]
    [obs][box]

         [box][obs]
    [obs][box]

    [obs][box]
         [box][obs]
    
    [box][obs]
    [box][obs]

    and same for horizontal alignment of boxes
    '''

    dead = False

    for neighbour_box in surround_boxes:
        if neighbour_box[0] == x:
            # boxes aligned vertically
            if (left in obstacles or right in obstacles) and (LEFT.move(neighbour_box) in obstacles or RIGHT.move(neighbour_box) in obstacles):
                dead = True
        elif neighbour_box[1] == y:
            # boxes aligned horizontally
            if (up in obstacles or down in obstacles) and (UP.move(neighbour_box) in obstacles or DOWN.move(neighbour_box) in obstacles):
                dead = True

    # print("Box:", box)
    # print(dead)
    # print(len(surround_boxes))
    return (dead, len(surround_boxes))

# other helper functions


# other assignment functions

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

    se = SearchEngine('custom', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
    final = se.search(timebound, (float('inf'), float('inf'), float('inf')))
    return final

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    end_time = os.times()[0] + timebound
    time_remaining = timebound

    multiplier = 0.5 # can change

    se = SearchEngine('custom', 'full')
    # custom search strategy --> need to specify fval function
    # full --> full cycle checking
    
    result = None, None
    best_cost = float('inf') # set no best cost initially
    time_remaining = end_time - os.times()[0]

    while time_remaining > 0:
        se.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
        final = se.search(time_remaining - 0.1, (float('inf'), float('inf'), best_cost))
        weight = min(weight*multiplier,1) # decrease weight for next iteration to find better solution
        time_remaining = end_time - os.times()[0]
        
        goal, stats = final

        if goal:
            result = final
            best_cost = goal.gval
        else:
            break
    
    return result

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

    while time_remaining > 0:
        final = se.search(time_remaining - 0.1, (best_cost, float('inf'), float('inf')))
        time_remaining = end_time - os.times()[0]

        goal, stats = final

        if goal:
            result = final
            best_cost = goal.gval
        else:
            break
    
    return result



