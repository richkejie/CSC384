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

    '''
    Explanation of heuristic:
    My alternate heuristic has three steps:

    1)  First, the heuristic checks for deadlock states (i.e., states where a goal configuration/state cannot be reached).
        In particular, the heuristic will look at boxes that have not been stored and if:
        (a) the unstored box is on an edge (i.e., next to the wall) and there is not an available storage on that wall
        (b) the unstored box has two non-aligned adjacent squares that are either obstacles or walls
        (c) the unstored box is in contact with another box and there are obstacles or walls around such that neither
            box is able to be moved
        If any of these conditions are true, the heuristic returns math.inf.
        Please see deadlock_check for some visual representations of these deadlock states.
        In addition, if condition (c) is not true, it will also count the number of neighbouring boxes for each box and
        add this to the final heuristic value.
    
    2)  Each unstored box is assigned to the nearest available storage and the Manhattan distance between the box and that
        storage is computed. This distance is then increased by the number of obstacles between the box and the storage.
        Once a box is assigned to a storage, that storage cannot be used for the next box. So in the end, each box will
        either be already stored or assigned to their own unique storage spot. The sum of these distances is added to the
        final heuristic value.

    3)  Each unstored box is assigned to its nearest robot and the Manhattan distance between the box and that robot is
        computed. Similar to step 2, this distance is increased by the number of obstacles between the box and the robot. 
        Multiple boxes can be assigned to the same robot. The sum of these distances is added to the final heuristic value.

    So the final heuristic value (if state is not dead) will be: 
        (sum of number of neighbour boxes for each unstored box) +
        (sum of distances between boxes and storage spots) +
        (sum of distances between boxes and robots)
    '''
    dead, result = deadlock_check(state)
    if dead:
        return math.inf

    storages_left = [storage for storage in state.storage if (storage not in state.boxes)]
    for box in state.boxes:
        if box not in state.storage:
            min_dist = math.inf
            current_used_storage = None
            for storage in storages_left:
                dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
                if dist < min_dist:
                    current_used_storage = storage
                    min_dist = dist
            storages_left.remove(current_used_storage)
            result += min_dist + num_objects_within(box, current_used_storage, state.obstacles)

    for box in state.boxes:
        if box not in state.storage:
            min_dist = math.inf
            for robot in state.robots:
                dist = abs(robot[0]-box[0]) + abs(robot[1]-box[1])
                min_dist = min(dist, min_dist)
            result += min_dist + num_objects_within(robot, box, state.obstacles)

    return result

def num_objects_within(A, B, objects):
    '''Returns the number of objects (obstacles) within the grid formed by point A and B.'''
    result = 0
    (left, right) = (min(A[0], B[0]), max(A[0], B[0]))
    (upper, lower) = (min(A[1], B[1]), max(A[1], B[1]))
    for object in objects:
        if (left <= object[0]) and (object[0] <= right) and (upper <= object[1]) and (object[1] <= lower):
            result += 1
    return result

def deadlock_check(state):
    walls = [(x, y) for x in [-1, state.width] for y in range(state.height)]
    walls.extend([(x, y) for x in range(state.width) for y in [-1, state.height]])

    obstacles_and_walls = state.obstacles.union(frozenset(walls))

    free_storages_x = set()
    free_storages_y = set()
    for storage in state.storage:
        if storage not in state.boxes:
            free_storages_x.add(storage[0])
            free_storages_y.add(storage[1])

    result = 0
    for box in [box for box in state.boxes if box not in state.storage]:
        x,y = box
        up = UP.move(box)
        down = DOWN.move(box)
        left = LEFT.move(box)
        right = RIGHT.move(box)

        # check if on edge and no storage available on edge:
        if (x == 0 or x == state.width-1) and x not in free_storages_x:
            return True, 0
        elif (y == 0 or y == state.height-1) and y not in free_storages_y:
            return True, 0

        '''If a box is cornered as below, state is dead:
             [obs]
        [obs][box]

        [obs][box]
             [obs]
        
        [obs]
        [box][obs]

        [box][obs]
        [obs]
        '''
        if left in obstacles_and_walls and (up in obstacles_and_walls or down in obstacles_and_walls):
            return True, 0
        elif right in obstacles_and_walls and (up in obstacles_and_walls or down in obstacles_and_walls):
            return True, 0
        
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
        neighbour_boxes = set((up, down, left, right)).intersection(state.boxes)
        for neighbour_box in neighbour_boxes:
            if neighbour_box[0] == x:
                # boxes aligned vertically
                if (left in obstacles_and_walls or right in obstacles_and_walls):
                    if (LEFT.move(neighbour_box) in obstacles_and_walls or RIGHT.move(neighbour_box) in obstacles_and_walls):
                        return True, 0
            elif neighbour_box[1] == y:
                # boxes aligned horizontally
                if (up in obstacles_and_walls or down in obstacles_and_walls):
                    if (UP.move(neighbour_box) in obstacles_and_walls or DOWN.move(neighbour_box) in obstacles_and_walls):
                        return True, 0

        result += len(neighbour_boxes)

    return False, result

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def heur_manhattan_distance(state):
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

    sum = 0
    boxes = state.boxes
    storage_spots = state.storage

    not_stored = [box for box in boxes if (box not in storage_spots)]
    for box in not_stored:
        min_dist = math.inf
        for storage in state.storage:
            min_dist = min(min_dist, abs(box[0] - storage[0]) + abs(box[1] - storage[1]))
        sum += min_dist

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
    final = se.search(timebound, (math.inf, math.inf, math.inf))
    return final

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of iterative astar algorithm'''

    end_time = os.times()[0] + timebound

    multiplier = 0.5 # can change
    se = SearchEngine('custom', 'full')
    
    result = None, None
    best_cost = math.inf # set no best cost initially
    processing_time = 0.05
    found = False
    time_remaining = end_time - os.times()[0]

    while time_remaining > processing_time:
        se.init_search(initial_state, sokoban_goal_state, heur_fn, (lambda sN: fval_function(sN, weight)))
        final = se.search(time_remaining - processing_time, (math.inf, math.inf, best_cost))
        weight = max(weight*multiplier,1) # decrease weight for next iteration to find better solution
        
        goal, stats = final

        if goal:
            result = final
            best_cost = goal.gval
            found = True
        else:
            if not found:
                return False, stats
            break

        time_remaining = end_time - os.times()[0]

    return result

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of iterative gbfs algorithm'''

    end_time = os.times()[0] + timebound

    se = SearchEngine('best_first', 'full')
    se.init_search(initial_state, sokoban_goal_state, heur_fn)
    
    result = None, None
    best_cost = math.inf # set no best cost initially
    processing_time = 0.05
    found = False
    time_remaining = end_time - os.times()[0]

    while time_remaining > processing_time:
        final = se.search(time_remaining - processing_time, (best_cost, math.inf, math.inf))

        goal, stats = final

        if goal:
            result = final
            best_cost = goal.gval
            found = True
        else:
            if not found:
                return False, stats
            break

        time_remaining = end_time - os.times()[0]
    
    return result
