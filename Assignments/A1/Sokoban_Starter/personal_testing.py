import os  # for time functions
import math  # for infinity
from search import *  # for search engines
from sokoban import sokoban_goal_state, SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

from solution import *

problem_0 = PROBLEMS[1]

problem_0.print_state()
print("Boxes:", problem_0.boxes)
print("Storage spots:", problem_0.storage)
print("Manhattan dist:", heur_manhattan_distance(problem_0))
print("heur_alternate:", heur_alternate(problem_0))
print("\n")

successors = problem_0.successors()
for s in successors:
    s.print_state()
    print("Boxes:", s.boxes)
    print("Storage spots:", s.storage)
    print("Manhattan dist:", heur_manhattan_distance(s))
    print("heur_alternate:", heur_alternate(s))
    print("\n")

# print(problem_0.robots)
# print(problem_0.boxes)
# print(problem_0.storage)

# print("Robot 0:", problem_0.robots[0])

# # frozenset not subscriptable but is iterable
# for i, box in enumerate(problem_0.boxes):
#     print(f"Box {i}: {box}")
#     print((0,0) in problem_0.boxes)

# for i, storage_spot in enumerate(problem_0.storage):
#     print(f"Storage spot {i}: {storage_spot}")
#     print(storage_spot in problem_0.storage)

# boxes = problem_0.boxes
# storage_spots = problem_0.storage

# new = [box for box in boxes if (box not in storage_spots)]
# print(new)

# print("\n\n")

# problem_1 = PROBLEMS[1]
# problem_1.print_state()
# print(problem_1.hashable_state())

