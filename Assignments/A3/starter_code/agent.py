"""
An AI player for Othello. 
"""

import random
import sys
import time

# You can use the functions in othello_shared to write your AI
from othello_shared import find_lines, get_possible_moves, get_score, play_move

# caching dictionaries
min_cache = {}
max_cache = {}
alphabeta_min_cache = {}
alphabeta_max_cache = {}


def eprint(*args, **kwargs): #you can use this for debugging, as it will print to sterr and not stdout
    print(*args, file=sys.stderr, **kwargs)
    
# Method to compute utility value of terminal state
def compute_utility(board, color):
    #IMPLEMENT
    
    # assume this is called only on terminal states
    # so do not have to check whether state is terminal or not
    final_score = get_score(board)
    if color == 1:
        # 1 is 1st company
        return final_score[0] - final_score[1]
    else:
        # 2 is 2nd company
        return final_score[1] - final_score[0]

    # return 0 #change this!

# Better heuristic value of board
def compute_heuristic(board, color): #not implemented, optional
    #IMPLEMENT
    return 0 #change this!

############ MINIMAX ###############################
def opponent(color):
    if color == 1:
        return 2
    else:
        return 1

def minimax_min_node(board, color, limit, caching = 0):
    #IMPLEMENT (and replace the line below)

    # returns a tuple: ((r,c), util)
    # where (r,c) is the best move with utility util

    if caching:
        if (board, color) in min_cache:
            return min_cache[(board, color)]

    moves = get_possible_moves(board, color)
    if moves == [] or limit == 0:
        # we have reached a terminal state (base case)
        return (None, compute_utility(board, opponent(color)))

    min_util = float("inf")
    best_move = moves[0]

    for i in range(len(moves)):
        util = minimax_max_node(play_move(board, color, moves[i][0], moves[i][1]), opponent(color), limit-1, caching)[1]
        if util < min_util:
            best_move = moves[i]
            min_util = util
    
    if caching:
        min_cache[(board, color)] = (best_move, min_util)

    return (best_move, min_util)

    # return ((0,0),0)

def minimax_max_node(board, color, limit, caching = 0): #returns highest possible utility
    #IMPLEMENT (and replace the line below)

    # returns a tuple: ((r,c), util)
    # where (r,c) is the best move with utility util

    if caching:
        if (board, color) in max_cache:
            return max_cache[(board, color)]

    moves = get_possible_moves(board, color)
    if moves == [] or limit == 0:
        # we have reached a terminal state (base case)
        return (None, compute_utility(board, color))

    max_util = float("-inf")
    best_move = moves[0]

    for i in range(len(moves)):
        util = minimax_min_node(play_move(board, color, moves[i][0], moves[i][1]), opponent(color), limit-1, caching)[1]
        if util > max_util:
            best_move = moves[i]
            max_util = util

    if caching:
        max_cache[(board, color)] = (best_move, max_util)
    
    return (best_move, max_util)

    # return ((0,0),0)

def select_move_minimax(board, color, limit, caching = 0):
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    """
    #IMPLEMENT (and replace the line below)

    # it is our turn, so call max node first
    return minimax_max_node(board, color, limit, caching)[0]

    # return (0,0) #change this!

############ ALPHA-BETA PRUNING #####################
def alphabeta_min_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    #IMPLEMENT (and replace the line below)

    # returns a tuple: ((r,c), util)
    # where (r,c) is the best move with utility util

    if caching:
        if (board, color, alpha, beta) in alphabeta_min_cache:
            return alphabeta_min_cache[(board, color, alpha, beta)]

    moves = get_possible_moves(board, color)
    if moves == [] or limit == 0:
        # we have reached a terminal state (base case)
        return (None, compute_utility(board, opponent(color)))
    
    min_util = float("inf")
    best_move = None

    children_boards = []
    for i in range(len(moves)):
        children_boards.append(play_move(board, color, moves[i][0], moves[i][1]))

    if ordering:
        children_boards.sort(key=lambda child_board: compute_utility(child_board, opponent(color)))
        # check best move for min first: the move that gives the lowest utility

    for i in range(len(children_boards)):
        util = alphabeta_max_node(children_boards[i], opponent(color), alpha, beta, limit-1, caching, ordering)[1]
        if util < min_util:
            best_move = moves[i]
            min_util = util
        beta = min(beta, min_util)
        if alpha >= beta:
            break
    
    if caching:
        alphabeta_min_cache[(board, color, alpha, beta)] = (best_move, min_util)

    return (best_move, min_util)

    # return ((0,0),0) #change this!

def alphabeta_max_node(board, color, alpha, beta, limit, caching = 0, ordering = 0):
    #IMPLEMENT (and replace the line below)

    # returns a tuple: ((r,c), util)
    # where (r,c) is the best move with utility util

    if caching:
        if (board, color, alpha, beta) in alphabeta_max_cache:
            return alphabeta_max_cache[(board, color, alpha, beta)]

    moves = get_possible_moves(board, color)
    if moves == [] or limit == 0:
        # we have reached a terminal state (base case)
        return (None, compute_utility(board, color))
    
    max_util = float("-inf")
    best_move = None

    children_boards = []
    for i in range(len(moves)):
        children_boards.append(play_move(board, color, moves[i][0], moves[i][1]))

    if ordering:
        children_boards.sort(key=lambda child_board: compute_utility(child_board, color), reverse=True)
        # check best move for max first: the move that gives highest utility

    for i in range(len(children_boards)):
        util = alphabeta_min_node(children_boards[i], opponent(color), alpha, beta, limit-1, caching, ordering)[1]
        if util > max_util:
            best_move = moves[i]
            max_util = util
        alpha = max(alpha, max_util)
        if alpha >= beta:
            break

    if caching:
        alphabeta_max_cache[(board, color, alpha, beta)] = (best_move, max_util)
    
    return (best_move, max_util)

    # return ((0,0),0) #change this!

def select_move_alphabeta(board, color, limit, caching = 0, ordering = 0):
    """
    Given a board and a player color, decide on a move. 
    The return value is a tuple of integers (i,j), where
    i is the column and j is the row on the board.  

    Note that other parameters are accepted by this function:
    If limit is a positive integer, your code should enfoce a depth limit that is equal to the value of the parameter.
    Search only to nodes at a depth-limit equal to the limit.  If nodes at this level are non-terminal return a heuristic 
    value (see compute_utility)
    If caching is ON (i.e. 1), use state caching to reduce the number of state evaluations.
    If caching is OFF (i.e. 0), do NOT use state caching to reduce the number of state evaluations.    
    If ordering is ON (i.e. 1), use node ordering to expedite pruning and reduce the number of state evaluations. 
    If ordering is OFF (i.e. 0), do NOT use node ordering to expedite pruning and reduce the number of state evaluations. 
    """
    #IMPLEMENT (and replace the line below)

    # it is our turn, so call max node first
    # initialize alpha to -inf
    # initialize beta to +inf
    return alphabeta_max_node(board, color, float("-inf"), float("inf"), limit, caching, ordering)[0]

    #return (0,0) #change this!

####################################################
def run_ai():
    """
    This function establishes communication with the game manager.
    It first introduces itself and receives its color.
    Then it repeatedly receives the current score and current board state
    until the game is over.
    """
    print("Othello AI") # First line is the name of this AI
    arguments = input().split(",")
    
    color = int(arguments[0]) #Player color: 1 for dark (goes first), 2 for light. 
    limit = int(arguments[1]) #Depth limit
    minimax = int(arguments[2]) #Minimax or alpha beta
    caching = int(arguments[3]) #Caching 
    ordering = int(arguments[4]) #Node-ordering (for alpha-beta only)

    if (minimax == 1): eprint("Running MINIMAX")
    else: eprint("Running ALPHA-BETA")

    if (caching == 1): eprint("State Caching is ON")
    else: eprint("State Caching is OFF")

    if (ordering == 1): eprint("Node Ordering is ON")
    else: eprint("Node Ordering is OFF")

    if (limit == -1): eprint("Depth Limit is OFF")
    else: eprint("Depth Limit is ", limit)

    if (minimax == 1 and ordering == 1): eprint("Node Ordering should have no impact on Minimax")

    while True: # This is the main loop
        # Read in the current game status, for example:
        # "SCORE 2 2" or "FINAL 33 31" if the game is over.
        # The first number is the score for player 1 (dark), the second for player 2 (light)
        next_input = input()
        status, dark_score_s, light_score_s = next_input.strip().split()
        dark_score = int(dark_score_s)
        light_score = int(light_score_s)

        if status == "FINAL": # Game is over.
            print
        else:
            board = eval(input()) # Read in the input and turn it into a Python
                                  # object. The format is a list of rows. The
                                  # squares in each row are represented by
                                  # 0 : empty square
                                  # 1 : dark disk (player 1)
                                  # 2 : light disk (player 2)

            # Select the move and send it to the manager
            if (minimax == 1): #run this if the minimax flag is given
                movei, movej = select_move_minimax(board, color, limit, caching)
            else: #else run alphabeta
                movei, movej = select_move_alphabeta(board, color, limit, caching, ordering)
            
            print("{} {}".format(movei, movej))

if __name__ == "__main__":
    run_ai()
