#Look for #IMPLEMENT tags in this file.
'''
All encodings need to return a CSP object, and a list of lists of Variable objects 
representing the board. The returned list of lists is used to access the 
solution. 

For example, after these three lines of code

    csp, var_array = caged_csp(board)
    solver = BT(csp)
    solver.bt_search(prop_FC, var_ord)

var_array[0][0].get_assigned_value() should be the correct value in the top left
cell of the FunPuzz puzzle.

The grid-only encodings do not need to encode the cage constraints.

1. binary_ne_grid (worth 10/100 marks)
    - An enconding of a FunPuzz grid (without cage constraints) built using only 
      binary not-equal constraints for both the row and column constraints.

2. nary_ad_grid (worth 10/100 marks)
    - An enconding of a FunPuzz grid (without cage constraints) built using only n-ary 
      all-different constraints for both the row and column constraints. 

3. caged_csp (worth 25/100 marks) 
    - An enconding built using your choice of (1) binary binary not-equal, or (2) 
      n-ary all-different constraints for the grid.
    - Together with FunPuzz cage constraints.

'''
from cspbase import *
import itertools

### helper functions for setting up variables ###
def process(fpuzz_grid):
    '''Return some values associated with fpuzz_grid'''
    n = fpuzz_grid[0][0] # dimension of the board is n by n
    domain = [i for i in range(1, n+1)]
    return n, domain

def create_board(n, domain):
    '''Return a board.'''
    board = []
    for i in range(1, n+1):
        row = []
        for j in range(1, n+1):
            var = Variable("{}{}".format(i,j), domain)
            row.append(var)
        board.append(row)
    return board

def get_variables(board):
    '''Collect the variables from board.'''
    variables = []
    for row in board:
        variables += row
    return variables

def transpose_board(board):
    '''Treat the board as a matrix and transpose it.'''
    n = len(board)
    board_T = [[0 for i in range(n)] for i in range(n)]
    for i in range(n):
        for j in range(n):
            board_T[j][i] = board[i][j]
    return board_T


### helper functions for binary_ne_grid ###
def binary_ne_constraints(board, n, domain, type):
    '''Return binary ne constraints for the rows of board.'''
    tuples = []
    for t in itertools.product(domain, repeat=2):
        if t[0] != t[1]:
            tuples.append(t)

    constraints = []
    for row in board:
        for i in range(n):
            for j in range(i+1, n):
                cons = Constraint("binary_ne_{}_{}:{}!={}".format(type, row, row[i],row[j]), [row[i], row[j]])
                cons.add_satisfying_tuples(tuples)
                constraints.append(cons)
    return constraints


### helper functions for nary_ad_grid ###
def nary_ad_constraints(board, domain, type):
    '''Return nary ad constraints for the rows of board.'''

    tuples = []
    for t in itertools.product(domain, repeat=len(domain)):
        if len(t) == len(set(t)):
            tuples.append(t)

    constraints = []
    for i, row in enumerate(board):
        cons = Constraint("nary_ad_{}({})".format(type, i+1), row)
        cons.add_satisfying_tuples(tuples)
        constraints.append(cons)

    return constraints


### helper functions for caged_csp ###
def tup_satisfies(t, operation, target):

    # +
    if operation == 0:
        if sum(list(t)) == target:
            return True
    
    # -
    elif operation == 1:
        lst = list(t)
        for perm in itertools.permutations(lst):
            res = perm[0]
            for i in range(1,len(perm)):
                res -= perm[i]
            if res == target:
                return True

    # /
    elif operation == 2:
        lst = list(t)
        for perm in itertools.permutations(lst):
            res = perm[0]
            for i in range(1,len(perm)):
                res /= perm[i]
            if res == target:
                return True

    # *
    elif operation == 3:
        res = 1
        for i in range(len(t)):
            res *= t[i]
        if res == target:
            return True
    
    else:
        print("Error, operation {} is not an accepatable operation".format(operation))

    return False

def find_cage_constraints(fpuzz_grid, board, domain):

    constraints = []

    cages = fpuzz_grid[1:]

    # create the cage constraints
    for i, cage in enumerate(cages):

        # if cage list has 2 elements: cell and enforced target
        if len(cage) == 2:
            row, col = cage[0] // 10 - 1, cage[0] % 10 - 1
            cons = Constraint("cage_{}".format(i), [board[row][col]])
            cons.add_satisfying_tuples([tuple([cage[1]])])
            constraints.append(cons)
            continue

        # otherwise, cage list has >= 4 elements
        # the second last element is the target
        # and the last element is the operation:
        #   0=+
        #   1=-
        #   2=/
        #   3=*
        scope = []
        for cell in cage[:-2]:
            row, col = cell // 10 - 1, cell % 10 - 1
            scope.append(board[row][col])

        cons = Constraint("cage_{}".format(i), scope)
        
        # find the satisfying tuples
        operation = cage[-1]
        target = cage[-2]

        tuples = []
        for t in itertools.product(domain, repeat=len(scope)):
            if tup_satisfies(t, operation, target):
                tuples.append(t)
        cons.add_satisfying_tuples(tuples)
        constraints.append(cons)

    return constraints


def binary_ne_grid(fpuzz_grid):
    ##IMPLEMENT
    
    '''Return:
        1. A csp containing variables and constraints
           representing the problem. The constraints 
           are row and column constraints using
           binary not equal constraints.
        2. A list of lists of Variable objects representing
           the board.'''
    
    n, domain = process(fpuzz_grid)
    board = create_board(n, domain)
    
    # variables is a list of the Variable objects to be used for the csp
    variables = get_variables(board) ##IMPLEMENT 

    # row_constraints and col_constraints are each a list of binary_ne Constraint Objects
    # the constraints will have all satisfying tuples added already
    row_constraints = binary_ne_constraints(board, n, domain, "row") ##IMPLEMENT
    col_constraints = binary_ne_constraints(transpose_board(board), n, domain, "col") ##IMPLEMENT
    constraints = row_constraints + col_constraints

    # initialize the csp
    csp = CSP("binary_ne_grid", variables)
    for cons in constraints:
        csp.add_constraint(cons)

    return csp, board
    

def nary_ad_grid(fpuzz_grid):
    ##IMPLEMENT 
    n, domain = process(fpuzz_grid)
    board = create_board(n, domain)

    variables = get_variables(board)

    row_constraints = nary_ad_constraints(board, domain, "row")
    col_constraints = nary_ad_constraints(transpose_board(board), domain, "col")
    constraints = row_constraints + col_constraints

    csp = CSP("nary_ad_grid", variables)
    for cons in constraints:
        csp.add_constraint(cons)

    return csp, board
    

def caged_csp(fpuzz_grid):
    ##IMPLEMENT 
    
    n, domain = process(fpuzz_grid)
    board = create_board(n, domain)

    variables = get_variables(board)

    # use binary ne constaints for rows and columns
    row_constraints = binary_ne_constraints(board, n, domain, "row") ##IMPLEMENT
    col_constraints = binary_ne_constraints(transpose_board(board), n, domain, "col") ##IMPLEMENT
   
    # # use nary ad constraints for rows and columns
    # row_constraints = nary_ad_constraints(board, domain, "row")
    # col_constraints = nary_ad_constraints(transpose_board(board), domain, "col")

    # find cage constraints
    cage_constraints = find_cage_constraints(fpuzz_grid, board, domain) ##IMPLEMENT

    constraints = row_constraints + col_constraints + cage_constraints
    
    csp = CSP("caged_csp", variables)
    for cons in constraints:
        csp.add_constraint(cons)

    return csp, board
    