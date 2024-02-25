#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.  

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newly_instantiated_variable=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newly_instaniated_variable is an optional argument.
      if newly_instantiated_variable is not None:
          then newly_instantiated_variable is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newly_instantiated_variable = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.


      PROPAGATOR called with newly_instantiated_variable = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

   '''

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''
    
    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0:
            vals = []
            vars = c.get_scope()
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with 
       only one uninstantiated variable. Remember to keep 
       track of all pruned variable,value pairs and return '''
    #IMPLEMENT

    if newVar != None:
        # if newVar is given, check all constraints with newVar in their scope
        constraints = csp.get_cons_with_var(newVar)
    else:
        # if newVar not given, check all constraints
        constraints = csp.get_all_cons()

    num = 0
    for cons in constraints:
        if cons.get_n_unasgn() == 1:
            num += 1

    # keep track of [(Variable, Value), (Variable, Value) ...]
    # where (Variable, Value) represents Value having been pruned
    # from Variable's domain by prop_FC
    pruned = []

    def check(cons, var):
        '''Check that all values in variable var's current domain satisfy
           the constraint cons. If a value does not, prune it, and record
           it in the list pruned. Return True if DWO does not occur.
           Otherwise, return False immediately.'''
        
        for d in var.cur_domain():
            # assign d to var
            var.assign(d)
            # check if this new assignment satisfies cons
            asgns = []
            for v in cons.get_scope():
                asgns.append(v.get_assigned_value())
            # if it does not, prune d from var's domain
            # and record in list pruned
            if cons.check(asgns) == False:
                var.prune_value(d)
                pruned.append((var, d))
            # unassign d from var, and check next value in var's domain
            var.unassign()
        
        # if DWO occurred, return False
        if var.cur_domain_size() == 0:
            return False
        # otherwise, return True
        return True

    # we are only asked to forward check unary constraints
    # unary constraints are those with a "scope of just one variable"
    #   this includes constraints with a multi-variable scope but only
    #   one variable in the scope is unassigned
    # this is not quite the same as the FC algorithm from lecture
    for cons in constraints:
        if cons.get_n_unasgn() == 1:
            var = cons.get_unasgn_vars()[0] 
                # this is the singular variable that is not assigned
            if check(cons, var) == False:
                # DWO occurred, so there is no supporting assignment
                # bt_search must backtrack
                return False, pruned
        
    # pruning is finished and no DWO occurred
    return True, pruned
    

def prop_FI(csp, newVar=None):
    '''Do full inference. If newVar is None we initialize the queue
       with all variables.'''
    #IMPLEMENT
    
    if newVar != None:
        # if newVar is given, initialize queue with just newVar
        queue = [newVar]
    else:
        # if newVar is not given, initialize queue with all variables
        queue = csp.get_all_vars()

    # keep track of [(Variable, Value), (Variable, Value) ...]
    # where (Variable, Value) represents Value having been pruned
    # from Variable's domain by prop_FI
    pruned = []

    # run full inferencing:
    while len(queue) > 0:
        # pop first variable in queue
        w = queue.pop(0)
        # iterate over constraints with w in their scope
        for cons in csp.get_cons_with_var(w):
            # iterate over variables in scope of cons, excluding w
            for v in cons.get_scope():
                if v == w:
                    continue
                # make copy of v's current domain
                S = v.cur_domain()
                # iterate over v's current domain
                for d in v.cur_domain():
                    # if a supporting assignment cannot be found for v = d
                    # prune d from v's current domain and record in list pruned
                    # if DWO occurred, return False
                    if not cons.has_support(v, d):
                        v.prune_value(d)
                        pruned.append((v, d))
                        # if DWO occurred, return False
                        if v.cur_domain_size() == 0:
                            queue.clear()
                            return False, pruned
                # if DWO did not occur, check if any values were
                # pruned from v's domain, if so, add v back to queue
                if v.cur_domain() != S:
                    if v not in queue:
                        queue.append(v)

    # FI finishes without DWO, return True
    return True, pruned


