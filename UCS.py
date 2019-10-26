'''
UCS.py

An implementation of the Uniform Cost Search.

Intended USAGE:
 python3 UCS.py FranceWithCosts
'''
import sys
from PriorityQueue import *

VERBOSE = True  # Set to True to see progress; but it slows the search.

if sys.argv == [''] or len(sys.argv) < 2:
    try:
        import FranceWithCosts as Problem
    except:
        print("Note that the EightPuzzle formulation will be used in Assignment 3, not Assignment 2")
        print("Try python3 UCS.py FranceWithCosts")

else:
    import importlib
    Problem = importlib.import_module(sys.argv[1])


COUNT = None  # Number of nodes expanded.
MAX_OPEN_LENGTH = None  # How long OPEN ever gets.
SOLUTION_PATH = None  # List of states from initial to goal, along lowest-cost path.
TOTAL_COST = None  # Sum of edge costs along the lowest-cost path.
BACKLINKS = {}  # Predecessor links, used to recover the path.


g = {}  # global hash table to associate g values with states.


def runUCS():
    '''This is an encapsulation of some setup before running
    UCS, plus running it and then printing some stats.'''
    initial_state = Problem.CREATE_INITIAL_STATE()
    print("Initial State:")
    print(initial_state)
    global COUNT, BACKLINKS, MAX_OPEN_LENGTH, SOLUTION_PATH
    COUNT = 0
    BACKLINKS = {}
    MAX_OPEN_LENGTH = 0
    SOLUTION_PATH = UCS(initial_state)
    print(str(COUNT) + " states expanded.")
    print('MAX_OPEN_LENGTH = ' + str(MAX_OPEN_LENGTH))


def UCS(initial_state):
    '''Uniform Cost Search. This is the actual algorithm.'''
    global g, COUNT, BACKLINKS, MAX_OPEN_LENGTH, CLOSED, TOTAL_COST
    CLOSED = []
    BACKLINKS[initial_state] = None
    # The "Step" comments below help relate UCS's implementation to
    # those of Depth-First Search and Breadth-First Search.

    # STEP 1a. Put the start state on a priority queue called OPEN
    OPEN = PriorityQueue()
    OPEN.insert(initial_state, 0)
    # STEP 1b. Assign g=0 to the start state.
    g[initial_state] = 0.0
    cost_so_far = g[initial_state]

    # STEP 2. If OPEN is empty, output “DONE” and stop.
    #   while False: # ***STUDENTS CHANGE THIS CONDITION***
    while OPEN != []:
        # LEAVE THE FOLLOWING CODE IN PLACE TO INSTRUMENT AND/OR DEBUG YOUR IMPLEMENTATION
        if VERBOSE: report(OPEN, CLOSED, COUNT)
        if len(OPEN) > MAX_OPEN_LENGTH: MAX_OPEN_LENGTH = len(OPEN)

        # STEP 3. Select the state on OPEN having lowest priority value and call it S.
        #         Delete S from OPEN.
        #         Put S on CLOSED.
        #         If S is a goal state, output its description
        (S, P) = OPEN.delete_min()
        cost_so_far += P
        g[S] = cost_so_far
        # print(f"currently at {S.name} ")
        # print(f"cost from {initial_state.name} to {S.name} = {cost_so_far}")
        # print("In Step 3, returned from OPEN.delete_min with results (S,P)= \n", (str(S), P))

        # print("BACKLINKS = ")
        # for state in BACKLINKS.keys():
        #   print("parent = " + str(BACKLINKS.get(state)) + ", child = " + str(state))

        CLOSED.append(S)

        if Problem.GOAL_TEST(S):
            print("\n" + Problem.GOAL_MESSAGE_FUNCTION(S) + "\n")

            # handling backtracing
            SOLUTION_PATH = [str(s) for s in backtrace(S)]  # should return a list
            print(f"Solution path = {SOLUTION_PATH}")
            print('Length of solution path found: ' + str(len(SOLUTION_PATH) - 1) + ' edges')

            # handling total cost
            TOTAL_COST = g[S]
            print("TOTAL_COST = " + str(TOTAL_COST))

            return
        COUNT += 1
        print("EXPANDING = ", S.name)

        # STEP 4. Generate each successor of S
        #         and if it is already on CLOSED, delete the new instance.
        successors = []
        for op in Problem.OPERATORS:
            if op.precond(S):
                new_state = op.state_transf(S)
                priority = cost_so_far + new_state.edge_distance(S)

                # if successor is not on CLOSED and OPEN, add to successors
                if not (new_state in CLOSED) and not (new_state in OPEN):
                    successors.append((new_state, priority))
                    BACKLINKS[new_state] = S
                else:
                    if new_state in OPEN:
                        if OPEN[new_state] > priority:
                            OPEN.__delitem__(new_state)
                            OPEN.insert(new_state, priority)
                            # print(f"PREVIOUS BACKLINK FOR {new_state} = {BACKLINKS[new_state].name}")
                            BACKLINKS[new_state] = S
                            # print(f"UPDATING BACKLINK FOR {new_state} = {BACKLINKS[new_state].name}")
                            # print(f"UPDATING PRIORITY FOR {new_state} = {OPEN[new_state]}")
                    elif new_state in CLOSED:
                            # If sj occurs on CLOSED, and its new distance is smaller than its old distance,
                            # then it is taken off of CLOSED, put back on OPEN, but with the new, smaller distance.
                            for item in CLOSED:
                                if item == new_state:
                                    old_distance = g[item]
                                    if old_distance > priority:
                                        CLOSED.remove(item)
                                        # print(f"STATE {new_state} IS ON CLOSED.")
                                        # print(f"OLD DISTANCE = {old_distance} vs NEW DISTANCE = {new_distance}")
                                        # print(f"CURRENT CLOSED = {str(CLOSED)}")
                                        OPEN.insert(new_state, priority)
                                        BACKLINKS[new_state] = S

        for (s, p) in successors:
            OPEN.insert(s, p)

        print_state_queue("OPEN", OPEN)
        # print("CLOSED: [ ", end="")
        # for state in CLOSED:
        #     print(state.name, end=" ")
        # print("]")
        cost_so_far -= P  # reset cost_so_far to current shortest path
    # STEP 6. Go to Step 2.
    return None  # No more states on OPEN, and no goal reached.


def print_state_queue(name, q):
    # print(name+" is now: ",end='')
    print(str(q))


def backtrace(S):
    global BACKLINKS
    path = []
    while S:
        path.append(S)
        S = BACKLINKS[S]
    path.reverse()
    # print("Solution path: ")
    # for s in path:
    # print(s)
    return path


def report(open, closed, count):
    print("len(OPEN)=" + str(len(open)), end='; ')
    print("len(CLOSED)=" + str(len(closed)), end='; ')
    print("COUNT = " + str(count))
    print("\n")


if __name__ == '__main__':
    runUCS()
