import sys

'''
AStar.py

An Implementation of the A* Search algorithm.

Intended USAGE:
 python3 AStar.py FranceWithCosts
'''

VERBOSE = True  # Set to True to see progress; but it slows the search.

if sys.argv == [''] or len(sys.argv) < 2:
    try:
        import EightPuzzleWithManhattan as Problem
    except:
        print("Note that the EightPuzzle formulation will be used in Assignment 3, not Assignment 2")
        print("Try python3 AStar.py FranceWithCosts")

else:
    import importlib
    Problem = importlib.import_module(sys.argv[1])

print("\nWelcome to AStar search!")

COUNT = None  # Number of nodes expanded.
MAX_OPEN_LENGTH = None  # How long OPEN ever gets.
SOLUTION_PATH = None  # List of states from initial to goal, along lowest-cost path.
TOTAL_COST = None  # Sum of edge costs along the lowest-cost path.
BACKLINKS = {}  # Predecessor links, used to recover the path.

# The value g(s) represents the cost along the best path found so far
# from the initial state to state s.
g = {}  # We will use a global hash table to associate g values with states.
h = Problem.h
f = {}

class My_Priority_Queue:
    def __init__(self):
        self.q = []  # Actual data goes in a list.

    def __contains__(self, elt):
        '''If there is a (state, priority) pair on the list
        where state==elt, then return True.'''
        # print("In My_Priority_Queue.__contains__: elt= ", str(elt))
        for pair in self.q:
            if pair[0] == elt: return True
        return False

    def delete_min(self):
        ''' Standard priority-queue dequeuing method.'''
        if self.q == []: return []  # Simpler than raising an exception.
        temp_min_pair = self.q[0]
        temp_min_value = temp_min_pair[1]
        temp_min_position = 0
        for j in range(1, len(self.q)):  # start from index 1 of q bc 0 is temp_min_pair
            if self.q[j][1] < temp_min_value:  # comparing current value with temp_min_value
                temp_min_pair = self.q[j]  # update as necessary
                temp_min_value = temp_min_pair[1]
                temp_min_position = j
        del self.q[temp_min_position]  # once min is found, remove that item from q
        return temp_min_pair  # return the value of temp_min_pair

    def insert(self, state, priority):
        '''We do not keep the list sorted, in this implementation.'''
        # print("calling insert with state, priority: ", state, priority)

        if self[state] != -1:
            print("Error: You're trying to insert an element into a My_Priority_Queue instance,")
            print(" but there is already such an element in the queue.")
            return
        self.q.append((state, priority))

    def __len__(self):
        '''We define length of the priority queue to be the
        length of its list.'''
        return len(self.q)

    def __getitem__(self, state):
        '''This method enables Pythons right-bracket syntax.
        Here, something like  priority_val = my_queue[state]
        becomes possible. Note that the syntax is actually used
        in the insert method above:  self[state] != -1  '''
        for (S, P) in self.q:
            if S == state: return P
        return -1  # This value means not found.

    def __delitem__(self, state):
        '''This method enables Python's del operator to delete
        items from the queue.'''
        # print("In MyPriorityQueue.__delitem__: state is: ", str(state))
        for count, (S, P) in enumerate(self.q):
            if S == state:
                del self.q[count]
                return

    def __str__(self):
        # txt = "My_Priority_Queue: ["
        txt = "OPEN: ["
        for (s, p) in self.q: txt += '(' + str(s) + ',' + str(p) + ') '
        txt += ']'
        return txt


def runAStar():
    '''This is an encapsulation of some setup before running
    AStar, plus running it and then printing some stats.'''
    initial_state = Problem.CREATE_INITIAL_STATE()
    print("Initial State:")
    print(initial_state)
    global COUNT, BACKLINKS, MAX_OPEN_LENGTH, SOLUTION_PATH
    COUNT = 0
    BACKLINKS = {}
    MAX_OPEN_LENGTH = 0
    SOLUTION_PATH = AStar(initial_state)
    print(str(COUNT) + " states expanded.")
    print('MAX_OPEN_LENGTH = ' + str(MAX_OPEN_LENGTH))


def AStar(initial_state):
    '''A* Search'''
    global g, COUNT, BACKLINKS, MAX_OPEN_LENGTH, CLOSED, TOTAL_COST
    CLOSED = []
    BACKLINKS[initial_state] = None

    # STEP 1a. Put the start state on a priority queue called OPEN
    OPEN = My_Priority_Queue()

    g[initial_state] = 0.0
    f = g[initial_state] + h(initial_state)
    print("INITIAL STATE HAS F VALUE OF " + str(f))
    OPEN.insert(initial_state, f)

    while OPEN != []:
        # LEAVE THE FOLLOWING CODE IN PLACE TO INSTRUMENT AND/OR DEBUG YOUR IMPLEMENTATION
        if VERBOSE:
            report(OPEN, CLOSED, COUNT)

        if len(OPEN) > MAX_OPEN_LENGTH:
            MAX_OPEN_LENGTH = len(OPEN)

        # STEP 3. Select the state on OPEN having lowest priority value and call it S.
        #         Delete S from OPEN.
        #         Put S on CLOSED.
        #         If S is a goal state, output its description
        (S, P) = OPEN.delete_min()
        # P is the cost from starting state to S
        # print(f"S IS {S.name}")
        # print(f"G[S] IS {g[S]} WHICH REFLECTS THE COST TO GET FROM {initial_state.name} TO S")
        # print(f"H[S] IS {h(S)}")
        # print(f"F[S] IS {P}")
        # print(f"THE PRIORITY FOR S IS {P} WHICH REFLECTS THE COST TO GET FROM {initial_state.name} TO GOAL")
        # print("In Step 3, returned from OPEN.delete_min with results (S,P)= ", (str(S), P))

        # print("BACKLINKS = ")
        # for state in BACKLINKS.keys():
        #   print("parent = " + str(BACKLINKS.get(state)) + ", child = " + str(state))

        # CLOSED.append((S, P))
        CLOSED.append(S)

        if Problem.GOAL_TEST(S):
            print("\n" + Problem.GOAL_MESSAGE_FUNCTION(S) + "\n")

            # handling backtracing
            SOLUTION_PATH = [str(s) for s in backtrace(S)]  # should return a list
            print(f"Solution path = {SOLUTION_PATH}")
            print('Length of solution path found: ' + str(len(SOLUTION_PATH) - 1) + ' edges')

            # handling total cost
            TOTAL_COST = P
            print("TOTAL_COST = " + str(TOTAL_COST))

            return
        COUNT += 1

        # STEP 4. Generate each successor of S
        #         and if it is already on CLOSED, delete the new instance.
        successors = []
        for op in Problem.OPERATORS:
            if op.precond(S):
                new_state = op.state_transf(S)

                # h(curr_state) = an estimate of the distance between curr_state and GOAL
                temp_g = g[S] + new_state.edge_distance(S)
                # f = g[new_state] + h(new_state)
                f = temp_g + h(new_state)

                # pair at hand
                # (new_state, f[new_state])

                # print(f"\tCHILD OF {S.name} = {new_state}; THE COST TO GET FROM INITIAL {initial_state} TO GOAL THROUGH "
                      # f"{new_state.name} IS g=g[old]={g[S]} + actual edge weight={new_state.edge_distance(S)} + h={h(new_state)} = f={f[new_state]}")

                successors.append((new_state, f))
                # print(f"\tCHILD OF {S.name} =  {new_state.name} POINTS TO PARENT NODE {S.name}")

                # if there is new_state on CLOSED (for any priority q):
                # for i in range(len(CLOSED)):
                for closed_state in CLOSED:
                    # closed_state = CLOSED[i][0]
                    # q = CLOSED[i][1]
                    # closed_state = CLOSED[i]
                    q = g[closed_state] + h(closed_state)

                    if new_state == closed_state:
                        if f > q:
                            successors.remove((new_state, f))
                            # print(f"\tREMOVED FROM SUCCESSOR... {new_state.name}'s priority in CLOSED and is more expensive than q")

                        # if f(new_state) is smaller than or equal to q, then remove [s', q] from CLOSED.
                        # if f[new_state] <= q:
                        if f <= q:
                            CLOSED.remove(new_state)
                            BACKLINKS[new_state] = S
                            g[new_state] = temp_g
                            # print(f"\tREMOVED FROM CLOSED... {new_state.name}'s priority in CLOSED and is smaller than or equal to q")
                            # print(f"\tCLOSED =", end="")
                            # for state in CLOSED:
                            #     print(f"({state}), ", end="")
                            # print("\n")

                # if there is new_state on OPEN (for any priority q):
                if new_state in OPEN:
                    q = OPEN[new_state]
                    # if f(new_state) is more expensive than q, then remove[s, f(s')] from successors.
                    # if f[new_state] > q:
                    if f > q:
                        successors.remove((new_state, f))
                        # print(f"\tREMOVED FROM SUCCESSOR... {new_state.name}'s priority in OPEN and is more expensive than q")

                    # if f(new_state) is less expensive or equal to q, then remove [s', q] from OPEN.
                    # if f[new_state] <= q:
                    if f <= q:
                        OPEN.__delitem__(new_state)
                        # BACKLINKS[new_state] = S
                        g[new_state] = temp_g
                        BACKLINKS[new_state] = S
                        # print(f"\tREMOVED FROM OPEN... {new_state.name}'s priority in OPEN and is smaller than or equal to q")
                        # print(f"\tNEW PARENT NODE FOR {new_state.name} = {BACKLINKS[new_state].name}")

                if new_state not in CLOSED and OPEN.__contains__(new_state) is False:
                    g[new_state] = temp_g
                    BACKLINKS[new_state] = S
        print()

        for (s, p) in successors:
            OPEN.insert(s, p)
            # BACKLINKS[s] = S

        print_state_queue("OPEN", OPEN)

        # print("CLOSED: [ ", end="")
        # for (state, priority) in CLOSED:
            # print(f"({state.name}, {priority})", end=" ")
        # print("]")

        # cost_so_far -= P  # reset cost_so_far to current shortest path
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
    runAStar()
