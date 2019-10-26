'''EightPuzzleWithManhattan.py
This file augments EightPuzzle.py with heuristic information,
so that it can be used by an A* implementation.
The particular heuristic uses the sum of |x1 - x2 | + |y1 - y2|,
or Manhattan Distance, between two states.
'''

from EightPuzzle import *

GOAL_STATE = State([[0, 1, 2], [3, 4, 5], [6, 7, 8]])

def h(s):
    distance = 0
    for x1 in range(len(s.b)):
        for y1 in range(len(s.b[x1])):
            s_item = s.b[x1][y1]
            goal_item = GOAL_STATE.b[x1][y1]

            if s_item != goal_item and s_item != 0:  # s_item is out of place
                # find the actual goal item that needs to be at x1, y1
                for x2 in range(len(GOAL_STATE.b)):
                    for y2 in range(len(GOAL_STATE.b[x2])):
                        actual_item = GOAL_STATE.b[x2][y2]
                        if s_item == actual_item:
                            s_coord = (x1, y1)
                            goal_coord = (x2, y2)
                            distance += (abs(x1 - x2) + abs(y1 - y2))
    return distance


# A simple test:
# print(h(State([[0, 2, 1], [3, 4, 5], [6, 7, 8]])))
# print(h(State([[2, 1, 3], [5, 4, 0], [6, 7, 8]])))
