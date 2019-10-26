'''EightPuzzleWithHamming.py
This file augments EightPuzzle.py with heuristic information,
so that it can be used by an A* implementation.
The particular heuristic uses the number of tiles
out of place, not counting the blank tile, or
Hamming Distance.
'''

from EightPuzzle import *

GOAL_STATE = State([[0, 1, 2], [3, 4, 5], [6, 7, 8]])


def h(s):
    '''We return an estimate of the number of tiles out of place'''
    count = 0
    for i in range(len(s.b)):
        row = s.b[i]
        for j in range(len(row)):
            item = s.b[i][j]
            if item != 0:
                if s.b[i][j] != GOAL_STATE.b[i][j]:
                    count += 1
    return count


# A simple test:
# print(h(State([[0, 1, 5], [3, 4, 2], [6, 7, 8]])))
