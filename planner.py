# Look for ### IMPLEMENT BELOW ### tags in this file. These tags indicate what has
# to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os
# Search engines
from search import *
# Warehouse specific classes
from warehouse import WarehouseState, Direction, warehouse_goal_state


def heur_displaced(state):
    '''A trivial example heuristic that is admissible'''
    '''INPUT: a warehouse state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    '''In this case, simply the number of displaced boxes.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
        return count


def heur_manhattan_distance(state):
    '''admissible heuristic: manhattan distance'''
    '''INPUT: a warehouse state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must always underestimate the cost to get from the current state to the goal.
    # The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.

    ### IMPLEMENT BELOW ###
    distance = 0
    for box in state.boxes:
        distance += min([manhattan_distance(box, storage) for storage in state.storage])
    ### END OF IMPLEMENTATION ###

    return distance


def manhattan_distance(point1, point2):
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


def fval_fn(sN, weight):
    return sN.gval + (weight * sN.hval)


def weighted_astar(initial_state, heuristic, weight, timebound=10):
    '''Provides an implementation of weighted a-star, as described in the PA2 handout'''
    '''INPUT: a warehouse state that represents the start state, the heursitic to be used,'''
    '''       weight for the A* search (w >= 1), and a timebound (number of seconds)'''
    '''OUTPUT: A WarehouseState (if a goal is found), else False'''

    ### IMPLEMENT BELOW ###
    searching = SearchEngine(strategy='custom', cc_level='full')
    searching.init_search(initial_state, goal_fn=warehouse_goal_state, heur_fn=heuristic,
                          fval_function=(lambda sN: fval_fn(sN, weight)))
    goal, stats = searching.search(timebound=timebound)

    ### END OF IMPLEMENTATION ###

    return goal


def iterative_astar(initial_state, heuristic, weight, timebound=10):
    '''Provides an implementation of iterative a-star, as described in the PA2 handout'''
    '''INPUT: a warehouse state that represents the start state, the heursitic to be used,'''
    '''       weight for the A* search (w >= 1), and a timebound (number of seconds)'''
    '''OUTPUT: A WarehouseState (if a goal is found), else False'''

    # HINT: Use os.times()[0] to obtain the clock time. Your code should finish within the timebound.'''

    ### IMPLEMENT BELOW ###
    initial_time = os.times()[0]
    searching = SearchEngine(strategy='custom', cc_level='full')
    searching.init_search(initial_state, goal_fn=warehouse_goal_state, heur_fn=heuristic,
                          fval_function=(lambda sN: fval_fn(sN, weight)))
    goal, stats = searching.search(timebound=timebound)

    if goal:
        best = goal
        time_remaining = timebound - os.times()[0] + initial_time
        while time_remaining > 0 and weight >= 0:
            starting_loop = os.times()[0]
            weight = weight - 0.5
            searching.init_search(initial_state, goal_fn=warehouse_goal_state, heur_fn=heuristic,
                                  fval_function=(lambda sN: fval_fn(sN, weight)))
            goal, stats = searching.search(timebound=time_remaining)
            if goal:
                best = goal
                time_remaining = time_remaining - os.times()[0] + starting_loop
            else:
                break
    else:
        return False

    ### END OF IMPLEMENTATION ###

    return best


def heur_alternate(state):
    '''a better warehouse heuristic'''
    '''INPUT: a warehouse state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''

    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    ### IMPLEMENT BELOW ###
    distance = 0
    if is_dead_situation(state):
        # This must be larger than all other possible undead heur
        return state.height * state.width * (state.height + state.width)
    else:
        distance += heur_manhattan_distance(state)
        distance += robot_to_boxes_distance(state)

    ### END OF IMPLEMENTATION ###

    return distance


def robot_to_boxes_distance(state):
    distance = 0
    for box in state.boxes:
        distance += min([manhattan_distance(box, robot) for robot in state.robots])
    return distance


def is_dead_situation(state):
    for box in state.boxes:
        (x, y) = box

        # Check if there is box at corner of wall
        if x == 0 and (y == 0 or y == state.height - 1):
            return True
        if x == state.width - 1 and (y == 0 or y == state.height - 1):
            return True

        # Check if the box is at corner of obstacles:
        if (x + 1, y) in state.obstacles and (x, y + 1) in state.obstacles:
            return True
        if (x + 1, y) in state.obstacles and (x, y - 1) in state.obstacles:
            return True
        if (x - 1, y) in state.obstacles and (x, y + 1) in state.obstacles:
            return True
        if (x - 1, y) in state.obstacles and (x, y - 1) in state.obstacles:
            return True

        # Check if the box is at corner with both obstacles and walls:
        if (x == 0 or x == state.width - 1) and ((x, y + 1) in state.obstacles or (x, y - 1) in state.obstacles):
            return True
        if (y == 0 or x == state.height - 1) and ((x + 1, y) in state.obstacles or (x - 1, y - 1) in state.obstacles):
            return True

        # Check if the box is at corner with both boxes and walls:
        if (x == 0 or x == state.width - 1) and ((x, y + 1) in state.boxes or (x, y - 1) in state.boxes):
            return True
        if (y == 0 or x == state.height - 1) and ((x + 1, y) in state.boxes or (x - 1, y - 1) in state.boxes):
            return True

        # Check if Box in a row/column against the Wall that has no goal:
        if x == 0 or x == state.width - 1:
            for col in range(state.height):
                if (x, col) in state.storage:
                    break
            return True
        if y == 0 or y == state.height - 1:
            for row in range(state.width):
                if (row, y) in state.storage:
                    break
            return True
    return False
