import heapq

import numpy as np
from heapq import heappush, heappop
from animation import draw
import argparse

class Node():
    """
    cost_from_start - the cost of reaching this node from the starting node
    state - the state (row,col)
    parent - the parent node of this node, default as None
    """
    def __init__(self, state, cost_from_start, parent = None):
        self.state = state
        self.parent = parent
        self.cost_from_start = cost_from_start


class Maze():
    
    def __init__(self, map, start_state, goal_state, map_index):
        self.start_state = start_state
        self.goal_state = goal_state
        self.map = map
        self.visited = [] # state
        self.m, self.n = map.shape 
        self.map_index = map_index

    def draw(self, node):
        path=[]
        while node.parent:
            path.append(node.state)
            node = node.parent
        path.append(self.start_state)
    
        draw(self.map, path[::-1], self.map_index)

    def goal_test(self, current_state):
        return np.array_equal(current_state, self.goal_state)

    def get_cost(self, current_state, next_state):
        return 1

    def get_successors(self, state):
        successors = []

        # getting the specific row and column from the state tuple to get the exact integers
        temp_state = tuple(state)
        row, col = temp_state[0], temp_state[1]

        # initializing the four possible directions the red square could go to
        top = (row+1, col)
        bottom = (row-1, col)
        left = (row, col-1)
        right = (row, col+1)
        # Since the maze is surrounded by black squares, and we do not swap with them
            # there is no need to check whether a position is within the boundaries

        for direction in range(4):
            # temporary state:
            if direction == 0:
                # let's check top
                if self.map[top] == 1.0:
                    successors.append(top)
            elif direction == 1:
                # let's check bottom
                if self.map[bottom] == 1.0:
                    successors.append(bottom)
            elif direction == 2:
                # let's check left
                if self.map[left] == 1.0:
                    successors.append(left)
            else:
                # let's check right
                if self.map[right] == 1.0:
                    successors.append(right)
        return successors

    # heuristics function
    def heuristics(self, state):
        # I get the heuristic value by calculating how far the red square is from the goal position
        value = abs(self.goal_state[0] - state[0]) + abs(self.goal_state[1] - state[1])
        return value

    # priority of node 
    def priority(self, node):
        return node.cost_from_start + self.heuristics(node.state)

    # solve it
    def solve(self):
        # your code goes here:
        maze = [] # node
        state = tuple(self.start_state) # this is copying the position of red square that is a tuple
        node = Node(state, 0, None)
        self.visited.append(state)

        # in order to break the tie in priority queue
        tie_breaker = 0

        if self.goal_test(state):
            return state

        # push it into a queue
        item = (self.priority(node), tie_breaker, node)
        heapq.heappush(maze, item)

        while maze:
            current = heapq.heappop(maze)
            current = current[len(current) - 1]
            successors = self.get_successors(current.state)
            for next_state in successors:
                # to count that state is not in visited
                false_counter = 0
                for visited_state in self.visited:
                    if np.array_equal(visited_state, next_state) is False:
                        # run for everything in self.visited
                        false_counter += 1

                if false_counter == len(self.visited):
                    # create a node for next_state
                    next_cost = current.cost_from_start + self.get_cost(current.state, next_state)
                    next_node = Node(next_state, next_cost, current)
                    self.visited.append(next_state)

                    if self.goal_test(next_state) is True:
                        self.draw(next_node)
                        return next_state

                    tie_breaker += 1
                    item = (self.priority(next_node), tie_breaker, next_node)
                    heapq.heappush(maze, item)

    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='maze')
    parser.add_argument('-index', dest='index', required = True, type = int)
    index = parser.parse_args().index

    # Example:
    # Run this in the terminal solving map 1
    #     python maze_astar.py -index 1
    
    data = np.load('map_'+str(index)+'.npz')
    map, start_state, goal_state = data['map'], tuple(data['start']), tuple(data['goal'])

    game = Maze(map, start_state, goal_state, index)
    game.solve()
    