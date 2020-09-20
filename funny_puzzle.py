# The source code of data structure for priority queue is available at
# https://github.com/python/cpython/blob/3.8/Lib/heapq.py
# We use the priority queue and implement the A* algorithm to solve the 8-puzzle problem.
"""
Created on Thu Sep 17 21:33:45 2020

@author: Hongxu Chen
"""

from heapq import *
import numpy as np



# The correct position of number i in a 2d array.
corr_pos = {1:[0,0], 2:[0,1], 3:[0,2], 4:[1,0], 
            5:[1,1], 6:[1,2], 7:[2,0], 8:[2,1]}


# All neighbors of at position i.
neighbor = {0:[1,3], 1:[0,2,4], 2:[1,5], 3:[0,4,6], 
            4:[1,3,5,7], 5:[2,4,8], 6:[3,7], 7:[4,6,8], 8:[5,7]}


# Manhattan distance for two states given in form of 2d array.
def manhattan(pos_1, pos_2):
    return abs(pos_1[0]-pos_2[0])+abs(pos_1[1]-pos_2[1])


# Heuris of a given sate using Manhattan distance.
def heuris(state):
    # The state is given in 1d, first reshape into a 2d array.
    state_2d = np.reshape(state,(3,3))
    sum = 0
    # Use the correct position in line 14 to evaluate the distance to
    # the correct position.0
    for i in range(0,3):
        for j in range(0,3):
            if(state_2d[i][j]!=0):
                sum = sum + manhattan([i,j],corr_pos[state_2d[i][j]])
    return sum


# This function will be used in print_succ for printing the successors.
# Return all the successor states without sorting of printing.
def successor(state):
    succ = []
    pos_zero = 0
    # First find out the position of the 0.
    for i in range(0,9):
        if state[i]==0:
            pos_zero = i
            break
    # Use the position of neighbors in line 18 to swap with 0.
    for i in neighbor[pos_zero]:
        new_state = state.copy()
        temp = new_state[i]
        new_state[i] = 0
        new_state[pos_zero] = temp
        succ.append(new_state)
    return succ


# print all the successor with sorted order.
def print_succ(state):
    # Get all the successors.
    succ = successor(state)
    # Sort the successors in ascending order.
    sorted(succ)
    # Print all the successors.
    for line in succ:
        print(line, end=" ")
        print("h=" + str(heuris(line)))


# Implement the A* algorithm to find the optimal path the correct state.
# Here in order to so store the cost value of a given state and also
# have a better time complexity, 
# instead of using the priority queue as the open state, 
# we use dictionary for the open state so that we can use constant time
# to verify if a state is already in the open state or not and get its cost.
# In order to backtrack to get the path, we use a dictionary
# to store the parent of each state.
def solve(state):
    queue = []
    # Use the function in heapq to insert the original state
    # into the priority queue.
    heappush(queue,(heuris(state), state, (0, heuris(state), -1)))
    # Variable to denote if we find the goal state or not.
    find = False
    # We use disctionary to denote the open state and closed state.
    open = {}
    closed = {}
    # Dictionary that stores the parent of states.
    parent = {}
    # The parent of the original state is -1.
    parent[tuple(state)] = -1
    # The cost of the orginal state is 0, mark it as open.
    open[tuple(state)] = 0
    max = 0
    
    # Start the A* algorithm.
    while len(queue)>0:
        if len(queue)>max:
            max = len(queue)
        # Pop from the priority queue.
        next = heappop(queue)
        change = tuple(next[1])
        # Push into the closed state and pop from the open state.
        closed[change] = next[2][0]
        open.pop(change)
        
        # The the pop state is the goal state, we are done and break the loop.
        if heuris(next[1]) == 0:
            goal = next
            find = True
            break
        # Insert all the successor of the pop state if neccssary.
        for succ in successor(next[1]):
            tp = tuple(succ)
            # Insert into the queue if the successor not in open or closed.
            if tp not in open and tp not in closed:
                heappush(queue, (next[2][0]+1+heuris(succ), 
                        succ, (next[2][0]+1, heuris(succ), next[2][2]+1)))
                # Assign the parent index.
                parent[tp] = next[1]
                # Push the successor into open.
                open[tp] = next[2][0]+1
            # If the successor is already in open
            elif tp in open:
                # If the cost is the successor is smaller,
                # replace the previous one with the new successor.
                if next[2][0]+1<open[tp]:
                    for i in range(len(queue)):
                        if queue[i][1] == succ:
                            queue.pop(i)
                            break
                    heappush(queue, (next[2][0]+1+heuris(succ), 
                            succ, (next[2][0]+1, heuris(succ), next[2][2]+1)))
                    parent[tp] = next[1]
                    open[tp] = next[2][0] + 1 
            # If the successor is already in closed.
            else:
                # Reinsert the successor into queue and open 
                # if the new cost is smaller.
                if next[2][0]+1<closed[tp]:
                    parent[tp] = next[1]
                    heappush(queue, (next[2][0]+1+heuris(succ), 
                            succ, (next[2][0]+1, heuris(succ), next[2][2]+1)))
                    open[tp] = next[2][0]+1
    # Print the path if we successfully find the goal state.
    if find:
        path = []
        goal = goal[1]
        # Use the parent to backtrack to the original state.
        while goal!=-1:
            path.append(goal)
            goal = parent[tuple(goal)]
            
        index = 0
        # Print the path in reverse order since the path is backtrack.
        for i in range(len(path)-1,-1,-1):
            print(path[i], end=" ")
            print("h=" + str(heuris(path[i])), end=" ")
            print("moves=" + str(index))
            index = index+1
    # If we fail to find the goal state print can not find path.
    else:
        print("Can not find path")
    






