# 8-puzzle
We use the heapq package in Lib/heapq.py to construct the priority queue and we implement the A* algorithm to solve the 8-puzzle problem.\
Given an intial state in a 1-dimension list, we will return the optimal path to the goal state [1,2,3,4,5,6,7,8,0].\
For example, solve([4,3,8,5,1,6,7,2,0]) prints:\
[4, 3, 8, 5, 1, 6, 7, 2, 0] h=10 moves: 0\
[4, 3, 8, 5, 1, 0, 7, 2, 6] h=11 moves: 1\
[4, 3, 0, 5, 1, 8, 7, 2, 6] h=10 moves: 2\
[4, 0, 3, 5, 1, 8, 7, 2, 6] h=9 moves: 3\
[4, 1, 3, 5, 0, 8, 7, 2, 6] h=8 moves: 4\
[4, 1, 3, 5, 8, 0, 7, 2, 6] h=7 moves: 5\
[4, 1, 3, 5, 8, 6, 7, 2, 0] h=6 moves: 6\
[4, 1, 3, 5, 8, 6, 7, 0, 2] h=7 moves: 7\
[4, 1, 3, 5, 0, 6, 7, 8, 2] h=6 moves: 8\
[4, 1, 3, 0, 5, 6, 7, 8, 2] h=5 moves: 9\
[0, 1, 3, 4, 5, 6, 7, 8, 2] h=4 moves: 10\
[1, 0, 3, 4, 5, 6, 7, 8, 2] h=3 moves: 11\
[1, 3, 0, 4, 5, 6, 7, 8, 2] h=4 moves: 12\
[1, 3, 6, 4, 5, 0, 7, 8, 2] h=5 moves: 13\
[1, 3, 6, 4, 5, 2, 7, 8, 0] h=4 moves: 14\
[1, 3, 6, 4, 5, 2, 7, 0, 8] h=5 moves: 15\
[1, 3, 6, 4, 0, 2, 7, 5, 8] h=6 moves: 16\
[1, 3, 6, 4, 2, 0, 7, 5, 8] h=5 moves: 17\
[1, 3, 0, 4, 2, 6, 7, 5, 8] h=4 moves: 18\
[1, 0, 3, 4, 2, 6, 7, 5, 8] h=3 moves: 19\
[1, 2, 3, 4, 0, 6, 7, 5, 8] h=2 moves: 20\
[1, 2, 3, 4, 5, 6, 7, 0, 8] h=1 moves: 21\
[1, 2, 3, 4, 5, 6, 7, 8, 0] h=0 moves: 22
