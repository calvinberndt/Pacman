# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util
from game import Directions
from typing import List

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    
    
    fringe = util.Stack() #Stack is a LIFO data structure, which allows DFS to work since it will explore the deepest nodes first.
    visited = set() #visited keeps track of the states that have been visited

    start_state = problem.getStartState() #grab the initial node, which is a tuple (x,y) of integers specifying Pacman's position.
    fringe.push((start_state, [])) #push the initial node and the empty path to the fringe. 

    while not fringe.isEmpty():
        state, path = fringe.pop() #pop the last node and the path to the goal from the fringe, in the first iteration it will be the initial node and the empty path.

        if state in visited: #if the state has been visited, we already explored its successors and added them to the fringe.
            continue
        visited.add(state) 

        if problem.isGoalState(state): #isGoalState checks if the state is the goal state. If our current state is the goal state, we return the path to the goal.
            return path

        for successor, action, _ in problem.getSuccessors(state): #getSuccessors returns the successors of the current state.
            # successor is a tuple (x,y) of integers specifying the successor's position.
            # action is the action required to get to the successor. It is a Directions enum (North, South, East, West, Stop).
            # _ is the cost of the action. Since we are using DFS, cost is not needed so it can be thrown away.
            if successor not in visited: #if the successor has not been visited, we add it to the fringe.
                fringe.push((successor, path + [action])) #pair the successor coordinates with the full action sequence that leads from the start to that successor

    return []
    

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first.
    This is essentially the same as DFS, but with a dequeue instead of a pop."""
    
    
    
    fringe = util.Queue() #Queue is a FIFO data structure, which allows BFS to work since it will explore the shallowest nodes first.
    visited = set() #visited keeps track of the states that have been visited
    
    start_state = problem.getStartState() #grab the initial node, which is a tuple (x,y) of integers specifying Pacman's position.
    fringe.push((start_state, [])) #push the initial node and the empty path to the fringe. 

    while not fringe.isEmpty():
        state, path = fringe.pop()
        
        if state in visited:
            continue
        
        visited.add(state)
        
        if problem.isGoalState(state):
            return path
        
        for successor, action, _ in problem.getSuccessors(state):
            if successor not in visited:
                fringe.push((successor, path + [action]))
    
    return []

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first.
    This is the same as BFS and DFS, except the fringe is ordered by the cumulative path cost."""
    
    fringe = util.PriorityQueue() #PriorityQueue is a FIFO data structure, which allows UCS to work since it will explore the nodes with the lowest cost first.
    visited = set() #visited keeps track of the states that have been visited
    
    start_state = problem.getStartState() #grab the initial node, which is a tuple (x,y) of integers specifying Pacman's position.
    best_cost = {start_state: 0}
    fringe.push((start_state, []), 0) # With PriorityQueue, we expect two arguments: the tuple of the state and the path, and the priority (cost) of the state.
    
    while not fringe.isEmpty():
        state, path = fringe.pop() #pop only returns one value, the state and the path, which is packed in a tuple. Unpack the tuple
        cost_so_far = best_cost[state] #retrieve the cost to reach the state from the best_cost dictionary.
        
        if state in visited and cost_so_far > best_cost[state]: #if the state is visited AND the cost to reach it is greater than the best cost to reach it, we skip it.
            continue
        
        visited.add(state)
        
        if problem.isGoalState(state):
            return path
        
        for successor, action, cost in problem.getSuccessors(state):
            new_cost = cost_so_far + cost #update the cost to reach the successor.
            if successor not in visited or new_cost < best_cost[successor]: #if the state is not visited or the cost to reach it is less than the best cost to reach it, we add it to the fringe.
                fringe.push((successor, path + [action]), new_cost)
                best_cost[successor] = new_cost #update the best cost to reach the successor.
    
    return []

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost (g) and heuristic (h) first."""

    def priority(item):
        state, _, g_cost = item #item is a tuple of the state, the path, and the cost to reach the state.
        return g_cost + heuristic(state, problem) #return the cost to reach the state plus the heuristic value of the state.

    #The priority function is used to prioritize the nodes in the fringe by taking a function that returns the priority of the state.
    #We are using PriorityQueueWithFunction because it allows us to pass a function as an argument to the PriorityQueue constructor.
    #This in turn allows us to order the nodes in the fringe by their lowest cost.
    fringe = util.PriorityQueueWithFunction(priority) 
    visited = set() #visited keeps track of the states that have been visited

    start_state = problem.getStartState()
    best_cost = {start_state: 0.0}
    fringe.push((start_state, [], 0.0)) #pack everything into a single tuple, compared to my previous implementation, where i packed the state and the path into a tuple and the cost into a separate variable.

    while not fringe.isEmpty():
        state, path, g_cost = fringe.pop()

        if state in visited and g_cost > best_cost.get(state, float('inf')): #if best.cost(get(set)) doesn't exist we set it to infinite to prevent errors.
            continue
        visited.add(state)

        if problem.isGoalState(state):
            return path

        for successor, action, step_cost in problem.getSuccessors(state):
            new_cost = g_cost + step_cost
            if successor in visited and new_cost >= best_cost.get(successor, float('inf')):
                continue
            if successor not in best_cost or new_cost < best_cost[successor]:
                best_cost[successor] = new_cost
                fringe.push((successor, path + [action], new_cost))

    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
