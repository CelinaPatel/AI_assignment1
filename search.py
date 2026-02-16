# search.py
# Assignment 1:
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


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem):
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
    "*** YOUR CODE HERE *** (Q1)"

    # use stack in order to find the deepest nodes first
    fringe = util.Stack() # set visited states to refrain from expanding same state
    visit = set() # assign visited states to refrain from expanding a state that is already expanded
    start = problem.getStartState() # start state

    # push beginning state (here it goes through each item)
    fringe.push((start,[]))

    # used pseudocode given: go through until no states are left or while stack is not empty, pop
    while not fringe.isEmpty():
        state, path = fringe.pop()

        # if state is goal, return path
        if problem.isGoalState(state):
            return path
        # if the state is visited then skip
        if state in visit:
            continue
        visit.add(state) # add state

        # this code will expand the state we are in currently and it will do this by looking at the successors it has
        for succ, action, cost in problem.getSuccessors(state):
            if succ in visit:
                continue
            # if successor is not in visit,  add successor that are not expanded currently and push the fringe
            if succ not in visit:
                fringe.push((succ, path + [action]))
    return[]

    # util.raiseNotDefined()

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    
    "*** YOUR CODE HERE *** (Q2)"

    # First in First Out (FIFO) is used and because of this we expand the oldest nodes that are present first
    fringe = util.Queue()
    visit = set() 
    start = problem.getStartState()

    fringe.push((start,[]))

    #  while the queue is not empty, pop fringe
    while not fringe.isEmpty():
        state, path = fringe.pop()

        # if goal is state, return path
        if problem.isGoalState(state):
            return path
        # if state is in visit, skip 
        if state in visit:
            continue
        visit.add(state) # add state to visit to make it is processed once only

        # current state needs to be expanded and use successor to do this
        for succ, action, cost in problem.getSuccessors(state):
            # if successor is in visit, skip
            if succ in visit:
                continue
            # if successor is not in visit, push fringe
            if succ not in visit:
                fringe.push((succ, path + [action]))
    return[]
    
    # util.raiseNotDefined()

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    
    "*** YOUR CODE HERE *** (Q3)"
    
    priorityQueue = util.PriorityQueue() # this will pop whichever state with the lowest total cost for its path which is
    start = problem.getStartState()
    bestCost = {} 
    priorityQueue.push((start, [], 0), 0) 
    bestCost[start] = 0 # set lowest cost to 0 as the best cost
   
    # while priority queue not empty, pop state with lowest cost
    while not priorityQueue.isEmpty():
        state, path, cost_so_far = priorityQueue.pop()
        
        if problem.isGoalState(state):
            return path

        # largen the neighbours
        for succ, action, stepCost in problem.getSuccessors(state):
            # add current cost to cost of the present step and assign to new cost
            new_cost = cost_so_far + stepCost

            # if state not visited or it found cheaper cost, record best cost
            if succ not in bestCost or new_cost < bestCost[succ]:
                bestCost[succ] = new_cost
                # push successors with updated cost
                priorityQueue.push((succ, path + [action], new_cost), new_cost)

    return []
    
    #util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    
    "*** YOUR CODE HERE *** (Q4)"
    priorityQueue = util.PriorityQueue()
    start = problem.getStartState()
    bestCost = {}
    priorityQueue.push((start, [], 0), heuristic(start, problem))
    bestCost[start] = 0
   

    while not priorityQueue.isEmpty():
        state, path, cost_so_far = priorityQueue.pop()
        
        if problem.isGoalState(state):
            return path

        for succ, action, stepCost in problem.getSuccessors(state):
            new_cost = cost_so_far + stepCost

            if succ not in bestCost or new_cost < bestCost[succ]:
                bestCost[succ] = new_cost
                priorityQueue.push((succ, path + [action], new_cost), new_cost + heuristic(succ, problem)) 

    return []

    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
