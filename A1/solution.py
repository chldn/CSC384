#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems

import numpy as np


  
#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def distance(coord1, coord2):
  return abs(coord1[0]-coord2[0]) + abs(coord1[1]-coord2[1]) 

def closest_storage_cost(coord, coords):
  distances = (np.abs(coords[:,0]-coord[0])+np.abs(coords[:,1]-coord[1]))
  return distances[np.argmin(distances)]
    
    
def heur_manhattan_distance(state):
  '''
  INPUT: sokoban state
  OUTPUT: numeric value - distance estimate of current to goal state

  Used to estimate how many moves a current state is from a goal state. 
  The Manhattan distance between coordinates (x0,y0) and (x1,y1) is |x0 - x1| + |y0 - y1|. 

  State Manhattan distance = sum of Manhattan distances btwn each [unstored box & the nearest storage point]
                  * permitted by the storage restrictions
                  * ignore obstacle positions 
                  * assume multiple boxes can be stored at one storage loccation
  - optimistic heuristic - always underestimates cost 
  '''
  
  box_coords = np.array(list(state.boxes.keys()))
  box_indices = np.array(list(state.boxes.values()))
  
  
  dist = 0
  for box in state.boxes:
    if box not in state.storage:
      if not state.restrictions:
        dist += closest_storage_cost(box, np.array(list(state.storage.keys())))
      else:
        dist += closest_storage_cost(box, np.array([tuple(c)[0] for c in state.restrictions]))
  return dist


def heur_alternate(state):
  #IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''         
    #- improves upon heur_manhattan_distance 
    #     to estimate distance between current and goal states
    
    #Your function should return a numeric value - estimate of distance to goal
    return 0

def fval_function(sN, weight):
    #IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    # Many searches explore nodes/states ordered by node f-value
    #   For UCS, fvalue = gval of the state. 
    #   For best-first search, fvalue = hval of the state.
    # Use this function to create alternate f-value for states; 
    #               *must be a function of the state and the weight
    
    #Returns numeric f-value - determines state's position on Frontier list in 'custom' search
    
    #Must initialize search engine object as 'custom' if you supply a custom fval function


    return 0

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
    '''
    INPUT: sokoban state representing start state & timebound (number of seconds)
	OUTPUT: goal state (if a goal is found), else False'''
	''' 
	Anytime Greedy Best-First Search
	- first expands nodes with lowest h(n) 
	- may not be optimal
	- continues searching after a solution is found in order to improve solution quality
	- Since found a path to the goal after the first iteration, 
			cost bound for pruning: 
				if node has g(node) > best path the goal found so far:
					prune it
	- algorithm returns when:
		1. expanded all non-pruned nodes => optimal solution found
		2. time out

	- prune based on the g-value of the node only because greedy best-first search is not necessarily run with an admissible heuristic
	
	Record the time when anytime gb f s is called with os.times()[0]. 
	Each time you call search, update the time bound with the remaining allowed time. 
    ''' 
    OPEN = Open(_BEST_FIRST)
    return False

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 
    # return False
    # OPEN = Open(_ASTAR) #   OPEN = priority queue containing START
    # CLOSED = []

   ##   current = OPEN.extract
    # CLOSED.append(current)
    # while not sokoban_goal_state(current): # while lowest rank in OPEN is not the GOAL:
    #   current = OPEN.extract # remove lowest rank item from OPEN
    #   CLOSED.append(current) #   add current to CLOSED
    #   for 
    

    #   for neighbors of current:
    #     cost = g(current) + movementcost(current, neighbor)
    #     if neighbor in OPEN and cost less than g(neighbor):
    #       remove neighbor from OPEN, because new path is better
    #     if neighbor in CLOSED and cost less than g(neighbor):
    #       remove neighbor from CLOSED
    #     if neighbor not in OPEN and neighbor not in CLOSED:
    #       set g(neighbor) to cost
    #       add neighbor to OPEN
    #       set priority queue rank to g(neighbor) + h(neighbor)
    #       set neighbor's parent to current

    # reconstruct reverse path from goal to start
    # by following parent pointers

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_manhattan_distance)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  # solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  # print("Running Anytime Weighted A-star")   

 ##   for i in range(0, 10):
  #   print("*************************************")  
  #   print("PROBLEM {}".format(i))

 ##     s0 = PROBLEMS[i] #Problems get harder as i gets bigger
  #   weight = 10
  #   final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

 ##     if final:
  #     final.print_path()   
  #     solved += 1 
  #   else:
  #     unsolved.append(i)
  #   counter += 1      

 ##   if counter > 0:  
  #   percent = (solved/counter)*100   
  #     
  # print("*************************************")  
  # print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  # print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  # print("*************************************") 



