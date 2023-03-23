import copy
import time
import sys
from abc import ABC
from typing import Tuple, Union, Dict, List, Any
import math
import numpy as np

from commonroad.scenario.trajectory import State

sys.path.append('../')
from SMP.maneuver_automaton.motion_primitive import MotionPrimitive
from SMP.motion_planner.node import Node, CostNode
from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.motion_planner.queue import FIFOQueue, LIFOQueue, PriorityQueue
from SMP.motion_planner.search_algorithms.base_class import SearchBaseClass
from SMP.motion_planner.utility import MotionPrimitiveStatus, initial_visualization, update_visualization

class SequentialSearch(SearchBaseClass, ABC):
    """
    Abstract class for search motion planners.
    """

    # declaration of class variables
    path_fig: Union[str, None]

    steps = 0

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)

    def initialize_search(self, time_pause, cost=True):
        """
        initializes the visualizer
        returns the initial node
        """
        self.list_status_nodes = []
        self.dict_node_status: Dict[int, Tuple] = {}
        self.time_pause = time_pause
        self.visited_nodes = []

        # first node
        if cost:
            node_initial = CostNode(list_paths=[[self.state_initial]],
                                        list_primitives=[self.motion_primitive_initial],
                                        depth_tree=0, cost=0)
        else:
            node_initial = Node(list_paths=[[self.state_initial]],
                                list_primitives=[self.motion_primitive_initial],
                                depth_tree=0)
        initial_visualization(self.scenario, self.state_initial, self.shape_ego, self.planningProblem,
                              self.config_plot, self.path_fig)
        self.dict_node_status = update_visualization(primitive=node_initial.list_paths[-1],
                                                     status=MotionPrimitiveStatus.IN_FRONTIER,
                                                     dict_node_status=self.dict_node_status, path_fig=self.path_fig,
                                                     config=self.config_plot,
                                                     count=len(self.list_status_nodes), time_pause=self.time_pause)
        self.list_status_nodes.append(copy.copy(self.dict_node_status))
        return node_initial

    def take_step(self, successor, node_current, cost=True):
        """
        Visualizes the step of a successor and checks if it collides with either an obstacle or a boundary
        cost is equal to the cost function up until this node
        Returns collision boolean and the child node if it does not collide
        """
        # translate and rotate motion primitive to current position
        list_primitives_current = copy.copy(node_current.list_primitives)
        path_translated = self.translate_primitive_to_current_state(successor,
                                                                    node_current.list_paths[-1])
        list_primitives_current.append(successor)
        self.path_new = node_current.list_paths + [[node_current.list_paths[-1][-1]] + path_translated]
        if cost:
            child = CostNode(list_paths=self.path_new,
                                 list_primitives=list_primitives_current,
                                 depth_tree=node_current.depth_tree + 1,
                                 cost=self.cost_function(node_current))
        else:
            child = Node(list_paths=self.path_new, list_primitives=list_primitives_current,
                         depth_tree=node_current.depth_tree + 1)

        # check for collision, skip if is not collision-free
        if not self.is_collision_free(path_translated):

            position = self.path_new[-1][-1].position.tolist()
            self.list_status_nodes, self.dict_node_status, self.visited_nodes = self.plot_colliding_primitives(current_node=node_current,
                                                                                           path_translated=path_translated,
                                                                                           node_status=self.dict_node_status,
                                                                                           list_states_nodes=self.list_status_nodes,
                                                                                           time_pause=self.time_pause,
                                                                                           visited_nodes=self.visited_nodes)
            return True, child
        self.update_visuals()
        return False, child

    def update_visuals(self):
        """
        Visualizes a step on plot
        """
        position = self.path_new[-1][-1].position.tolist()
        if position not in self.visited_nodes:
            self.dict_node_status = update_visualization(primitive=self.path_new[-1],
                                                         status=MotionPrimitiveStatus.IN_FRONTIER,
                                                         dict_node_status=self.dict_node_status, path_fig=self.path_fig,
                                                         config=self.config_plot,
                                                         count=len(self.list_status_nodes), time_pause=self.time_pause)
            self.list_status_nodes.append(copy.copy(self.dict_node_status))
        self.visited_nodes.append(position)

    def goal_reached(self, successor, node_current):
        """
        Checks if the goal is reached.
        Returns True/False if goal is reached
        """
        path_translated = self.translate_primitive_to_current_state(successor,
                                                                    node_current.list_paths[-1])
        # goal test
        if self.reached_goal(path_translated):
            # goal reached
            self.path_new = node_current.list_paths + [[node_current.list_paths[-1][-1]] + path_translated]
            path_solution = self.remove_states_behind_goal(self.path_new)
            self.list_status_nodes = self.plot_solution(path_solution=path_solution, node_status=self.dict_node_status,
                                                        list_states_nodes=self.list_status_nodes, time_pause=self.time_pause)
            return True
        return False

    def get_obstacles_information(self):
        """
        Information regarding the obstacles.
        Returns a list of obstacles' information, each element
        contains information regarding an obstacle:
        [x_center_position, y_center_position, length, width]

        """
        return self.extract_collision_obstacles_information()

    def get_goal_information(self):
        """
        Information regarding the goal.
        Returns a list of the goal's information
        with the following form:
        [x_center_position, y_center_position, length, width]
        """
        return self.extract_goal_information()

    def get_node_information(self, node_current):
        """
        Information regarding the input node_current.
        Returns a list of the node's information
        with the following form:
        [x_center_position, y_center_position]
        """
        return node_current.get_position()

    def get_node_path(self, node_current):
        """
        Information regarding the input node_current.
        Returns the path starting from the initial node and ending at node_current.
        """
        return node_current.get_path()

    def cost_function(self, node_current):
        """
        Returns g(n) from initial to current node, !only works with cost nodes!
        """
        velocity = node_current.list_paths[-1][-1].velocity

        node_center = self.get_node_information(node_current)
        goal_center = self.get_goal_information()
        distance_x = goal_center[0] - node_center[0]
        distance_y = goal_center[1] - node_center[1]
        length_goal = goal_center[2]
        width_goal = goal_center[3]

        distance = 4.5
        if(abs(distance_x)<length_goal/2 and abs(distance_y)<width_goal/2):
            prev_x = node_current.list_paths[-2][-1].position[0]
            prev_y = node_current.list_paths[-2][-1].position[1]
            distance = goal_center[0] - length_goal / 2 - prev_x
        cost = node_current.cost + distance
        
        return cost

    def heuristic_function(self, node_current, heuristic_def):
        """
        Enter your heuristic function h(x) calculation of distance from node_current to goal
        Returns the distance normalized to be comparable with cost function measurements
        """

        ditance = 0
        if heuristic_def:
            distance = self.calc_euclidean_distance(node_current)
        else:
            D = D2 = 1
            dx = abs(self.get_node_information(node_current)[0] - self.get_goal_information()[0])
            dy = abs(self.get_node_information(node_current)[1] - self.get_goal_information()[1])
            distance = D * (dx + dy) #+ (D2 - 2 * D) * min(dx, dy)
        return distance


    def evaluation_function(self, node_current, choose_heuristic, weight):
        """
        f(x) = g(x) + h(x)
        """
        g = self.cost_function(node_current)
        h = self.heuristic_function(node_current, choose_heuristic)
        f = g + h*weight
        return f

    def execute_search(self, time_pause, weight, choose_heuristic,filename) -> Tuple[Union[None, List[List[State]]], Union[None, List[MotionPrimitive]], Any]:
        node_initial = self.initialize_search(time_pause=time_pause)
        #print(self.get_obstacles_information())
        #print(self.get_goal_information())
        #print(self.get_node_information(node_initial))
        goal_found = self.Astar_alogorithm(node_initial, weight, choose_heuristic,filename)
        return True


    # function used to print and save all the needed information
    def print_all(self, choose_heuristic, node_initial,node_final):
        print("Visited Nodes : " ,self.steps)
        self.save_file("Visited Nodes : " + str(self.steps) + "\n")
        print("Path : " + self.create_path(self.get_node_path(node_final)))
        self.save_file("Path : " + self.create_path(self.get_node_path(node_final)) + "\n")
        print("Heuristic cost : " , self.heuristic_function(node_initial, choose_heuristic))
        self.save_file("Heuristic cost : " + str(self.heuristic_function(node_initial, choose_heuristic)) + "\n")
        print("Estimated Cost On Goal: ",self.cost_function(node_final))
        self.save_file("Estimated Cost On Goal: " + str(self.cost_function(node_final)) + "\n")
        print("\n===============================================================\n")
        self.save_file("\n===============================================================\n")

    
    # function used to create the path that our algoritm suggests to take 
    def create_path(self, path):
        my_path = ""
        my_path_len = len(path)
        i = 0 
        for x in path:
            i+=1
            my_path+= "(" + str("{:.2f}".format(x[0])) + "," + str("{:.2f}".format(x[1])) + ")"
            # if its the last node don't add arrow in the end 
            if my_path_len != i:
                my_path+="->"

        return my_path

    # function used to save info to our file 
    def save_file(self, my_str):
        f.write(my_str+"\n")


    # our search algorithm for  A*
    def Astar_alogorithm(self, node_initial, weight, choose_heuristic,filename):
        self.steps = 0  #initialize steps to
        global f
        f = open(filename, "a+")    #open file 
        heur = "manhattan"  # if choose_heuristic is False then heur equals manhattan
        if choose_heuristic:    # if it's true then heur value is gonna be euclidean 
            heur = "euclidean"
        string = "A* Search Heuristic: "+ heur + " w = " + str(weight)  # string written to all outputs 
        self.save_file(string)  
        print(string)

        fringe = PriorityQueue();
        fringe.insert(node_initial, self.evaluation_function(node_initial, choose_heuristic, weight))

        # while fringe has nodes meaning is not empty 
        while not fringe.empty():
            node_current = fringe.pop();    # pop a node from fringe

            for primitive_successor in node_current.get_successors():                
                collision_flag, child = self.take_step(successor=primitive_successor, node_current=node_current)
                self.steps+=1   # plus 1 to visited nodes 
                
                # if it collides with an obstacle or boundary skip this successor
                if collision_flag:
                    continue

                # check if goal is reached
                goal_flag = self.goal_reached(successor=primitive_successor,
                                                                         node_current=node_current)
                # if goal is reached, return back with the solution path
                if goal_flag:
                    self.print_all(choose_heuristic, node_initial,child)
                    return True

                # if goal is not reached, continue the search recursive
                eval = self.evaluation_function(child, choose_heuristic, weight)
                fringe.insert(child, eval)
        print("Fringe empty")

        return False



class Astar(SequentialSearch):
    """
    Class for Astar Search algorithm.
    """

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)
 
