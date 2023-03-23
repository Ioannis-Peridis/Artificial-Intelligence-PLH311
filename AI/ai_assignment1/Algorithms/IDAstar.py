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
        distance = 0
        if heuristic_def:
            distance = self.calc_euclidean_distance(node_current)
        else:
            distance = self.manhattan_distance(node_current)
        return distance

    def manhattan_distance(self,node_current):
        D = D2 = 1
        dx = abs(self.get_node_information(node_current)[0] - self.get_goal_information()[0])
        dy = abs(self.get_node_information(node_current)[1] - self.get_goal_information()[1])
        distance = D * (dx + dy)
        return distance

    def evaluation_function(self, node_current,choose_heuristic,weight):
        """
        f(x) = g(x) + h(x)
        """
        g = self.cost_function(node_current)
        h = self.heuristic_function(node_current,choose_heuristic)
        f = g + h*weight
        return f

    # function used to start search
    def execute_search(self, time_pause, weight, choose_heuristic,filename) -> Tuple[Union[None, List[List[State]]], Union[None, List[MotionPrimitive]], Any]:  
        self.steps = 0  # initialise steps to 0
        node_initial = self.initialize_search(time_pause=time_pause)
        self.IDA_star_algoritm(node_initial,weight,choose_heuristic,filename);  # start IDA* search
        f.close()
        return True

    # clear fringe when no nodes are inside our current limit 
    def clear_fringe(self, fringe, node_initial,choose_heuristic, weight):
        fringe = PriorityQueue()
        fringe.insert(node_initial, self.evaluation_function(node_initial, choose_heuristic,weight))
        return fringe

    # function used to print and save to file all the info needed
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


    # converting the path that our algo showed into a string for our outputs  
    def create_path(self, path):
        my_path = ""
        my_path_len = len(path)
        i = 0 
        for x in path:
            i+=1
            my_path+= "(" + str("{:.2f}".format(x[0])) + "," + str("{:.2f}".format(x[1])) + ")"
            if my_path_len != i:
                my_path+="->"

        return my_path

    def save_file(self, my_str):
        f.write(my_str+"\n")


    # IDA* algorithm
    def IDA_star_algoritm(self, node_initial, weight, choose_heuristic,filename):
        global f
        f = open(filename, "a+")

        # if choose_heuristic is true then heur is euclidean else is manhattan
        heur = "manhattan"
        if choose_heuristic:
            heur = "euclidean"


        string = "IDA* Search Heuristic: "+ heur + " w = " + str(weight)
        self.save_file(string)
        print(string)

        
        fringe = PriorityQueue()
        fringe.insert(node_initial, self.evaluation_function(node_initial,choose_heuristic, weight))
        limit = self.evaluation_function(node_initial,choose_heuristic, weight)

        # while fringe is not empty 
        while not fringe.empty():
            node_current = fringe.pop() # pop node from fringe 

            for primitive_successor in node_current.get_successors():
                #execute step from node_current to primitive_successor
                collision_flag, child = self.take_step(successor=primitive_successor, node_current=node_current)
                self.steps+=1   #visited nodes increases by 1  

                eval_child = self.evaluation_function(child,choose_heuristic, weight)

                # if eval_child gretaer than limit then limit changes value and clear fringe and break 
                if (eval_child > limit):
                    new_lim = True
                    limit = eval_child
                    fringe = self.clear_fringe(fringe,node_initial,choose_heuristic, weight)
                    break
                else:
                    # if it collides with an obstacle or boundary skip this successor
                    if collision_flag:
                        continue

                    # check whether goal is reached
                    goal_found = self.goal_reached(successor=primitive_successor,
                                                                             node_current=node_current)
                    # if a recursive successor returns with goal reached and a solution path, no further recursion is required
                    if goal_found:
                        self.print_all(choose_heuristic, node_initial, child)
                        return True
                    fringe.insert(child, eval_child)
        print("Fringe empty")
        return False

class IterativeDeepeningAstar(SequentialSearch):
    """
    Class for Iterative Deepening Astar Search algorithm.
    """

    def __init__(self, scenario, planningProblem, automaton, plot_config=DefaultPlotConfig):
        super().__init__(scenario=scenario, planningProblem=planningProblem, automaton=automaton,
                         plot_config=plot_config)
