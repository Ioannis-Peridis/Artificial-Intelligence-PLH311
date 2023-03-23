import os
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt

try:
    mpl.use('Qt5Agg')
except ImportError:
    mpl.use('TkAgg')

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer

# add current directory to python path for local imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
from SMP.motion_planner.motion_planner import MotionPlanner
from SMP.motion_planner.plot_config import StudentScriptPlotConfig


def main():
    # configurations
    path_scenario = 'Scenarios/scenario3.xml'   #scenario path
    print("===================================")    
    file_motion_primitives = 'V_9.0_9.0_Vstep_0_SA_-0.2_0.2_SAstep_0.4_T_0.5_Model_BMW320i.xml'
    config_plot = StudentScriptPlotConfig(DO_PLOT=True) 
    # load scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(path_scenario).open()
    # retrieve the first planning problem
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    # create maneuver automaton and planning problem
    automaton = ManeuverAutomaton.generate_automaton(file_motion_primitives)

    # dictionary that contains the name of the algorithms and MotionPlanner 
    dict_motion_planners = {
        0: (MotionPlanner.Astar, "A* Search"),  
        1: (MotionPlanner.IterativeDeepeningAstar, "Iterative Deepening A* Search")
    }

    scenario_name = path_scenario[10:19]    # take the scenario name from the path
    fname = "OutputFiles/" + scenario_name + ".txt" # the filename in which the output is gonna be saved
    file = open(fname, "w") # open file for writing 
    print(scenario_name)   

    my_str = "<"+ scenario_name+ ">" +"\n===============================================================\n"
    file.write(my_str+"\n")
    print(my_str)
    file.close()

    for (class_planner, name_planner) in dict_motion_planners.values():
        planner = class_planner(scenario=scenario, planning_problem=planning_problem,
                                      automaton=automaton, plot_config=config_plot)

        # for differnet weights from 1 to 3 in our case 
        for w in range(1,4):
            # start search
            found_path = planner.execute_search(time_pause=0.01, weight = w, choose_heuristic = True,filename = fname)  # execute search with euclidean 
            found_path = planner.execute_search(time_pause=0.01, weight = w, choose_heuristic = False,filename = fname) # execute search with manhattan distance 

        file.close()

print('Done')

if __name__ == '__main__':
    main()
