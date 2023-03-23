from typing import Union
from enum import Enum, unique
import sys

from commonroad.scenario.scenario import Scenario
from commonroad.planning.planning_problem import PlanningProblem

from SMP.motion_planner.plot_config import DefaultPlotConfig
from SMP.maneuver_automaton.maneuver_automaton import ManeuverAutomaton
sys.path.append('../../')
from Algorithms.DFS_example import DepthFirstSearch
from Algorithms.Astar import Astar
from Algorithms.IDAstar import IterativeDeepeningAstar


@unique
class MotionPlannerType(Enum):
    """
    Enumeration definition of different algorithms.
    """
    DFS = "dfs"
    IDASTAR = "idastar"
    ASTAR = "astar"


class MotionPlanner:
    """
    Class to load and execute the specified motion planner.
    """

    class NoSuchMotionPlanner(KeyError):
        """
        Error message when the specified motion planner does not exist.
        """

        def __init__(self, message):
            self.message = message

    dict_motion_planners = dict()
    dict_motion_planners[MotionPlannerType.DFS] = DepthFirstSearch
    dict_motion_planners[MotionPlannerType.ASTAR] = Astar
    dict_motion_planners[MotionPlannerType.IDASTAR] = IterativeDeepeningAstar

    @classmethod
    def create(cls, scenario: Scenario, planning_problem: PlanningProblem, automaton: ManeuverAutomaton,
               plot_config=DefaultPlotConfig,
               motion_planner_type: MotionPlannerType = MotionPlannerType.DFS) -> Union[Astar,
                                                                                         DepthFirstSearch,
                                                                                         IterativeDeepeningAstar]:
        """
        Method to instantiate the specified motion planner.
        """
        try:
            return cls.dict_motion_planners[motion_planner_type](scenario, planning_problem, automaton,
                                                                 plot_config=plot_config)
        except KeyError:
            raise cls.NoSuchMotionPlanner(f"MotionPlanner with type <{motion_planner_type}> does not exist.")

    @classmethod
    def Astar(cls, scenario: Scenario, planning_problem: PlanningProblem, automaton: ManeuverAutomaton,
                           plot_config=DefaultPlotConfig) -> Astar:
        """
        Method to instantiate a Astar Search motion planner.
        """
        return MotionPlanner.create(scenario, planning_problem, automaton, plot_config, MotionPlannerType.ASTAR)


    @classmethod
    def DepthFirstSearch(cls, scenario: Scenario, planning_problem: PlanningProblem, automaton: ManeuverAutomaton,
                         plot_config=DefaultPlotConfig) -> DepthFirstSearch:
        """
        Method to instantiate a Depth-First-Search motion planner.
        """
        return MotionPlanner.create(scenario, planning_problem, automaton, plot_config, MotionPlannerType.DFS)

    @classmethod
    def IterativeDeepeningAstar(cls, scenario: Scenario, planning_problem: PlanningProblem, automaton: ManeuverAutomaton,
                    plot_config=DefaultPlotConfig) -> IterativeDeepeningAstar:
        """
        Method to instantiate an IDA* Search motion planner.
        """
        return MotionPlanner.create(scenario, planning_problem, automaton, plot_config, MotionPlannerType.IDASTAR)

