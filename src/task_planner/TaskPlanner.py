#!/usr/bin/env python
from unified_planning.shortcuts import *

class TaskPlanner:
    def __init__(self):
        #Types
        Obj = UserType('Obj')
        #Tool = UserType('Tool')
        Robot = UserType('Robot')

        #Fluents
        grasped = unified_planning.model.Fluent('grasped', BoolType(), r=Robot, o=Obj)
        is_tight = unified_planning.model.Fluent('is_tight', BoolType(), o=Obj)
        free_hands = unified_planning.model.Fluent('free_hands', BoolType(), r=Robot)
        can_tight = unified_planning.model.Fluent('can_tight', BoolType(), o=Obj)

        #Actions
        #Grasp
        grasp = InstantaneousAction('grasp', r=Robot, o=Obj)
        r = grasp.parameter('r')
        o = grasp.parameter('o')

        grasp.add_precondition(free_hands(r))
        grasp.add_precondition(Not(grasped(r, o)))

        grasp.add_effect(grasped(r, o), True)
        grasp.add_effect(free_hands(r), False)

        #Tight
        tight = InstantaneousAction('tight', r=Robot, t=Obj, o=Obj)
        r = tight.parameter('r')
        t = tight.parameter('t')
        o = tight.parameter('o')

        tight.add_precondition(can_tight(t))
        tight.add_precondition(grasped(r, t))
        tight.add_precondition(Not(is_tight(o)))

        tight.add_effect(is_tight(o), True)

        #Problem
        self.problem = Problem('Problem')
        self.problem.add_fluent(grasped, default_initial_value=False)
        self.problem.add_fluent(is_tight, default_initial_value=False)
        self.problem.add_fluent(free_hands, default_initial_value=False)
        self.problem.add_fluent(can_tight, default_initial_value=False)

        self.problem.add_action(grasp)
        self.problem.add_action(tight)

        #Add Objects
        Objects = [Object("Centauro", Robot),
                Object("Screwdriver", Obj),
                Object("Screw", Obj)]

        self.problem.add_objects(Objects)

        #Initial State
        self.problem.set_initial_value(is_tight(Objects[2]), False)
        self.problem.set_initial_value(free_hands(Objects[0]), True)
        self.problem.set_initial_value(can_tight(Objects[1]), True)

        #Add goal
        self.problem.add_goal(is_tight(Objects[2]))
        #problem.add_goal(grasped(Objects[0], Objects[1]))

        # print(self.problem)

    def solve(self):
        with OneshotPlanner(problem_kind=self.problem.kind) as planner:
            result = planner.solve(self.problem)
            print("%s returned: %s" % (planner.name, result.plan))