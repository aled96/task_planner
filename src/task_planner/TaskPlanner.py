#!/usr/bin/env python
from unified_planning.shortcuts import *

class TaskPlanner:
    def __init__(self):
        self.setTypes()
        self.setFluents()
        self.setActions()
        self.setProblem()
        self.addObjects()
        self.setInitState()
        self.setGoalState()

    def setTypes(self):
        #Types
        self.Obj = UserType('Obj')
        self.Tool = UserType('Tool', father = self.Obj)
        self.Robot = UserType('Robot')

    def setFluents(self):
        #Fluents
        self.grasped = unified_planning.model.Fluent('grasped', BoolType(), r=self.Robot, o=self.Obj)
        self.is_tight = unified_planning.model.Fluent('is_tight', BoolType(), o=self.Obj)
        self.requires_tool = unified_planning.model.Fluent('requires_tool', BoolType(), o=self.Obj, t= self.Tool)
        self.is_turnable = unified_planning.model.Fluent('is_turnable', BoolType(), o=self.Obj)
        self.free_hands = unified_planning.model.Fluent('free_hands', BoolType(), r=self.Robot)
        self.can_tight = unified_planning.model.Fluent('can_tight', BoolType(), o=self.Obj)
        self.robot_at = unified_planning.model.Fluent('robot_at', BoolType(), r=self.Robot, o=self.Obj)
        
    def setActions(self):
        #Grasp
        self.grasp = InstantaneousAction('grasp', r=self.Robot, o=self.Obj)
        r = self.grasp.parameter('r')
        o = self.grasp.parameter('o')

        self.grasp.add_precondition(self.robot_at(r, o))
        self.grasp.add_precondition(self.free_hands(r))
        self.grasp.add_precondition(Not(self.grasped(r, o)))

        self.grasp.add_effect(self.grasped(r, o), True)
        self.grasp.add_effect(self.free_hands(r), False)

        #Put Down
        self.put_down = InstantaneousAction('put_down', r=self.Robot, o=self.Obj)
        r = self.put_down.parameter('r')
        o = self.put_down.parameter('o')

        #Need to better specify where, now it's "go to Object and leave Object", no sense
        self.put_down.add_precondition(self.robot_at(r, o))
        self.put_down.add_precondition(Not(self.free_hands(r)))
        self.put_down.add_precondition(self.grasped(r, o))

        self.put_down.add_effect(self.grasped(r, o), False)
        self.put_down.add_effect(self.free_hands(r), True)


        #Tight
        self.tight = InstantaneousAction('tight', r=self.Robot, t=self.Tool, o=self.Obj)
        r = self.tight.parameter('r')
        t = self.tight.parameter('t')
        o = self.tight.parameter('o')

        self.tight.add_precondition(self.robot_at(r, o))
        self.tight.add_precondition(self.can_tight(t))
        self.tight.add_precondition(self.grasped(r, t))
        self.tight.add_precondition(self.requires_tool(o, t))
        self.tight.add_precondition(Not(self.is_tight(o)))

        self.tight.add_effect(self.is_tight(o), True)

        #Turn
        self.turn = InstantaneousAction('turn', r=self.Robot, o=self.Obj)
        r = self.turn.parameter('r')
        o = self.turn.parameter('o')

        self.turn.add_precondition(self.robot_at(r, o))
        self.turn.add_precondition(self.is_turnable(o))
        self.turn.add_precondition(self.grasped(r, o))
        self.turn.add_precondition(Not(self.is_tight(o)))

        self.turn.add_effect(self.is_tight(o), True)

        #Move
        self.move = InstantaneousAction('move', r=self.Robot, start=self.Obj, end=self.Obj)
        r = self.move.parameter('r')
        start = self.move.parameter('start')
        end = self.move.parameter('end')

        self.move.add_precondition(self.robot_at(r, start))

        self.move.add_effect(self.robot_at(r, start), False)
        self.move.add_effect(self.robot_at(r, end), True)
        

    def setProblem(self):
        self.problem = Problem('Problem')
        self.problem.add_fluent(self.grasped, default_initial_value=False)
        self.problem.add_fluent(self.is_tight, default_initial_value=False)
        self.problem.add_fluent(self.free_hands, default_initial_value=False)
        self.problem.add_fluent(self.can_tight, default_initial_value=False)
        self.problem.add_fluent(self.is_turnable, default_initial_value=False)
        self.problem.add_fluent(self.requires_tool, default_initial_value=False)
        self.problem.add_fluent(self.robot_at, default_initial_value=False)

        self.problem.add_action(self.grasp)
        self.problem.add_action(self.put_down)
        self.problem.add_action(self.tight)
        self.problem.add_action(self.turn)
        self.problem.add_action(self.move)

    def addObjects(self):
        self.Objects = [Object("Centauro", self.Robot),
                        Object("Screwdriver", self.Tool),
                        Object("Screw", self.Obj),
                        Object("Valve", self.Obj)]

        self.problem.add_objects(self.Objects)

    def setInitState(self):
        self.problem.set_initial_value(self.is_tight(self.Objects[2]), False)
        self.problem.set_initial_value(self.free_hands(self.Objects[0]), True)
        self.problem.set_initial_value(self.can_tight(self.Objects[1]), True)
        self.problem.set_initial_value(self.is_turnable(self.Objects[3]), True)
        self.problem.set_initial_value(self.requires_tool(self.Objects[2], self.Objects[1]), True)
        self.problem.set_initial_value(self.robot_at(self.Objects[0], self.Objects[2]), True)

    def setGoalState(self):
        self.problem.add_goal(self.is_tight(self.Objects[2]))
        self.problem.add_goal(self.is_tight(self.Objects[3]))


    def solve(self):
        with OneshotPlanner(problem_kind=self.problem.kind) as planner:
            result = planner.solve(self.problem)
            print("%s returned: %s" % (planner.name, result.plan))