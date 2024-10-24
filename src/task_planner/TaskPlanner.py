#!/usr/bin/env python
from unified_planning.shortcuts import *
import rospy

class TaskPlanner:
    def __init__(self):
        self.setTypes()
        self.setFluents()
        self.setActions()
        self.setProblem()
        self.addObjects()
        #self.setInitState()
        #self.setGoalState()

    def removeSpaces(self, string):
        return string.replace(" ", "")

    def setTypes(self):
        #Types
        self.types = {} #Map Obj Name - UserType Obj

        type_names_full = rospy.get_param("/task_planner/types/names")
        
        for type_name in type_names_full:
            type_name = self.removeSpaces(type_name)
            
            s = type_name.split("(")
            
            if(len(s) == 1):
                self.types[s[0]] = UserType(s[0])
            else:
                self.types[s[0]] = UserType(s[0], father=self.types[s[1][0:-1]])

    def setFluents(self):
        #Fluents
        self.fluents = {}
        
        fluents_names = rospy.get_param("/task_planner/fluents/names")
        
        for fluent_name in fluents_names:
            fluent_name = self.removeSpaces(fluent_name)
            
            fluent_type = rospy.get_param("/task_planner/fluents/"+fluent_name+"/type")
            fluent_type = self.removeSpaces(fluent_type)
            
            params = rospy.get_param("/task_planner/fluents/"+fluent_name+"/params")
            
            # Build the keyword arguments dynamically based on the number of parameters
            kwargs = {f'p{i}': self.types[self.removeSpaces(param)] for i, param in enumerate(params)}

            if(fluent_type == "BoolType"):
                self.fluents[fluent_name] = unified_planning.model.Fluent(fluent_name, BoolType(), **kwargs)

    def setActions(self):
        #Actions
        self.actions = {}
        
        actions_names = rospy.get_param("/task_planner/actions/names")
        
        for action_name in actions_names:
            action_name = self.removeSpaces(action_name)

            params = rospy.get_param("/task_planner/actions/"+action_name+"/params")
            preconditions = rospy.get_param("/task_planner/actions/"+action_name+"/preconditions")
            effect = rospy.get_param("/task_planner/actions/"+action_name+"/effect")

            #Parameters
            kwargs = {f'p{i}': self.types[self.removeSpaces(param)] for i, param in enumerate(params)}
            
            self.actions[action_name] = InstantaneousAction(action_name, **kwargs)
            p = []
            for i in range(0, len(params)):
                p.append(self.actions[action_name].parameter('p'+str(i)))
            
            #Preconditions
            for precond in preconditions:
                precond = self.removeSpaces(precond)
                #Deal with Not
                val = True
                if(precond[0] == '~'):
                    precond = precond[1:]
                    val = False
                
                #Isolate fluent name 
                elements = precond.split("(")

                elements[1] = elements[1][:-1] #Remove ")" at the end
                #Parameters of the fluent
                params_prec = elements[1].split(",")
                
                #Means there was only 1 param
                if (len(params_prec) == 1):
                    params_prec = elements[1]

                #Dynamic Parameters
                args = [p[int(param)] for param in params_prec]
                
                if(not val):
                    self.actions[action_name].add_precondition(Not(self.fluents[elements[0]](*args)))
                else:
                    self.actions[action_name].add_precondition(self.fluents[elements[0]](*args))
        
            #Effect
            for eff in effect:
                eff = self.removeSpaces(eff)

                #Deal with Not
                val = True
                if(eff[0] == '~'):
                    eff = eff[1:]
                    val = False
                
                #Isolate fluent name 
                elements = eff.split("(")
                elements[1] = elements[1][:-1] #Remove ")" at the end

                #Parameters of the fluent
                params_prec = elements[1].split(",")
                
                #Means there was only 1 param
                if (len(params_prec) == 1):
                    params_prec = elements[1]


                #Dynamic Parameters
                args = [p[int(param)] for param in params_prec]

                self.actions[action_name].add_effect(self.fluents[elements[0]](*args), val)
                
    def setProblem(self):
        self.problem = Problem('Problem')
        
        for f in self.fluents:
            self.problem.add_fluent(self.fluents[f], default_initial_value=False)

        for a in self.actions:
            self.problem.add_action(self.actions[a])

    def addObjects(self):
        
        Objects = []
        self.Objects_w_names = {}

        for type in self.types:
            type = self.removeSpaces(type)

            #For each type, get list of available objects and add them
            objs = rospy.get_param("/task_planner/objects/"+type)

            for o in objs:
                Objects.append(Object(o, self.types[type]))
                self.Objects_w_names[o] = Object(o, self.types[type])
     
        self.problem.add_objects(Objects)

    def ResetInitState(self):
        for fluent in self.problem.initial_values:
            # Reset each fluent to False (assuming it's a Boolean fluent)
            self.problem.set_initial_value(fluent, False)
            
    def setInitState(self, input_state=[]):

        self.ResetInitState()

        if(len(input_state) == 0 or
           (len(input_state) == 1 and len(input_state[0]) == 0)):
            init_state = rospy.get_param("/task_planner/init_state")
        else:
            init_state = input_state

        for s in init_state:
            s = self.removeSpaces(s)

            #Deal with Not
            val = True
            if(s[0] == '~'):
                s = s[1:]
                val = False
            
            #Isolate fluent name 
            elements = s.split("(")

            elements[1] = elements[1][:-1] #Remove ")" at the end
            #Parameters of the fluent
            params_prec = elements[1].split(",")
            
            #Means there was only 1 param
            if (len(params_prec) == 1):
                params_prec = [elements[1]]

            #Dynamic Parameters
            args = [self.Objects_w_names[obj] for obj in params_prec]
            
            self.problem.set_initial_value(self.fluents[elements[0]](*args), val)
            
    def setGoalState(self, input_state=[]):
        
        self.problem.clear_goals()

        if(len(input_state) == 0 or
           (len(input_state) == 1 and len(input_state[0]) == 0)):
            goal_state = rospy.get_param("/task_planner/goal_state")
        else:
            goal_state = input_state

        for s in goal_state:
            s = self.removeSpaces(s)

            #Deal with Not
            val = True
            if(s[0] == '~'):
                s = s[1:]
                val = False
            
            #Isolate fluent name 
            elements = s.split("(")

            elements[1] = elements[1][:-1] #Remove ")" at the end
            #Parameters of the fluent
            params_prec = elements[1].split(",")
            
            #Means there was only 1 param
            if (len(params_prec) == 1):
                params_prec = [elements[1]]

            #Dynamic Parameters
            args = [self.Objects_w_names[obj] for obj in params_prec]
            
            if(not val):
                self.problem.add_goal(Not(self.fluents[elements[0]](*args)))
            else:
                self.problem.add_goal(self.fluents[elements[0]](*args))

    def solve(self):
        
        with OneshotPlanner(problem_kind=self.problem.kind) as planner:
            result = planner.solve(self.problem)
            print("%s returned: %s" % (planner.name, result.plan))