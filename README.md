# Task Planner
Use PDDL to plan for a sequence of actions to execute.
The implementation allows to create a ROS server that instantiate a PDDL problem and allows to solve it.

The definition of the problem can be fully customized by configuration file (config/task_planner_params.yaml) so that it can be changed, also online according to the problem and requirements.

Furthermore, functions are available to set initial state and goal state in order to change the planning conditions online based on the real state and desired task.

# Dependencies
- Unified-Planning (https://unified-planning.readthedocs.io/en/latest/index.html)
- ROS

# Congifuration File
The configuration file enables to define all the elements PDDL's problem.

In more detail:
- __Types__: List of object Types custom defined. Put in parenthesis the father. i.e. Tool(Obj)
- __Objects__: For each type defined, list the existing objects/elements
- __Fluents__:
    - __names__: list of fluents available
    - __type__: Currently, only __BoolType__ is available
    - __params__: list of params types (using the types described before)
- __Actions__: 
    - __names__: list of actions available
    - __params__: list of params types (using the types described before)
    - __preconditions__: list of fluents with parameters. Here parameters are identified with integers being the indices of the array __params__. Example: params: [Robot, Obj]  precondition: ["grasped(0,1)"]. "~" to be used before the fluent name to specify negation.
    - __effect__: the definition is the same as preconditions.
- __init_state__: list of fluent using as parameter the Objects' names. Example: requires_tool(Nail,Hammer)
- __goal_state__: list of fluent using as parameter the Objects' names

__NOTE__:
Each element of: __preconditions__, __effect__, __init_state__, __goal_state__ has to be defined within " ".


# Contributor
* Alessio De Luca 2024,  email: <alessio.deluca.iic96d@gmail.com> 