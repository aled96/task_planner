#!/usr/bin/env python

import os
import time

#ROS
import rospy

from task_planner.srv import *
from task_planner.TaskPlanner import TaskPlanner


global task_planner_srv

# Callback for the ROS service
def solve_task_cb(req):
    global TaskPlanner

    task_planner_srv.setInitState(req.init_state)
    task_planner_srv.setGoalState(req.goal_state)

    pddl_solution = task_planner_srv.solve()

    print(pddl_solution)
    
    return SolveTaskResponse(pddl_solution)

    
def main():

    rospy.init_node('TaskPlannerNode', anonymous=True)

    # Create global TaskPlanner object
    global task_planner_srv
    task_planner_srv = TaskPlanner()

    # Define service provided
    srv = rospy.Service('solve_task', SolveTask, solve_task_cb)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
