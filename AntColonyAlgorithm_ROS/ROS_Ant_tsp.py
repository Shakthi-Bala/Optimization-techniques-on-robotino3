#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  9 00:16:30 2021

@author: wellpast
"""
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ant_tsp_12MAY import *
from time import sleep

if __name__ == '__main__':
    _colony_size = 10
    _steps = 50
    cities = []
    tp_number = int(input("Enter the number of points to go? :"))
    
    
    
    print("Enter destination points to go as x and y respectively.")
    for i in range(tp_number):
        if i<1:
            
            print("****First enter the robot's starting location on the map?****")
            x_co = float(input("Robot starting point x = [{}][0] = ".format(i)))
            y_co = float(input("Robot starting point y =[{}][1] = ".format(i)))
            cities.append([x_co,y_co])
        else:
            print("\n***********\n {}. Point: Enter Destination\n***********".format(i))
            x_co = float(input("{}. Target Point = x[{}][0] = ".format(i,i)))
            y_co = float(input("{}. Target Point = y[{}][1] = ".format(i,i)))
            cities.append([x_co,y_co])
            
    print("Destination points to go are as follows:",cities)
    print("\n\n\n**************TRAVELLING SALESMAN PROBLEM**************")
    _nodes= []
    _nodes = cities
    acs = SolveTSPUsingACO(mode='ACS', colony_size=_colony_size, steps=_steps, nodes=_nodes)
    acs.run()
    acs.plot()
    elitist = SolveTSPUsingACO(mode='Elitist', colony_size=_colony_size, steps=_steps, nodes=_nodes)
    elitist.run()
    elitist.plot()
    max_min = SolveTSPUsingACO(mode='MaxMin', colony_size=_colony_size, steps=_steps, nodes=_nodes)
    max_min.run()
    max_min.plot()
    


solution_points = []
solution_points = max_min.array 
print("\n*\n*\nAnt Colony - Solution set of TSP points respectively : ",solution_points)  


for i in range(tp_number+1):
    print("{}. destination coordinates x={} y={}".format(i,solution_points[i][0],solution_points[i][1]))

for k in range(tp_number+1):
    if k == 0:
        print("Robot is moving...")
    else:
        print("\nThe new route is being set.\n")
        sleep(2)
        x=solution_points[k][0] 
        y=solution_points[k][1]
        print("Location points of the target: x=",x,"y=",y)
        
        def movebase_client():
                client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                client.wait_for_server()
                target = MoveBaseGoal()
                target.target_pose.header.frame_id = "map"
                target.target_pose.pose.position.x = x 
                target.target_pose.pose.position.y = y
                target.target_pose.pose.orientation.w = 1.0
                client.send_goal(target)
                wait = client.wait_for_result()
                if not wait:
                    rospy.signal_shutdown("No Action Service!")
                else:
                    return client.get_result()
        if __name__ == '__main__':
                try:
                    rospy.init_node('Move_Target')
                    result = movebase_client()
                    if result:
                         rospy.loginfo("Reached at the target point.!")
                         x_guncel=x
                         y_guncel=y
                    else:
                         rospy.loginfo("Going to target point......")
                         
                except rospy.ROSInterruptException:
                    pass
                
