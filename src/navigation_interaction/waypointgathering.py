#!/usr/bin/env python
# 2023 Kat Allen kat.allen@tufts.edu 

import rospy 
from sensor_msgs.msg import JointState
from kinova_msgs.msg import JointAngles
from kinova_msgs.msg import JointTorque
import numpy as np
import csv
import armpy
import armpy.arm
import armpy.gripper
import pickle
import os
arm = armpy.arm.Arm()
start_force_control=arm.start_force_control
stop_force_control= arm.stop_force_control


def gather_waypoints(outputfile='waypoints.csv'):
    # this CSV file should exist with these field names/headers
    # but will be created by the writer if it does not exist

#    rospy.init_node("waypointgathering")
    fieldnames=['positionname', "j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7" ]

    
    morepoints = True # set up the loop
    all_points=[]

    while morepoints ==True:
        # get the name of the position
        print("Enter the name of the joint position for the waypoint file, e.g. Triangle 1\n")
        positionname = input()

        # put the robot into kinetic teaching mode so the human can manipulate it
        start_force_control()
        
        print("Kinetic teaching mode engaged. \nMove the arm to the location you want to capture and then press enter\n")

        proceed = str(input())

        # get the robot out of KT mode
        stop_force_control()

        #get joint configurations at current position
        message = rospy.wait_for_message("joint_states", JointState)

        # create a dictionary of the joint positions by parsing the message from ROS
        joint_vals = {name:val for name,val in zip(message.name, message.position)}

        joint_data = [joint_vals[name] for name in fieldnames[1:]]
        all_points.append([positionname, *joint_data])

        print("closed the csv. Waypoint", positionname , "saved\n")

        # check if we should loop
        print("If you are done entering waypoints (to quit the program), press q\n")
        print("To enter more positions/waypoints, press any other key\n")
        loopcheck = input()
        if loopcheck == "q":
            morepoints = False # so the loop will quit
            print("Waypoint Gathering Complete\n")
        else: 
            morepoints = True # loop for more points

    with open(outputfile, 'a') as f: 
        writer_object = csv.writer(f, delimiter=',')
        writer_object.writerow(fieldnames)
        # write joints 1-7 from the message to the CSV, for all of the points
        for points in all_points:	
            writer_object.writerow(points)



def create_trajectory_from_waypoints(filename="waypoints.csv", arm=arm):
    # filename should be a CSV file, formatted like waypointgathering.py
    print("Loading waypoints from", filename)

    try:
        with open(filename, 'r') as f:
            filelist = csv.DictReader(f)    
            data = [row for row in filelist]
    except:
        print("No such file name", filename)
        return
    jointnames = ["j2s7s300_joint_1", "j2s7s300_joint_2", "j2s7s300_joint_3", "j2s7s300_joint_4", "j2s7s300_joint_5", "j2s7s300_joint_6", "j2s7s300_joint_7"]
    # set velocity so for playback - can not be changed at run
    print("Set velocity (0-1, .2 is default)")
    velset = float(input())
    
    arm.set_velocity(velset)

    print("Select the points to use in the trajectory")
    positionname_list = []
    for row in data:
        positionname_list.append(row['positionname'])
        
    print(positionname_list)
    # loop
    morepoints = True # init
    waypointlist = []    
    while morepoints ==True:
        # get point name
        point = str(input())
        # check if point is in the list data,
        #  [{'positionname': 'beaker5', 'j2s7s300_joint_1': '3.2092276242139905', 'j2s7s300_joint_2': '3.6030092808958316', 'j2s7s300_joint_3': '7.44210992295606', 'j2s7s300_joint_4': '0.8380117736301355', 'j2s7s300_joint_5': '0.0038275429723377183', 'j2s7s300_joint_6': '2.20180744653298', 'j2s7s300_joint_7': '-0.4488199086394775'}]

        for row in data:
            if row['positionname']==point:
                # if it is, add to waypointlist
                print("point found, adding to trajectory")
                goodrow = row.copy() # make a copy disconnected from original
                
                # position name has to be removed from the dictionary
                # of joint positions before it is passed to armpy
                del goodrow['positionname']
                print("row", row, "\n")
                print("goodrow", goodrow, "\n")
                # values need to be converted to floats for armpy
                for k, v in goodrow.items():
                    goodrow[k] = float(v)
                waypointlist.append(goodrow)
                print("New waypoint", goodrow, "added to trajectory \n")
                print("waypoints so far", waypointlist,"\n")
                break # stop looking, we found it
            else:
                print("not in this row")
        print("Get another point? n to generate trajectory,\n any other key to select another point")
        morepointq = input()
        if morepointq=="n":
            morepoints=False
            print("no more points \n")
        else:
            print(positionname_list)
    
    print("Generating trajectories from waypoints:\n")
    print("waypoint list is ", waypointlist, "\n")

    trajectory = arm.plan_joint_waypoints(waypointlist)
       
    # now save the trajectory out to a file so we can load it later
    print("name this trajectory/pickle filename")
    trajectoryname = input()
    picklepath = 'trajectorypickles/'+trajectoryname+'.pkl'
    exists = os.path.isfile(picklepath)
    if exists!=True:
        open(picklepath, 'w+').close()
    with open(picklepath, "wb") as f:
        pickle.dump(trajectory, f)

    print("press enter to try out the trajectory")
    input()
    for i, plan in enumerate(trajectory):
        print("executing plan", i) 
        arm.move_robot(plan)


if __name__=="__main__":
# might need this 
	#armpy.move_to_point(jointpositionlist) # move to the joint position, defined as a 7dof list
    rospy.init_node("waypointgathering")
    gripper = armpy.gripper.Gripper()
    arm = armpy.arm.Arm()

    print("Gather trajectories using hapticstudylibrary.py")
    #waypoints2trajectories("waypoints.csv")
