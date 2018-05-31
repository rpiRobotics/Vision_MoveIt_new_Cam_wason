import cv2
import cv2.aruco as aruco
import numpy as np


import time
import timeit
import rpi_abb_irc5

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import rospkg
import numpy as np
from tf.transformations import *
from tf2_msgs.msg import TFMessage

import rpi_ati_net_ft

def Map_Camera2Robot(Pca,Rca):
    M = [[0.001079456145327,   0.000089969321948,   0.000188329763132], 
         [0.000036782394649,  -0.000451661936652,   0.000096581780769], 
         [-0.000014377904855,   0.000257933701523,  -0.000906958561208], 
         [0.440496549972441,  -2.894868321199681,   2.915728528963717]]
         
    Mirror = [[-1,0,0],[0,-1,0],[0,0,1]];
    Rca = np.matmul(Rca,Mirror)
    #R = [[0,-1,0],[1,0,0],[0,0,1]]
    R = [[-0.0396,-0.9989,0.0243],[0.9992,-0.0394,0.0073],[-0.0063,0.0246,0.9997]]
   
    Poa = np.matmul(np.vstack((Pca,1)).T,M)
    Roa = np.matmul(Rca,R)
    
    return [Poa, Roa]

def get_object_pose(corners, tag_size, camMatrix, distCoeff):
    # AR Tag Dimensions
    objPoints = np.zeros((4, 3), dtype=np.float64)
    objPoints[0,0] = -1*tag_size/2.0 
    objPoints[0,1] = tag_size/2.0 
    objPoints[0,2] = 0.0
    objPoints[1,0] = tag_size/2.0 
    objPoints[1,1] = tag_size/2.0 
    objPoints[1,2] = 0.0
    objPoints[2,0] = tag_size/2.0 
    objPoints[2,1] = -1*tag_size/2.0 
    objPoints[2,2] = 0.0
    objPoints[3,0] = -1*tag_size/2.0 
    objPoints[3,1] = -1*tag_size/2.0 
    objPoints[3,2] = 0.0

    # Get each corner of the tags
    imgPoints = np.zeros((4, 2), dtype=np.float64)
    for i in range(4):
        imgPoints[i, :] = corners[0, i, :]

    # SolvePnP
    retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, camMatrix, distCoeff)
    Rca, b = cv2.Rodrigues(rvec)
    Pca = tvec

    return [Pca, Rca]

def get_object_pose_M(corners, tag_size, tag_distance_H, tag_distance_V, camMatrix, distCoeff):
    # AR Tag Dimensions
    d = 65.9
    objPoints = np.zeros((16, 3), dtype=np.float64)
    objPoints[0,0] = -1*tag_size/2.0 
    objPoints[0,1] = tag_size/2.0 
    objPoints[0,2] = 0.0
    objPoints[1,0] = tag_size/2.0 
    objPoints[1,1] = tag_size/2.0 
    objPoints[1,2] = 0.0
    objPoints[2,0] = tag_size/2.0 
    objPoints[2,1] = -1*tag_size/2.0 
    objPoints[2,2] = 0.0
    objPoints[3,0] = -1*tag_size/2.0 
    objPoints[3,1] = -1*tag_size/2.0 
    objPoints[3,2] = 0.0
    objPoints[4,0] = -1*tag_size/2.0 + tag_distance_H 
    objPoints[4,1] = tag_size/2.0 
    objPoints[4,2] = 0.0
    objPoints[5,0] = tag_size/2.0 + tag_distance_H 
    objPoints[5,1] = tag_size/2.0
    objPoints[5,2] = 0.0
    objPoints[6,0] = tag_size/2.0 + tag_distance_H
    objPoints[6,1] = -1*tag_size/2.0 
    objPoints[6,2] = 0.0
    objPoints[7,0] = -1*tag_size/2.0 + tag_distance_H
    objPoints[7,1] = -1*tag_size/2.0 
    objPoints[7,2] = 0.0
    objPoints[8,0] = -1*tag_size/2.0 
    objPoints[8,1] = tag_size/2.0 - tag_distance_V
    objPoints[8,2] = 0.0
    objPoints[9,0] = tag_size/2.0 
    objPoints[9,1] = tag_size/2.0 - tag_distance_V
    objPoints[9,2] = 0.0
    objPoints[10,0] = tag_size/2.0 
    objPoints[10,1] = -1*tag_size/2.0 - tag_distance_V
    objPoints[10,2] = 0.0
    objPoints[11,0] = -1*tag_size/2.0 
    objPoints[11,1] = -1*tag_size/2.0 - tag_distance_V
    objPoints[11,2] = 0.0
    objPoints[12,0] = -1*tag_size/2.0 + tag_distance_H 
    objPoints[12,1] = tag_size/2.0 - tag_distance_V +1
    objPoints[12,2] = 0.0
    objPoints[13,0] = tag_size/2.0 + tag_distance_H 
    objPoints[13,1] = tag_size/2.0 - tag_distance_V
    objPoints[13,2] = 0.0
    objPoints[14,0] = tag_size/2.0 + tag_distance_H 
    objPoints[14,1] = -1*tag_size/2.0 - tag_distance_V
    objPoints[14,2] = 0.0
    objPoints[15,0] = -1*tag_size/2.0 + tag_distance_H
    objPoints[15,1] = -1*tag_size/2.0 - tag_distance_V
    objPoints[15,2] = 0.0    

    # SolvePnP
    retVal, rvec, tvec = cv2.solvePnP(objPoints, corners, camMatrix, distCoeff)
    Rca, b = cv2.Rodrigues(rvec)
    Pca = tvec

    return [Pca, Rca]


if __name__ == '__main__':
    print "============ Starting setup"
    Force_Measurement = 0
    if(Force_Measurement):
        if (len(sys.argv) < 2):
            raise Exception('IP address of ATI Net F/T sensor required')
        host=sys.argv[1]
        netft=rpi_ati_net_ft.NET_FT(host)
        netft.set_tare_from_ft()
        print netft.read_ft_http()

        netft.start_streaming()
        FTtime = []
        FTread = []
    
    Robot_Pos = []
    Robot_Joint = []
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('collision_checker','move_group_python_interface_tutorial',
                  anonymous=True)

  ## MoveIt! Initialization
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("move_group")
    group.set_goal_position_tolerance(0.04)
    group.allow_replanning(True)
    #group.set_planner_id("KPIECEkConfigDefault") #RRTConnectkConfigDefault/SBLkConfigDefault/KPIECEkConfigDefault/BKPIECEkConfigDefault/LBKPIECEkConfigDefault/
    group.set_num_planning_attempts(5)
    display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)    
    rospy.sleep(2)

    print "============ Printing robot Pose"
    print group.get_current_pose().pose

            

    tic = timeit.default_timer()
    dt = 0
    while dt< 3:
        toc = timeit.default_timer()
        dt = toc - tic
    print 'Start'
       

    if (1):

    
        print "============ Printing robot Pose"
        print group.get_current_pose()  
        #print robot.get_current_state().joint_state.position
        print "============ Generating plan 1"
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = 0.707#0
        pose_target.orientation.y = 0.707#0.707
        pose_target.orientation.z = 0#0.707
        pose_target.orientation.w = 0.05759#qoa[3] #0#0
        pose_target.position.x = 0.288#Poa[0][0]
        pose_target.position.y =-2#Poa[0][1]#-2.02630600362
        pose_target.position.z =1.5
        group.set_pose_target(pose_target)
        
        print pose_target

  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        
        plan1 = group.plan()
        print plan1
        cnt = 0
        while( (not plan1.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 1"
            plan1 = group.plan()
            cnt = cnt+1
            
        time.sleep(5)    
        print "============ Executing plan1"
        group.execute(plan1)
        print 'Execution Finished.'
        
        tic = timeit.default_timer()
        dt = 0
        while dt< 3:
            toc = timeit.default_timer()
            dt = toc - tic
            a_pos = group.get_current_pose().pose.position
            a_ori = group.get_current_pose().pose.orientation
            a_joint = robot.get_current_state().joint_state.position
            
            if(Force_Measurement):
                s = netft.try_read_ft_streaming(.1)
                FTread.append(s[1])
                FTtime.append(time.time())
		        
            Robot_Pos.append([a_pos.x, a_pos.y, a_pos.z,a_ori.x, a_ori.y, a_ori.z,a_ori.w])
            Robot_Joint.append(a_joint[0:6])
            
    # When everything done, release the capture
    if (Force_Measurement):
                np.savetxt('FTtime.out', FTtime, delimiter=', ')
                np.savetxt('FTread.out', FTread, delimiter=', ')
    np.savetxt('Robot_Pos.out', Robot_Pos, delimiter=', ')
    np.savetxt('Robot_Joint.out', Robot_Joint, delimiter=', ')
    

