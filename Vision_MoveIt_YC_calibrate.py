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
from CameraService import *



if __name__ == '__main__':
    print "============ Starting setup"
    Force_Measurement = 1
    #P,Q = CameraService()

    
    if(Force_Measurement):
        if (len(sys.argv) < 2):
            raise Exception('IP address of ATI Net F/T sensor required')
        host=sys.argv[1]
        netft=rpi_ati_net_ft.NET_FT(host)
        netft.set_tare_from_ft()
        print netft.try_read_ft_http()

        netft.start_streaming()
        FTtime = timeit.default_timer()
        FTread = netft.try_read_ft_streaming(.1)[1]
    
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
    Pose = group.get_current_pose().pose
    Joint = robot.get_current_state().joint_state.position
    FTread = netft.try_read_ft_streaming(.1)[1]

    print Joint
    print Pose
    print FTtime
    print FTread

    f_handle = file('YC_FTread.txt', 'a')
    np.savetxt(f_handle, FTread)
    f_handle.close()        

    f_handle = file('YC_Joint.txt', 'a')
    np.savetxt(f_handle, Joint)
    f_handle.close() 

    f_handle = file('YC_Position.txt', 'a')
    np.savetxt(f_handle, np.array([Pose.position.x, Pose.position.y, Pose.position.z]))
    f_handle.close() 

    f_handle = file('YC_Orientation.txt', 'a')
    np.savetxt(f_handle, np.array([Pose.orientation.x, Pose.orientation.y, Pose.orientation.z, Pose.orientation.w]))
    f_handle.close() 
    '''
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
        pose_target.orientation.x = Q[1]
        pose_target.orientation.y = Q[2]#0.707
        pose_target.orientation.z = Q[3]#0.707
        pose_target.orientation.w = Q[0]#qoa[3] #0#0
        pose_target.position.x = P[0][0]
        pose_target.position.y = P[0][1]#-2.02630600362
        pose_target.position.z = P[0][2] + 0.35
        group.set_pose_target(pose_target)
        
        print 'Target:',pose_target

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
    '''
        
