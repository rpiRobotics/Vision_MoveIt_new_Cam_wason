#import cv2
#import cv2.aruco as aruco
import numpy as np


import time
import timeit
import rpi_abb_irc5

import sys
import copy
import rospy
import arm_controller_commander
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import rospkg
import numpy as np
from tf.transformations import *
from tf2_msgs.msg import TFMessage
import copy

import rpi_ati_net_ft
from CameraService import *


ft_threshold=[300,300,300,300,300,300]

if __name__ == '__main__':
    print "============ Starting setup"
    Force_Measurement = 0
    P,Q = CameraService()
   
 #   P=np.zeros((1,4))
   # Q=[0,0,0,0]
    '''
    if(Force_Measurement):
        if (len(sys.argv) < 2):
            raise Exception('IP address of ATI Net F/T sensor required')
        host=sys.argv[1]
        netft=rpi_ati_net_ft.NET_FT(host)
        netft.set_tare_from_ft()
        print netft.try_read_ft_http()

        netft.start_streaming()
        FTtime = []
        FTread = []
    '''
    Robot_Pos = []
    Robot_Joint = []
    armcontroller=arm_controller_commander.ARMControllerCommander()
    group=armcontroller.move_it_init()
    '''
    #Code to automatically add in scene object
    
    scene_pose=geometry_msgs.msg.PoseStamped()
    scene_pose.header.frame_id = "testbed"
    scene_pose.pose.orientation.z=-1.57
    scene_pose.pose.position.x=3.6
    scene_pose.pose.position.y=3.3
    scene_pose.pose.position.z=0
    scene_name="testbed"
    scene.add_mesh(scene_name,scene_pose,'./meshes/testbed_walls/testbed_walls.stl',size=(1,1,1))

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(1, 1, 1))


    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < 10) and not rospy.is_shutdown():
  # Test if the box is in attached objects
	attached_objects = scene.get_attached_objects([scene_name])
	is_attached = len(attached_objects.keys()) > 0

  # Test if the box is in the scene.
  # Note that attaching the box will remove it from known_objects
	is_known = scene_name in scene.get_known_object_names()

  # Test if we are in the expected state
	if (is_attached):
	    break

  # Sleep so that we give other threads time on the processor
	rospy.sleep(0.1)
	seconds = rospy.get_time()
    '''

    armcontroller.set_controller(4,1,ft_threshold)
   
        
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
        pose_target.orientation.x = Q[1]
        pose_target.orientation.y = Q[2]#0.707
        pose_target.orientation.z = Q[3]#0.707
        pose_target.orientation.w = Q[0]#qoa[3] #0#0
        pose_target.position.x = P[0][0]
        pose_target.position.y = P[0][1]#-2.02630600362
        pose_target.position.z = P[0][2] + 0.3
        group.set_pose_target(pose_target)

        '''
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = -0.02892
        pose_target.position.y = -1.79035
        pose_target.position.z = 0.914083
        pose_target.orientation.x = 0.426720076618065
        pose_target.orientation.y = 0.5339800423981502
        pose_target.orientation.z = -0.4531605121430878
        pose_target.orientation.w = 0.5722069911891644

        group.set_pose_target(pose_target)
        '''
        print 'Target 1:',pose_target
        
  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        
        plan1 = group.plan()
        #print plan1
        cnt = 0
        while( (not plan1.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 1"
            plan1 = group.plan()
            cnt = cnt+1
            
        raw_input("Press Enter to continue")    
        print "============ Executing plan1"
        group.execute(plan1)
        print 'Execution Finished.'
        
        ########## Vertical Path ############
        armcontroller.set_controller(4,0.2,ft_threshold)
        

        print "============ Printing robot Pose"
        print group.get_current_pose()  
        print "============ Generating plan 2"
        """pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.x = Q[1]
        pose_target.orientation.y = Q[2]#0.707
        pose_target.orientation.z = Q[3]#0.707
        pose_target.orientation.w = Q[0]#qoa[3] #0#0
        pose_target.position.x = P[0][0]
        pose_target.position.y = P[0][1]#-2.02630600362
        pose_target.position.z = P[0][2] + 0.2"""
        
        pose_target2 = copy.deepcopy(pose_target)
        pose_target2.position.z -= 0.45
        
        group.set_pose_target(pose_target2)
        
        print 'Target 2:',pose_target2
        
  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        time.sleep(3)
        plan2 = group.plan()
	print "Planned"
        cnt = 0
        while( (not plan2.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 2"
            plan2 = group.plan()
            cnt = cnt+1
            
        time.sleep(3)    
        print "============ Executing plan2"
        group.execute(plan2)
        print 'Execution Finished.'
        armcontroller.set_controller(4,0.4,[])
        
        
        print "============ Lift panel!"
        armcontroller.set_vacuum(1)
        
        print group.get_current_pose() 
        #rospy.sleep(3)
        pose_target3 = copy.deepcopy(pose_target)
        pose_target3.position.z += 0.25
         
        group.set_pose_target(pose_target3)
       
        print 'Target:',pose_target3
        
  ## Now, we call the planner to compute the plan
  ## and visualize it if successful
  ## Note that we are just planning, not asking move_group 
  ## to actually move the robot
        #time.sleep(3)
        plan3 = group.plan()
        print "Planned"
        cnt = 0
        #raw_input("Press Enter to continue")  
        while( (not plan3.joint_trajectory.points) and (cnt<3)):
            print "============ Generating plan 3"
            plan3 = group.plan()
            cnt = cnt+1
        print "post loop"
        time.sleep(1)    
        print "============ Executing plan3"
        group.execute(plan3)
        print 'Execution Finished.'
        
        
        '''
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
    '''

