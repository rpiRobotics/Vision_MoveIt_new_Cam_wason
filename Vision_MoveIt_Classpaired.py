#import cv2
#import cv2.aruco as aruco
import numpy as np


import time
import timeit
import rpi_abb_irc5
import Camera_Class
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



'''
class MovementPlan:
    def __init__(self,move_group,pose_target):
        self.group=move_group
	self.pose_target=pose_target

    def create_plan(self):

    def load_seed_plan(self):

    
'''	
  
class VisionMoveIt:
    def __init__(self,arm_controller_commander):
	self.ft_threshold=[250,250,250,250,250,250]
	self.Force_Measurement=0
	self.pose_target=0
	#could also be inserted into the starting section of control code for better visibility
	self.armcontroller=arm_controller_commander
	self.pose_targets=[]
	self.plans=[]
	self.Cameras=Camera_Class.Camera_Class()

    def camera_read(self):
	
		self.P,self.Q=self.Cameras.CameraService()

    def moveit_init(self):
	
		self.group=self.armcontroller.move_it_init()

    #ROS service calls to check system variables
    def set_positions(self):
		print "============ Printing robot Pose"
		print self.group.get_current_pose().pose
		Q=self.Q
		P=self.P
	
		tic = timeit.default_timer()
		dt = 0
		while dt< 3:
			toc = timeit.default_timer()
			dt = toc - tic

		print 'Start'
	
		print "============ Printing robot Pose"
		print self.group.get_current_pose()
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
		self.pose_targets.append(pose_target)
		#self.group.set_pose_target(pose_target)
	

		#print "============ Printing robot Pose"
		#print group.get_current_pose()  
		#print "============ Generating plan 2"
		pose_target2 = copy.deepcopy(pose_target)
		pose_target2.position.z -= 0.45
		self.pose_targets.append(pose_target2)
		#print "============ Generating plan 3"
		pose_target3 = copy.deepcopy(pose_target)
		pose_target3.position.z += 0.25
		self.pose_targets.append(pose_target3)

    def generate_plan(self,plan_num):
	#done so that the first position the robot will move to is shown first
	#downside to this approach is that Rviz will not automatically show latest plan, unless 
	#in the execute function we add in a command to load in the previously generated plan to
	#vizualize it again
	#i=len(self.pose_targets)
		group=self.group
		pose_targets=self.pose_targets
	
		group.set_pose_target(pose_targets[plan_num-1])
		plan = group.plan()
            #print plan
		cnt = 0
		while( (not plan.joint_trajectory.points) and (cnt<3)):
			print "============ Generating plan "+ str(i)
			plan = group.plan()
			cnt = cnt+1

		return plan
        #self.plans.insert(0,plan)
	#i-=1

    def execute_plans(self,plan):
	#eventually might like a for loop here if speeds are more consistent, or could use switch statement to call set controller
	'''
	display_trajectory.trajectory_start=self.group.get_current_pose()
	display_trajectory.trajectory.append(self.plans[plan_num])
	self.armcontroller.display_trajectory_publisher.publish(display_trajectory)
	'''
	#print self.plans[plan_num]        
	#print "============ Executing plan "+str(plan_num)
        self.group.execute(plan)
        print 'Execution Finished.'

    def reset_pos(self):
		P = [[ 1.8288, -0.0447, 1.237]]
		Q = [0.718181636243,-0.0836401543762,0.687115714468,0.0713544453462]
		self.armcontroller.set_controller(4,0.5,[])
		print self.group.get_current_pose()  
        #print robot.get_current_state().joint_state.position
		print "============ Generating plan 1"
		pose_target = geometry_msgs.msg.Pose()
		pose_target.orientation.x = Q[1]
		pose_target.orientation.y = Q[2]
		pose_target.orientation.z = Q[3]
		pose_target.orientation.w = Q[0]
		pose_target.position.x = P[0][0]
		pose_target.position.y = P[0][1]
		pose_target.position.z = P[0][2]
		self.group.set_pose_target(pose_target)
		plan1 = self.group.plan()
		cnt = 0
		while( (not plan1.joint_trajectory.points) and (cnt<3)):
			print "============ Generating plan 1"
			plan1 = self.group.plan()
			cnt = cnt+1
		time.sleep(5)    
	 	print "============ Executing plan1"
		self.group.execute(plan1)
		print 'Execution Finished.'

if __name__ == '__main__':
    armcontroller=arm_controller_commander.ARMControllerCommander()
    new=VisionMoveIt(armcontroller)
    new.camera_read()
    new.moveit_init()
    new.set_positions()
    #new.generate_plans()
    Robot_Pos = []
    Robot_Joint = []
    new.reset_pos()
    '''
    armcontroller.set_controller(4,1,new.ft_threshold)
    
    plan1=new.generate_plan(1)
    raw_input("Press Enter to continue")
    new.execute_plans(plan1)
    armcontroller.set_controller(4,0.4,new.ft_threshold)
    rospy.sleep(1)
    time.sleep(1) 
     
    plan2=new.generate_plan(2)
    raw_input("Press Enter to continue") 
    new.execute_plans(plan2)
    armcontroller.set_controller(4,0.7,[])
    armcontroller.set_vacuum(1)
    rospy.sleep(1)
    time.sleep(1) 
    
    plan3=new.generate_plan(3)
    raw_input("Press Enter to continue") 
    new.execute_plans(plan3)
   

##########################################


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

    
  
