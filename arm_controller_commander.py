import rospy
import moveit_commander
import moveit_msgs.msg

from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest, SetControllerModeResponse
    
from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    RapidStart, RapidStartRequest, RapidStartResponse, \
    RapidStop, RapidStopRequest, RapidStopResponse, \
    RapidGetStatus, RapidGetStatusRequest, RapidGetStatusResponse, \
    RapidGetDigitalIO, RapidGetDigitalIORequest, RapidGetDigitalIOResponse, \
    RapidSetDigitalIO, RapidSetDigitalIORequest, RapidSetDigitalIOResponse, \
    RapidReadEventLog, RapidReadEventLogRequest, RapidReadEventLogResponse


class ARMControllerCommander:
    def __init__(self):
	
	set_controller_mode=rospy.ServiceProxy('set_controller_mode', SetControllerMode)
        set_digital_io=rospy.ServiceProxy('rapid/set_digital_io', RapidSetDigitalIO)
    def move_it_init(self):
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('collision_checker','move_group_python_interface_tutorial',
                  anonymous=True)

  ## MoveIt! Initialization
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("move_group")
        group.set_goal_position_tolerance(0.04)
        group.allow_replanning(True)
        group.set_planner_id("RRTConnectkConfigDefault") #RRTConnectkConfigDefault/SBLkConfigDefault/KPIECEkConfigDefault/BKPIECEkConfigDefault/LBKPIECEkConfigDefault/
        group.set_num_planning_attempts(5)
        display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
	return group
    
    def set_controller(self,mode,speed_scalar,ft_threshold):
	req=SetControllerModeRequest()
    	req.mode.mode=mode
    	req.speed_scalar=speed_scalar
    	req.force_torque_stop_threshold=ft_threshold
	res=set_controller_mode(req)
	if (not res.success): raise Exception("Could not set controller mode")

    def set_vacuum(self,status):
	#send 0 or 1 to change vacuum
	req=RapidSetDigitalIORequest()
        req.signal="Vacuum_enable"
        req.lvalue=status
        set_digital_io(req)



