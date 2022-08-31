#!/home/hvh/miniconda3/envs/MRTA/bin/python

import sys, rospy, copy, moveit_commander, moveit_msgs.msg, geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander('arm')
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)


group_variable_values = group.get_current_joint_values()

group_variable_values[4] = -0.3
group_variable_values[5] = 1
group.set_joint_value_target(group_variable_values)

plan2 = group.plan()
group.go(wait=True)

moveit_commander.roscpp_shutdown()