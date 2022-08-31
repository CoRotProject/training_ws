#!/home/hvh/miniconda3/envs/MRTA/bin/python

import sys, rospy, tf, actionlib, moveit_commander
from control_msgs.msg import (GripperCommandAction, GripperCommandGoal)
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler
from look_at_bin import look_at_bin
from std_srvs.srv import Empty
from moveit_msgs.msg import CollisionObject
from moveit_python import PlanningSceneInterface

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_up_item')
    args = rospy.myargv(argv = sys.argv)
    if len(args) != 2:
        print("usage: pick_up_item.py BIN_NUMBER")
        sys.exit(1)
    item_frame = "item_%d" % int(args[1])

    rospy.wait_for_service("/clear_octomap")
    clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)

    gripper = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
    gripper.wait_for_server()   # Wait for the gripper server to grab the target object

    arm = moveit_commander.MoveGroupCommander("arm")    # use MoveGroupCommander to the Moveit motion planning system
    arm.allow_replanning(True)
    tf_listener = tf.TransformListener()    # subscribe the transformation (static and dynamic) broadcasted by the rest of the system
    rate = rospy.Rate(10)

    gripper_goal = GripperCommandGoal()     # Need a gripper goal object to send to the gripper action server
    gripper_goal.command.max_effort = 10.0

    scene = PlanningSceneInterface("base_link")

    p = Pose()
    p.position.x = 0.4 + 0.15
    p.position.y = -0.4
    p.position.z = 0.7 + 0.15
    p.orientation = Quaternion(*quaternion_from_euler(0, 1, 1))
    arm.set_pose_target(p)      # this pose is out of the way of the depth camera

    while True:
        if arm.go(True):
            break
        clear_octomap()
        scene.clear()

    look_at_bin()
    while not rospy.is_shutdown():
        rate.sleep()
        try:
            t = tf_listener.getLatestCommonTime('/base_link', item_frame)   # use recent data in 200ms
            if (rospy.Time.now() - t).to_sec() > 0.2:
                rospy.sleep(0.1)
                continue
            
            # Extract the resquested transformation from tf library's local presentation of the transform tree
            (item_translation, item_orientation) = tf_listener.lookupTransform('/base_link', item_frame, t)     

        except(tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        gripper_goal.command.position = 0.15
        gripper.send_goal(gripper_goal)     # open the gripper
        gripper.wait_for_result(rospy.Duration(1.0))

        print "item: " + str(item_translation)
        scene.addCube("item", 0.05, item_translation[0], item_translation[1], item_translation[2])

        p.position.x = item_translation[0] - 0.01 - 0.06
        p.position.y = item_translation[1]
        p.position.z = item_translation[2] + 0.04 + 0.14
        p.orientation = Quaternion(*quaternion_from_euler(0, 1.2, 0))
        arm.set_pose_target(p)
        arm.go(True)    # ask Moveit to plan and execute a collision-free path to the item's location

        #os.system("rosservice call clear_octomap")

        gripper_goal.command.position = 0
        gripper.send_goal(gripper_goal)
        gripper.wait_for_result(rospy.Duration(2.0))

        scene.removeAttachedObject("item")

        clear_octomap()

        p.position.x = 0.00
        p.position.y = -0.25
        p.position.z = 0.75 - .1
        p.orientation = Quaternion(*quaternion_from_euler(0, -1.5, -1.5))
        arm.set_pose_target(p)
        arm.go(True)        # lift up the object and bring it back closer to the robot's torso
        break