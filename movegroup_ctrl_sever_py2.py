#!/usr/bin/env python

# from __future__ import print_function

import sys
import rospy
from py2_client.srv import *
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True



class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
      super(MoveGroupPythonIntefaceTutorial, self).__init__()


      ## First initialize `moveit_commander`_ and a `rospy`_ node:
      moveit_commander.roscpp_initialize(sys.argv)
      rospy.init_node('movegroup_ctrl_server_py2', anonymous=True)

      ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
      ## kinematic model and the robot's current joint states
      robot = moveit_commander.RobotCommander()

      ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
      ## to a planning group (group of joints) which can be used to plan and execute motions. 
      #  In this tutorial the group is the primary arm joints in the Panda robot
      group_name = "panda_arm"
      move_group = moveit_commander.MoveGroupCommander(group_name)

      ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
      ## trajectories in Rviz:
      display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                  moveit_msgs.msg.DisplayTrajectory,
                                                  queue_size=20)


      # We can get a list of all the groups in the robot:
      group_names = robot.get_group_names()
      print "============ Available Planning Groups:", robot.get_group_names()

      # Sometimes for debugging it is useful to print the entire state of the
      # robot:
      print "============ Printing robot state"
      print robot.get_current_state()
      print ""

      # Misc variables
      self.box_name = ''
      self.robot = robot
      self.move_group = move_group
      self.display_trajectory_publisher = display_trajectory_publisher
      self.group_names = group_names
      # Pose Goal var
      self.x = 0
      self.y = 0
      self.z = 0
      self.w = 0

# bourne
  def server_loop(self):
    s = rospy.Service('tactile_motion_ctrl', CtrlSrv, self.handle_next_command)
    print("Ready to Recieve Pose Goal" )
    rospy.spin()

# bourne
  def handle_next_command(self, req):
    print("Recieved pose goal: x = [%s], y = [%s], z = [%s], w = [%s]" %(req.x, req.y, req.z, req.w))

    move_group = self.move_group

    ## Planning to a Pose Goal
    pose_goal = geometry_msgs.msg.Pose()
    # 
    
    pose_goal.orientation.w = req.w
    pose_goal.position.x = req.x
    pose_goal.position.y = req.y
    pose_goal.position.z = req.z

    self.move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # Note: It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    # current_pose = self.move_group.get_current_pose().pose
    # print("Motion Finished")
    # cannot put server_loop here, otherwise it would be a deadly loop
    # self. server_loop()
    return CtrlSrvResponse("Motion Completed.")




def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Ready to execute a movement using a pose goal."
    tutorial.server_loop()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()