#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander

import actionlib

import std_msgs.msg
import geometry_msgs.msg
import moveit_msgs.msg
import tf_conversions
import math
import franka_control.srv
from math import pi
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, rotation_matrix, translation_from_matrix, quaternion_matrix
import tf
from tf.listener import TransformerROS, Transformer
import numpy as np

import yaml
import tf2_ros

from math import radians
from whole_part import SceneObject

class MoveGroupPlanner():
    def __init__(self):
        ### MoveIt! 
        moveit_commander.roscpp_initialize(sys.argv)
             #rospy.init_node('move_group_planner',
        #                anonymous=True)
 
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group = moveit_commander.MoveGroupCommander("panda_arms")
        self.group_left = moveit_commander.MoveGroupCommander("panda_left")
        # self.object_group = moveit_commander.MoveGroupCommander("stefan")
        self.hand_left = moveit_commander.MoveGroupCommander("hand_left")
        self.hand_right = moveit_commander.MoveGroupCommander("hand_right")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        self.planning_frame = self.group.get_planning_frame()
        print ("============ Reference frame: %s" % self.planning_frame)

        # We can also print the name of the end-effector link for this group:
        self.eef_link = self.group.get_end_effector_link()
        print ("============ End effector: %s" % self.eef_link)

        # We can get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()
        print ("============ Robot Groups:", self.robot.get_group_names())

        rospy.sleep(1)
        self.stefan_dir = "/home/jiyeong/STEFAN/stl/"
        self.stefan = SceneObject()

        self.scene.remove_attached_object(self.group.get_end_effector_link())
        rospy.sleep(1)
        self.scene.remove_world_object()
        for key, value in self.stefan.list.items():
            self.scene.add_mesh(key, value, self.stefan_dir + key + ".stl")
        rospy.sleep(1)
        ### Franka Collision
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        #self.set_collision_behavior.wait_for_service()

        self.active_controllers = []


    # geometry_msgs.msg.Pose() or self.group.get_current_joint_values()
    def plan(self, goal, arm_name):
        if (arm_name == 'panda_arms'):
            self.group.set_max_velocity_scaling_factor = 0.6
            self.group.set_max_acceleration_scaling_factor = 0.4
            self.group.set_start_state_to_current_state()
            trajectory = self.group.plan(goal)
        return trajectory
    
    # def plan_cartesian_target(self):
    #     pose_goal = geometry_msgs.msg.Pose()
    #     pose_goal.orientation = geometry_msgs.msg.Quaternion(
    #         *tf_conversions.transformations.quaternion_from_euler(math.radians(90), math.radians(90), math.radians(0)))
    #     pose_goal.position.x =  0.5429
    #     pose_goal.position.y = 0.05
    #     pose_goal.position.z = 0.6 + 0.30

    #     trajectory = self.group_left.plan(pose_goal)
    #     return trajectory

    def initial_pose(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[6] = pi/4
        joint_goal[13] = pi/4
        self.group.plan(joint_goal)
        self.group.go()
        
    def plan_joint_target(self):
        joint_goal = self.group.get_current_joint_values()
        joint_goal = [-1.86503 ,0.422209, 1.68431 ,-2.56123, -0.219032 ,2.29539, 1.83065, 2.11514, -1.07206 ,-1.29386, -1.69659 ,-0.523207 ,1.54499 ,2.63821 ]
        self.group.plan(joint_goal)
        self.group.go()
    # def plan_object(self):
    #     pose_goal = geometry_msgs.msg.Pose()
    #     q = tf.transformations.quaternion_from_euler(radians(85), radians(3), 0)
    #     pose_goal.orientation.x = q[0]
    #     pose_goal.orientation.y = q[1]
    #     pose_goal.orientation.z = q[2]
    #     pose_goal.orientation.w = q[3]

    #     pose_goal.position.x = 1.15
    #     pose_goal.position.y = 0.15
    #     pose_goal.position.z = 0.65

    #     success = self.object_group.plan(pose_goal)

    #     self.object_group.go()

    def gripper_open(self, arm):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.04
        joint_goal[1] = 0.04
        if (arm == "left"):
            self.hand_left.plan(joint_goal)
            print("============ OPEN LEFT GRIPPER")
            self.hand_left.go()
        else :
            self.hand_right.plan(joint_goal)
            print("============ OPEN RIGHT GRIPPER")
            self.hand_right.go()
        

    def gripper_close(self):
        joint_goal = self.hand_left.get_current_joint_values()
        joint_goal[0] = 0.0
        joint_goal[1] = 0.0
        trajectory = self.hand_left.plan(joint_goal)
        return trajectory

    def display_trajectory(self, plan):
        self.display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        self.display_trajectory.trajectory_start = self.robot.get_current_state()
        self.display_trajectory.trajectory.append(plan)
        # Publish
        self.display_trajectory_publisher.publish(display_trajectory)

if __name__ == '__main__':

    sys.argv.append('joint_states:=/panda_dual/joint_states')
    rospy.init_node('ggg')
    
    mdp = MoveGroupPlanner()
    # mdp.initial_pose()
    
    # mdp.gripper_open("right")
    # mdp.gripper_open("left")

    # mdp.plan_joint_target()
    # for key, value in mdp.stefan.list.items():
    #         mdp.scene.add_mesh(key, value, mdp.stefan_dir + key + ".stl")
        
    # rospy.sleep(1)
    # mdp.scene.attach_mesh(mdp.group_left.get_end_effector_link(), "assembly")
    # rospy.sleep(1)
    

    
    

