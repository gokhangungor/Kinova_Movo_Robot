#!/usr/bin/env python
import rospy
import copy
import tf
import actionlib
import sys
import math
import csv

import moveit_commander

from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("trajectory_demo")

right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
right_arm_group.set_planner_id("RRTConnectkConfigDefault")
print("MoveIt Initial Loading Done")

# Maybe not useful
right_traj_pub = rospy.Publisher("/movo/right_arm_controller/command", JointTrajectory, queue_size=3)


right_action_client = actionlib.SimpleActionClient("/movo/right_arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
right_action_client.wait_for_server()

right_arm_joints = ["right_shoulder_pan_joint",
                                    "right_shoulder_lift_joint",
                                    "right_arm_half_joint",
                                    "right_elbow_joint",
                                    "right_wrist_spherical_1_joint",
                                    "right_wrist_spherical_2_joint",
                                    "right_wrist_3_joint"]


right_joints_tucked = [-1.6,-1.5,0.4,-2.7,0.0,0.5, -1.7]


def execute_plan_async(traj):
    print(type(traj))
    print("Executing trajectory of %s" % (len(traj.points)))
    if len(traj.points) > 1:
        goal = create_trajectory_action_goal(traj)
        print "bar"

        right_action_client.send_goal(goal)
        print "foo"
        return max([traj.points[-1].time_from_start.secs, 3])

    print("Invalid")
    return 3

def create_trajectory_action_goal(joint_traj):
    goal = FollowJointTrajectoryActionGoal()
    goal.goal.trajectory = joint_traj
    return goal.goal


csv_file = open("q_test.csv", "r")
f = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
header = next(f)
joint1 = [header[0]]
joint2 = [header[1]]
joint3 = [header[2]]
joint4 = [header[3]]
joint5 = [header[4]]
joint6 = [header[5]]
joint7 = [header[6]]
for row in f:
    joint1.append(row[0])
    joint2.append(row[1])
    joint3.append(row[2])
    joint4.append(row[3])
    joint5.append(row[4])
    joint6.append(row[5])
    joint7.append(row[6])


jt = JointTrajectory()
jt.joint_names = right_arm_joints

for i in range(0, len(joint1)-1, 200):
    t= i/2
    jtp = JointTrajectoryPoint()
    jtp.time_from_start = rospy.Duration(t)

    q1 = float(joint1[i])
    q2 = float(joint2[i])
    q3 = float(joint3[i])
    q4 = float(joint4[i])
    q5 = float(joint5[i])
    q6 = float(joint6[i])
    q7 = float(joint7[i])

    jtp.positions = [q1, q2, q3, q4, q5, q6, q7]
    #jtp.positions = [q, q, q, 0, 0, 0, 0]
    jt.points.append(jtp)

print(jt)
execute_plan_async(jt)
import time
time.sleep(3)