#!/usr/bin/env python

import sys
import rospy as ros
import numpy as np

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
from relaxed_ik_ros1.srv import IKPoseRequest, IKPose, IKPoseResponse

from transformations import quaternion_slerp
from uitils import ModuleTimer

class Planner:
    def __init__(self):

        self.ee_target: Pose
        self.ee_current: Pose

        self.step_size: float = 0.01
        self.localplanner_steps: int = 10
        self.localplanner_size: float = self.step_size * float(self.localplanner_steps)

        self.use_topic_not_service: bool = False

        # Subscribe to IK service/topics
        ros.wait_for_service('relaxed_ik/solve_pose')
        self.ik_pose_service = ros.ServiceProxy('relaxed_ik/solve_pose', IKPose)

        self.debug: bool = False
        if self.debug:
            self.timer = ModuleTimer("Planner")


    def global_planner(self):
        # Time Consumption Analysis
        if self.debug:
            self.timer.start()
        
        self.distance = np.linalg.norm([self.ee_target.position.x - self.ee_current.position.x,
                                        self.ee_target.position.y - self.ee_current.position.y,
                                        self.ee_target.position.z - self.ee_current.position.z])
        if self.distance <= self.localplanner_size:
            ros.loginfo("Approaching target. Ending planning...")
            local_ee_target = self.ee_target
            self.local_planner(local_ee_target, end=True)
        else:
            ros.loginfo(f"Global planning. Distance: {self.distance}")
            local_ee_target = Pose()
            local_ee_target.position.x = self.ee_current.position.x + self.localplanner_size * (self.ee_target.position.x - self.ee_current.position.x) / distance
            local_ee_target.position.y = self.ee_current.position.y + self.localplanner_size * (self.ee_target.position.y - self.ee_current.position.y) / distance
            local_ee_target.position.z = self.ee_current.position.z + self.localplanner_size * (self.ee_target.position.z - self.ee_current.position.z) / distance
            self.local_planner(local_ee_target, end=False)

        # Time Consumption Analysis
        if self.debug:
            self.timer.end()
            self.timer.get_time_consumption()

    def local_planner(self, local_ee_target: Pose, end: bool = False) -> list[list[float] | None]:
        

        ik_joint_seq = list()
        fraction = 1 / self.localplanner_steps
        max_fraction = self.distance / self.localplanner_size

        for i in range(1, self.localplanner_steps):
            req = IKPoseRequest()
                
            if i * fraction > max_fraction:
                req.ee_pose = local_ee_target
                break
            else:
                ee_pose = Pose()
                ee_pose.position.x = self.ee_current.position.x + i * fraction * (self.ee_target.position.x - self.ee_current.position.x)
                ee_pose.position.y = self.ee_current.position.y + i * fraction * (self.ee_target.position.y - self.ee_current.position.y)
                ee_pose.position.z = self.ee_current.position.z + i * fraction * (self.ee_target.position.z - self.ee_current.position.z)
                # quaternion_slerp uses [w, x, y, z] as quaternion, O(1) complexity
                if end:
                    q_f = quaternion_slerp(self.ee_current.orientation, self.ee_target.orientation, i * fraction)
                    ee_pose.orientation.x = q_f[1]
                    ee_pose.orientation.y = q_f[2]
                    ee_pose.orientation.z = q_f[3]
                    ee_pose.orientation.w = q_f[0]
                # No tolerance
                req.ee_pose = ee_pose

            res: IKPoseResponse = self.ik_pose_service(req)
            ik_joint_solutions = res.joint_state
            ik_joint_seq.append(ik_joint_solutions)
        
        return ik_joint_seq


    def update(self, ee_target: Pose, ee_current: Pose):
        """
        Update the target pose and current pose
        """
        self.ee_target = ee_target
        self.ee_current = ee_current

        self.global_planner()