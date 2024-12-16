#!/usr/bin/env python

import sys
import rospy as ros
import numpy as np
import math

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
from relaxed_ik_ros1.srv import IKPoseRequest, IKPose, IKPoseResponse

from transformations import quaternion_slerp
from uitils import time_consumption


class Params:
    """
    Load ROS parameters for global planner, local planner, inverse kinematics, and debugger
    """
    gp_config = ros.get_param('~global_planner', None)
    lp_config = ros.get_param('~local_planner', None)
    ik_config = ros.get_param('~inverse_kinematics', None)
    debugger = ros.get_param('~debugger', False)
    
    lp_size = lp_config['step_size'] * lp_config['steps']


class GlobalPlanner:
    def __init__(self):
        
        self.gp_config = Params.gp_config
        self.ik_config = Params.ik_config
        self.lp_size = Params.lp_size

        # # Subscribe to IK service/topics
        # ros.wait_for_service(self.ik_config['service'])
        # self.ik_pose_service = ros.ServiceProxy(self.ik_config['service'], IKPose)


    @time_consumption(enabled=Params.debugger['evaluate_time'])
    def plan(self, current_state: Pose, target_state: Pose) -> Pose:
        """
        Global planner to plan the path from current state to target state
        return local_ee_target
        """
        distance = np.linalg.norm([target_state.position.x - current_state.position.x,
                                        target_state.position.y - current_state.position.y,
                                        target_state.position.z - current_state.position.z])
        local_ee_target = Pose()
        
        if distance <= self.lp_size:
            ros.loginfo("Approaching target in Euclidean space. Euclidean distance: {distance}")
            local_ee_target = target_state

        else:
            ros.loginfo(f"Getting global linear samples. Euclidean distance: {distance}")
            local_ee_target.position.x = current_state.position.x + self.localplanner_size * (target_state.position.x - current_state.position.x) / distance
            local_ee_target.position.y = current_state.position.y + self.localplanner_size * (target_state.position.y - current_state.position.y) / distance
            local_ee_target.position.z = current_state.position.z + self.localplanner_size * (target_state.position.z - current_state.position.z) / distance
            
        return local_ee_target


class LocalPlanner:
    def __init__(self):
        
        self.lp_config = Params.lp_config
        self.ik_config = Params.ik_config
        self.lp_size = Params.lp_size
    
    
    @time_consumption(enabled=Params.debugger['evaluate_time'])
    def plan(self, current_state: Pose, target_state: Pose) -> list[Pose]:
        """
        Local planner to plan the path from current state to target state with SLERP Interpolation
         @ return ee_pose_plan
        """
        distance = np.linalg.norm([target_state.position.x - current_state.position.x,
                                   target_state.position.y - current_state.position.y,
                                   target_state.position.z - current_state.position.z])
        
        distance_to_plan = min(distance.item(), self.lp_size)
        steps_to_plan = math.ceil(distance_to_plan / self.lp_config['step_size'])
        assert steps_to_plan <= self.lp_config['steps'], "Steps to plan exceed the maximum steps"

        ee_pose_plan = list()
        for i in range(1, self.steps_to_plan):
            ee_pose = Pose()
            ee_pose.position.x = current_state.position.x + i / self.steps_to_plan * (target_state.position.x - current_state.position.x)
            ee_pose.position.y = current_state.position.y + i / self.steps_to_plan * (target_state.position.y - current_state.position.y)
            ee_pose.position.z = current_state.position.z + i / self.steps_to_plan * (target_state.position.z - current_state.position.z)
            # quaternion_slerp uses [w, x, y, z] as quaternion, O(1) complexity
            q_f = quaternion_slerp(current_state.orientation, target_state.orientation, i / self.steps_to_plan)
            ee_pose.orientation.x = q_f[1]
            ee_pose.orientation.y = q_f[2]
            ee_pose.orientation.z = q_f[3]
            ee_pose.orientation.w = q_f[0]
            # No tolerance
            ee_pose_plan.append(ee_pose)
        
        ee_pose_plan.append(target_state)   # Add the target state as the last waypoint
        assert len(ee_pose_plan) == steps_to_plan, f"Actual waypoints: {ee_pose_plan}, Expected steps: {steps_to_plan}"
        
        return ee_pose_plan

      
    # Add this to the ROS node
    def use_ik_service(self, ee_pose: Pose) -> list[list[float]]:
        """
        It is not recommended to use the ROS functionality in scripts outside the ROS node.
        """
        pass
        # req = IKPoseRequest()
        # req.ee_pose = ee_pose
        # res: IKPoseResponse = self.ik_pose_service(req)
        # ik_joint_solutions = res.joint_state
        # ik_joint_seq.append(ik_joint_solutions)
    
        # return ik_joint_seq