#!/usr/bin/env python

import sys
import rospy as ros

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

NODE_NAME = 'any_joint_pose'

class any_joint_pose_node:
    def __init__(self):

        # In launchfile these are remapped to namespace 
        # effort_joint_trajectory_controller/ and franka_state_controller/
        self.action: str = ros.resolve_name('~follow_joint_trajectory')
        self.topic = ros.resolve_name('~joint_states')

        # action client and pseudo action server
        self.client = SimpleActionClient(self.action, FollowJointTrajectoryAction)
        ros.loginfo(f"{NODE_NAME}: Waiting for {self.action} action to come up")
        self.client.wait_for_server()
        
        # read current joint state for once
        # TODO: create a subscriber for joint states
        ros.loginfo(f"{NODE_NAME}: Waiting for message on topic {self.topic}")
        joint_state = ros.wait_for_message(self.topic, JointState)
        self.initial_pose = dict(zip(joint_state.name, joint_state.position))
        self.current_pose = self.initial_pose   # TODO: deep copy


    def readOnce(self):
        """
        Read joint pose from ROS parameter for once
        """
        param: str = ros.resolve_name('~joint_pose')
        self.joint_pose = ros.get_param(param, None)    # target joint pose
        if self.joint_pose is None:
            ros.logerr(f'{NODE_NAME}: Could not find required parameter {param}')
            sys.exit(1)


    def move_to_pose(self):
        self.readOnce()     # update self.joint_pose
        max_movement = max(abs(self.joint_pose[joint] - self.current_pose[joint]) for joint in self.joint_pose)
        point = JointTrajectoryPoint()
        point.time_from_start = ros.Duration.from_sec(
            # Use either the time to move the furthest joint with 'max_dq' or 500ms,
            # whatever is greater
            max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
        )
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*self.joint_pose.items())]    # joint names and target joint pose
        
        point.velocities = [0] * len(self.joint_pose)   # stop at target pose (velocity = 0)
        goal.trajectory.points.append(point)    # only one point in trajectory
        goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

        ros.loginfo(f'{NODE_NAME}: Sending trajectory Goal to move into specified config')
        self.client.send_goal_and_wait(goal)
        result = self.client.get_result()
        self.check_result(result)


    def check_result(self, result: FollowJointTrajectoryResult):
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            ros.logerr('move_to_start: Movement was not successful: ' + {
                FollowJointTrajectoryResult.INVALID_GOAL:
                """
                The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
                Is the 'joint_pose' reachable?
                """,

                FollowJointTrajectoryResult.INVALID_JOINTS:
                """
                The joint pose you specified is for different joints than the joint trajectory controller
                is claiming. Does you 'joint_pose' include all 7 joints of the robot?
                """,

                FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
                """
                During the motion the robot deviated from the planned path too much. Is something blocking
                the robot?
                """,

                FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
                """
                After the motion the robot deviated from the desired goal pose too much. Probably the robot
                didn't reach the joint_pose properly
                """,
            }[result.error_code])

        else:
            ros.loginfo(f'{NODE_NAME}: Successfully moved into specified pose')


def main():
    ros.init_node(NODE_NAME)
    node = any_joint_pose_node()
    node.move_to_pose()

    # TODO: finally spin the node


if __name__ == '__main__':
    main()