#!/usr/bin/env python

import sys
import rospy as ros
import tf2_ros

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from geometry_msgs.msg import TransformStamped, Pose
from franka_gripper.msg import MoveAction, MoveGoal, MoveResult
from franka_msgs.msg import FrankaState

from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
from relaxed_ik_ros1.srv import IKPoseRequest,  IKPose, IKPoseResponse

from planner import GlobalPlanner, LocalPlanner
from uitils import time_consumption
from robot import Robot


class Params:
    """
    Load ROS parameters
    """
    rate = ros.get_param('~rate', None)
    node_name = ros.get_param('~node_name', "move_to_pos")
    target_cart_pose = ros.get_param('~target_cart_pose', None)
    ik_config = ros.get_param('~inverse_kinematics', None)
    debugger = ros.get_param('~debugger', False)


class MoveToPose:
    def __init__(self):

        # read parameters
        self.rate = ros.Rate(Params.rate)

        # In launchfile these topics will be remapped to namespace for the effort controller
        # effort_joint_trajectory_controller/ and franka_state_controller/
        self.action: str = ros.resolve_name('~follow_joint_trajectory')
        self.topic = ros.resolve_name('~joint_states')
        # action client and pseudo action server
        self.client = SimpleActionClient(self.action, FollowJointTrajectoryAction)
        ros.loginfo(f"{Params.node_name}: Waiting for {self.action} action to come up")
        self.client.wait_for_server()
        
        # Subscribe to franka_state_controller
        self.franka_state_sub = ros.Subscriber("franka_state_controller/franka_states", FrankaState, self.franka_state_callback)
        self.franka_joint_state_sub = ros.Subscriber("franka_state_controller/joint_states", JointState, self.franka_joint_state_callback)
        
        # transformation listener
        tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(tf_buffer)
        # read tf tree
        try:
            init_ee_trans = tf_buffer.lookup_transform("world", "panda_hand", ros.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            ros.logerror(f"Failed to lookup transform: {e}")

        # read joint state
        joint_state = ros.wait_for_message(self.topic, JointState)
        init_joint_state = dict(zip(joint_state.name, joint_state.position))
        self.current_joint_state = init_joint_state

        # create robot object
        setting_file_path = ros.get_param('~setting_file_path', None)
        self.robot = Robot(setting_file_path)
        init_fk_ee_pos = self.robot.fk(init_joint_state['panda_hand'])
        
        ros.loginfo(f"Initial EE pose from /tf: {init_ee_trans.transform.translation} \nInitial EE pose from IK: {init_fk_ee_pos}")

        # connect to inverse kinematics service
        ros.wait_for_service(Params.ik_config['service'])
        self.ik_pose_service = ros.ServiceProxy(Params.ik_config['service'], IKPose)
        self.ik_pose_sub = ros.Publisher(Params.ik_config['relaxed_ik/ee_pose_goals'], EEPoseGoals, self.ik_pose_callback)
        
        # connect to effort controller
        
        
        

        # # read joint pose and gripper width from ROS parameter
        # param: str = ros.resolve_name('~load_gripper')
        # self.load_gripper: bool = ros.get_param(param, False)
        # # if self.load_gripper:
        # #     # create a client for the gripper action
        # #     self.gripper_move: str = ros.resolve_name('~move/goal')
        # #     self.gripper_client = SimpleActionClient(self.gripper_move, MoveAction)
        # #     self.gripper_client.wait_for_server()
    
    def franka_state_callback(self, msg: FrankaState):
        
    def franka_joint_state_callback(self, msg: JointState):
    
    def plan(self):
        gp = GlobalPlanner()
        lp = LocalPlanner()
        # get current state
        current_state = self.current_joint_state['panda_hand']
        # get target state
        target_state = self.target_cart_pose
        # plan global path
        local_ee_target = gp.plan(current_state, target_state)
        # plan local path
        joint_path = lp.plan(current_state, local_ee_target)
        return joint_path
        

    def move_to_pose(self, pose: Pose):
        """
        Move the end effector to the specified pose
        """
        gp = GlobalPlanner()
        lp = LocalPlanner()
        
        
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

        ros.loginfo(f'{Params.node_name}: Sending trajectory Goal to move into specified config')
        self.client.send_goal_and_wait(goal)
        result = self.client.get_result()
        self.check_result(result)




    def check_result(self, result: FollowJointTrajectoryResult):
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            ros.logerr(f'{Params.node_name}: Movement was not successful: ' + {
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
            ros.loginfo(f'{Params.node_name}: Successfully moved arm into specified pose')

    # def readOnce(self):
    #     """
    #     Read joint pose and gripper pose from ROS parameter for once
    #     """
    #     param: str = ros.resolve_name('~joint_pose')
    #     self.joint_pose = ros.get_param(param, None)    # target joint pose
    #     if self.joint_pose is None:
    #         ros.logerr(f'{Params.node_name}: Could not find required parameter {param}')
    #         sys.exit(1)
        
    #     if self.load_gripper:
    #         param: str = ros.resolve_name('~gripper_config')
    #         self.gripper_config = ros.get_param(param, None)    # target gripper width
    #         if self.gripper_config is None:
    #             ros.logerr(f'{Params.node_name}: Could not find required parameter {param}')
    #             sys.exit(1)

    
    # def move_gripper(self):
    #     if not self.load_gripper:
    #         ros.logerr(f'{Params.node_name}: Gripper is not loaded. Cannot move gripper.')
    #         sys.exit(1)
        
    #     goal = MoveGoal()
    #     goal.width = self.gripper_config.width
    #     goal.speed = self.gripper_config.speed

    #     ros.loginfo(f'{Params.node_name}: Sending gripper Goal to move into specified config')
    #     self.gripper_client.send_goal_and_wait(goal)
    #     result = self.gripper_client.get_result()
    #     self.check_gripper_result(result)

    # def check_gripper_result(self, result: MoveResult):
    #     if not result.success:
    #         ros.logerr(f'{Params.node_name}: Gripper movement was not successful\n {result.error}')
    #     else:
    #         ros.loginfo(f'{Params.node_name}: Successfully moved gripper into specified pose')




def main():
    ros.init_node(Params.node_name)
    node = MoveToPose()
    node.move_to_pose()
    # node.move_gripper()



if __name__ == '__main__':
    main()