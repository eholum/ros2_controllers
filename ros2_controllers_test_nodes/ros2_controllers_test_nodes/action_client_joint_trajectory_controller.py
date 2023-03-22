# Copyright 2023 PickNik Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Lovro Ivanov
#

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

from control_msgs.action import FollowJointTrajectory
from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_GetResult_Response
from rclpy.action import ActionClient
from threading import Thread
from rclpy.node import Rate


class ActionClientJointTrajectory(Node):
    def __init__(self):
        super().__init__("action_client_position_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "joint_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", 6)
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("joints", [""])
        self.declare_parameter("check_starting_point", False)

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "/joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []  # List of JointTrajectoryPoint
        for name in goal_names:
            self.declare_parameter(name, descriptor=ParameterDescriptor(dynamic_typing=True))
            goal = self.get_parameter(name).value

            # TODO(anyone): remove this "if" part in ROS Iron
            if isinstance(goal, list):
                self.get_logger().warn(
                    f'Goal "{name}" is defined as a list. This is deprecated. '
                    "Use the following structure:\n<goal_name>:\n  "
                    "positions: [joint1, joint2, joint3, ...]\n  "
                    "velocities: [v_joint1, v_joint2, ...]\n  "
                    "accelerations: [a_joint1, a_joint2, ...]\n  "
                    "effort: [eff_joint1, eff_joint2, ...]"
                )

                if goal is None or len(goal) == 0:
                    raise Exception(f'Values for goal "{name}" not set!')

                float_goal = [float(value) for value in goal]

                point = JointTrajectoryPoint()
                point.positions = float_goal
                point.time_from_start = Duration(sec=4)

                self.goals.append(point)

            else:
                point = JointTrajectoryPoint()

                def get_sub_param(sub_param):
                    param_name = name + "." + sub_param
                    self.declare_parameter(param_name, [float()])
                    param_value = self.get_parameter(param_name).value

                    float_values = []

                    if len(param_value) != len(self.joints):
                        return [False, float_values]

                    float_values = [float(value) for value in param_value]

                    return [True, float_values]

                one_ok = False

                [ok, values] = get_sub_param("positions")
                if ok:
                    point.positions = values
                    one_ok = True

                [ok, values] = get_sub_param("velocities")
                if ok:
                    point.velocities = values
                    one_ok = True

                [ok, values] = get_sub_param("accelerations")
                if ok:
                    point.accelerations = values
                    one_ok = True

                [ok, values] = get_sub_param("effort")
                if ok:
                    point.effort = values
                    one_ok = True

                if one_ok:
                    point.time_from_start = Duration(sec=4)
                    self.goals.append(point)
                    self.get_logger().info(f'Goal "{name}" has definition {point}')

                else:
                    self.get_logger().warn(
                        f'Goal "{name}" definition is wrong. This goal will not be used. '
                        "Use the following structure: \n<goal_name>:\n  "
                        "positions: [joint1, joint2, joint3, ...]\n  "
                        "velocities: [v_joint1, v_joint2, ...]\n  "
                        "accelerations: [a_joint1, a_joint2, ...]\n  "
                        "effort: [eff_joint1, eff_joint2, ...]"
                    )

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        action_name = "/" + controller_name + "/" + "follow_joint_trajectory"

        self.get_logger().info(
            f'Sending {len(goal_names)} goals on action server "{action_name}" until stopped '
        )

        self.action_client_ = ActionClient(self, FollowJointTrajectory, action_name)

        while not self.action_client_.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('server not available, waiting again...')

        self.i = 0

        # start main thread

        self.spin_thread = Thread(target=rclpy.spin, args=(self, ), name="spin_thread")
        self.spin_thread.start()

        self.run_thread = Thread(target=self.run(), name="run_thread")
        self.run_thread.start()

    def finalize(self):
        # join thread
        self.spin_thread.join()
        self.run_thread.join()

    def run(self):
        try:
            while rclpy.ok():

                if self.starting_point_ok:

                    self.get_logger().info(f"Sending goal {self.goals[self.i]}.")

                    result_response = self.send_goal()
                    result = result_response.result
                    status = result_response.status

                    self.get_logger().info(
                        f"Status: {status}"
                    )

                    if result.error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
                        self.get_logger().info(
                            f"Result: GOAL_TOLERANCE_VIOLATED {result.error_string}"
                        )
                    elif result.error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
                        self.get_logger().info(
                            f"Result: INVALID_JOINTS {result.error_string}"
                        )
                    elif result.error_code == FollowJointTrajectory.Result.INVALID_GOAL:
                        self.get_logger().info(
                            f"Result: INVALID_GOAL {result.error_string}"
                        )
                    elif result.error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
                        self.get_logger().info(
                            f"Result: OLD_HEADER_TIMESTAMP {result.error_string}"
                        )
                    elif result.error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
                        self.get_logger().info(
                            f"Result: PATH_TOLERANCE_VIOLATED {result.error_string}"
                        )
                    elif result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                        self.get_logger().info(
                            f"Result: SUCCESSFUL {result.error_string}"
                        )

                    self.i += 1
                    self.i %= len(self.goals)

                elif self.check_starting_point and not self.joint_state_msg_received:
                    self.get_logger().warn(
                        'Start configuration could not be checked! Check "joint_state" topic!'
                    )
                else:
                    self.get_logger().warn("Start configuration is not within configured limits!")
        except KeyboardInterrupt:
            pass

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.joints
        traj.points.append(self.goals[self.i])
        goal_msg.trajectory = traj

        self.action_client_.wait_for_server()

        return self.action_client_.send_goal(goal_msg)

    def joint_state_callback(self, msg):

        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    action_client_joint_trajectory = ActionClientJointTrajectory()

    # rclpy.spin(action_client_joint_trajectory)

    action_client_joint_trajectory.finalize()

    action_client_joint_trajectory.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
