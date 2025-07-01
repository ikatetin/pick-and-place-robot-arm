#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from arduinobot_msgs.action import ArduinobotTask




class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Server")
        self.action_server = ActionServer(
            self, ArduinobotTask, "task_server", self.goalCallback
        )

        # MoveIt 2 Interface
        self.arduinobot = MoveItPy(node_name="moveit_py")
        self.arduinobot_arm = self.arduinobot.get_planning_component("arm")
        self.arduinobot_gripper = self.arduinobot.get_planning_component("gripper")

    def goalCallback(self, goal_handle):
        self.get_logger().info(
            "Received goal request with id %d" % goal_handle.request.task_number
        )

        arm_state = RobotState(self.arduinobot.get_robot_model())
        gripper_state = RobotState(self.arduinobot.get_robot_model())

        arm_joint_goal = []
        gripper_joint_goal = []

        if goal_handle.request.task_number == 0:
            arm_joint_goal = np.array(goal_handle.request.arm_goal)
            # gripper_joint_goal = np.array([-1.7, 1.7])
        elif goal_handle.request.task_number == 1:
            # arm_joint_goal = np.array(goal_handle.request.arm_goal)
            gripper_joint_goal = np.array(goal_handle.request.gripper_goal)
        elif goal_handle.request.task_number == 2:
            arm_joint_goal = np.array(goal_handle.request.arm_goal)
            gripper_joint_goal = np.array(goal_handle.request.gripper_goal)

        else:
            self.get_logger().error("Invalid Task Number")
            return
        
        arm_plan_result = None 
        gripper_plan_result = None 
        if goal_handle.request.task_number != 1:
            arm_state.set_joint_group_positions("arm", arm_joint_goal)
            self.arduinobot_arm.set_start_state_to_current_state()
            self.arduinobot_arm.set_goal_state(robot_state=arm_state)
            arm_plan_result = self.arduinobot_arm.plan()

        if goal_handle.request.task_number != 0:
            gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)      
            self.arduinobot_gripper.set_start_state_to_current_state()
            self.arduinobot_gripper.set_goal_state(robot_state=gripper_state)
            gripper_plan_result = self.arduinobot_gripper.plan()

        

        if arm_plan_result is not None: 
            self.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
        if gripper_plan_result is not None: 
            self.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
        
        
        self.get_logger().info("Goal succeeded")
     
        goal_handle.succeed()
        result = ArduinobotTask.Result()
        result.success = True
        return result 


def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)

if __name__ == "__main__":
    main()