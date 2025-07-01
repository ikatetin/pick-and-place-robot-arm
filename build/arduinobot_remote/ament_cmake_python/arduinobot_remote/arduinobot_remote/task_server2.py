#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from arduinobot_msgs.action import ArduinobotTask
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
import threading
from threading import Lock
import time

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)
    
class TaskServer(Node):
    def __init__(self):
        super().__init__("task_server")
        self.get_logger().info("Starting the Server La la la")
        
        # MoveIt 2 Interface
        self.arduinobot = MoveItPy(node_name="moveit_py")
        self.arduinobot_arm = self.arduinobot.get_planning_component("arm")
        self.arduinobot_gripper = self.arduinobot.get_planning_component("gripper")
        self.is_busy = False
        self.lock = Lock()
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "base_link"

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
            self.listener_callback,
            10)
        
    def listener_callback(self, data):
        with self.lock:
            if self.is_busy:
                return
            self.is_busy = True

        self.get_logger().info(f"{data}")
        self.get_logger().info("Start plan to: " + str(self.pose_goal))
        self.move_to(data.data[0], data.data[1], data.data[2], 1.0,  0.0, 0.0, 0.0)

        with self.lock:
            self.is_busy = False
    
    def move_to(self, x, y, z, xo, yo, zo, wo):

        self.pose_goal.pose.position.x = x
        self.pose_goal.pose.position.y = y
        self.pose_goal.pose.position.z = z
        self.pose_goal.pose.orientation.x = xo
        self.pose_goal.pose.orientation.y = yo
        self.pose_goal.pose.orientation.z = zo
        self.pose_goal.pose.orientation.w = wo
        self.arduinobot_arm.set_goal_state(self.pose_goal, "claw_support")
        self.get_logger().info("Start plan to: " + str(self.pose_goal))
        plan_and_execute(self.arduinobot, self.arduinobot_arm, self.get_logger(), sleep_time=3.0)



def main(args=None):
    rclpy.init(args=args)
    task_server = TaskServer()
    rclpy.spin(task_server)

if __name__ == "__main__":
    main()