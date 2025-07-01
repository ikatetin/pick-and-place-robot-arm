#!/usr/bin/env python3

import time
import threading
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.logging import get_logger
from moveit.planning import MoveItPy
from std_msgs.msg import Float64MultiArray
# moveit python library
from moveit.core.robot_state import RobotState
from threading import Lock

def plan_and_execute(
    robot,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    arm_plan_result = robot.arduinobot_arm.plan()
    gripper_plan_result = robot.arduinobot_gripper.plan()
    # plan to goal
    if arm_plan_result and gripper_plan_result:
        logger.info("Planner SUCCEED, moving the arm and the gripper")
        robot.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
        robot.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
    else:
        logger.info("One or more planners failed!")
    
    logger.info("Plan succeeded")
    time.sleep(sleep_time)

class Controller(Node):

    def __init__(self):
        super().__init__('commander')
    
        # MoveIt 2 Interface
        self.arduinobot = MoveItPy(node_name="moveit_py")
        self.arduinobot_arm = self.arduinobot.get_planning_component("arm")
        self.arduinobot_gripper = self.arduinobot.get_planning_component("gripper")
        self.logger = get_logger("moveit_py.control")
        self.is_busy = False
        self.lock = Lock()

        # self.robot_model = self.arduinobot.get_robot_model()
        # self.arm_state = RobotState(self.arduinobot.get_robot_model())
        # self.gripper_state = RobotState(self.arduinobot.get_robot_model())

        # init param
        # self.arr_init_state = np.array([0.0, 0.0, 0.0, 0.0])
        # self.arr_drop_state = np.array([0.0, 0.0, 0.0, 0.0])
        # self.height = 0.18
        # self.pick_height = 0.126
        # self.carrying_height = 0.3
        # self.init_angle = -0.3825

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/target_point',
            self.listener_callback,
            10)

    # function to move a gripper
    def move_to(self, arr):
        self.get_logger().info("Received goal request move_to " )
        arm_state = RobotState(self.arduinobot.get_robot_model())
  
        arm_state.set_joint_group_positions("arm", arr)
        self.arduinobot_arm.set_start_state_to_current_state()
        
        self.arduinobot_arm.set_goal_state(robot_state = arm_state)

        # plan_and_execute(self, self.logger, sleep_time=3.0)
        arm_plan_result = self.arduinobot_arm.plan()
        # plan to goal
        if arm_plan_result:
            self.logger.info("Planner SUCCEED, moving the arm")
            self.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
        else:
            self.logger.info("One or more planners failed!")
        
        self.logger.info("move_to Plan succeeded")
        time.sleep(3.0)
        
    # function to move arm and gripper to specific pose
    def default_action(self, action):
        self.get_logger().info("Received default_action " + action )

        arm_joint_goal = []
        gripper_joint_goal = []
        arm_state = RobotState(self.arduinobot.get_robot_model())
        gripper_state = RobotState(self.arduinobot.get_robot_model())
        if action == 'init':
            arm_joint_goal = np.array([0.0, 0.0, 0.0, 1.6])
            gripper_joint_goal = np.array([-0.7, 0.7])
        elif action == 'drop_state':
            arm_joint_goal = np.array([1.7, 0.0, 0.0, 0.0])
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().info("no such action")

        arm_state.set_joint_group_positions("arm", arm_joint_goal)
        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)

        self.arduinobot_arm.set_start_state_to_current_state()
        self.arduinobot_gripper.set_start_state_to_current_state()

        self.arduinobot_arm.set_goal_state(robot_state=arm_state)
        self.arduinobot_gripper.set_goal_state(robot_state=gripper_state)

        # plan_and_execute(self, self.logger, sleep_time=3.0)
        arm_plan_result = self.arduinobot_arm.plan()
        gripper_plan_result = self.arduinobot_gripper.plan()
        # plan to goal
        if arm_plan_result and gripper_plan_result:
            self.logger.info("Planner SUCCEED, moving the arm and the gripper")
            self.arduinobot.execute(arm_plan_result.trajectory, controllers=[])
            self.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.logger.info("One or more planners failed!")
        
        self.logger.info(action + " default_action Plan succeeded")
        time.sleep(3.0)


        # plan_and_execute(self, self.logger, sleep_time=3.0)


    # function for a gripper action
    def gripper_action(self, action):
        self.get_logger().info("Received gripper_action " + action )
        gripper_state = RobotState(self.arduinobot.get_robot_model())

        gripper_joint_goal = np.array([0.0, 0.0])
        if action == 'open':
            gripper_joint_goal = np.array([-0.7, 0.7])
        if action == 'open2':
            gripper_joint_goal = np.array([-1.4, 1.4])
        elif action == 'close':
            gripper_joint_goal = np.array([0.0, 0.0])
        else:
            self.get_logger().info("no such action")

        gripper_state.set_joint_group_positions("gripper", gripper_joint_goal)
        self.arduinobot_gripper.set_start_state_to_current_state()
        self.arduinobot_gripper.set_goal_state(robot_state=gripper_state)

        # plan_and_execute(self, self.logger, sleep_time=1.0)
        # plan_and_execute(self, self.logger, sleep_time=3.0)
        # arm_plan_result = self.arduinobot_arm.plan()
        gripper_plan_result = self.arduinobot_gripper.plan()
        # plan to goal
        if gripper_plan_result:
            self.logger.info("Planner SUCCEED, moving the gripper")
            self.arduinobot.execute(gripper_plan_result.trajectory, controllers=[])
        else:
            self.logger.info("One or more planners failed!")
        
        self.logger.info("gripper_action Plan succeeded")
        time.sleep(3.0)


    def listener_callback(self, data):
        print("LOCK:", self.lock)
        with self.lock:
            if self.is_busy:
                return
            self.is_busy = True
        
        arr_np = np.array(data.data, dtype=np.float64)
        self.get_logger().info(f"{arr_np}")
        self.move_to([0.0, 0.0, 0.0, 1.6])
        self.gripper_action('open')

        # ham tinh toan goc, tra ve float arr 4 phan tu
        self.move_to(arr_np)

        self.gripper_action("close")

        self.default_action("drop_state")
        # self.move_to([1.7, 0.0, 0.0, 0.0])

        self.gripper_action("open2")

        self.move_to([0.0, 0.0, 0.0, 1.6])
        # self.default_action("init")
        

        with self.lock:
            self.is_busy = False
        


if __name__ == '__main__':
    rclpy.init(args=None)

    controller = Controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()
