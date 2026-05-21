import sys
from math import cos, pi, sin

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from rclpy.action import ActionClient
from rclpy.node import Node

Joint_NAMES = ("joint1", "joint2", "joint3", "joint4")
LINK_LENGTH = (0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040)


class MoveGroupPythonInterface(Node):
    def __init__(self):
        super().__init__("move_group_python_interface")

        self.action_client = ActionClient(self, MoveGroup, "move_action")

        self.GROUP_NAME = "ldsc_arm"

        self.get_logger().info("Waiting for move_group action server...")
        self.action_client.wait_for_server()
        self.get_logger().info("MoveGroup Interface Initialized")

    def go_to_joint_state(
        self,
        joint_angles: tuple[float, float, float, float],
    ) -> None:
        joint_constraints = [
            JointConstraint(
                joint_name=name,
                position=angle,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for name, angle in zip(Joint_NAMES, joint_angles)
        ]
        constraints = Constraints(joint_constraints=joint_constraints)

        motion_plan_request = MotionPlanRequest(
            group_name=self.GROUP_NAME,
            num_planning_attempts=10,
            allowed_planning_time=5.0,
            goal_constraints=[constraints],
        )

        goal_msg = MoveGroup.Goal(
            request=motion_plan_request,
            planning_options=PlanningOptions(plan_only=False, replan=True),
        )

        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("Motion executed successfully")
        else:
            self.get_logger().error(f"Motion failed with error code: {result.error_code.val}")


# def Your_IK(x: float, y: float, z: float, pitch=pi/2) -> tuple[float, float, float, float]:
#     '''
#     Write your code here!
#     x,y,z,q is in world frame (the same as link0 frame)
#     The end-effector should parallel to the ground.
#     '''
#     # TODO


#     # joint_angle = your_IK_solution
#     joint_angle = [0.0, 0.0, 0.0, 0.0]
#     return joint_angle

def Your_IK(x: float, y: float, z: float, pitch=pi/2) -> tuple[float, float, float, float]:
    # 根據 LINK_LENGTH 定義長度
    l0, l1, l2, l3, l4, l5 = 0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040

    # 1. 計算 Theta 1 (底座旋轉)
    theta1 = np.arctan2(y, x)


    # 2. 定義目標平面座標 (d_target, z_target)
    d_target = np.sqrt(x**2 + y**2) - l4 
    z_target = z - (l0 + l1 - l5) 

    # 3. 求解 theta3 (使用餘弦定理)
    # 這裡的 D 是從肩部到腕部的直線距離
    D_sq = d_target**2 + z_target**2
    D = np.sqrt(D_sq)

    # 根據餘弦定理: D^2 = l2^2 + l3^2 - 2*l2*l3*cos(pi - theta3)
    # 簡化後得到 cos(theta3)
    cos_t3 = (D_sq - l2**2 - l3**2) / (2 * l2 * l3)
    
    # 數值保護：確保目標點在工作範圍內
    cos_t3 = np.clip(cos_t3, -1.0, 1.0)
    theta3 = np.arccos(cos_t3) # 這裡得到的是肘部向上的解，若要向下則取負值

    # 4. 求解 theta2
    # 利用三角形幾何關係：theta2 = 總仰角(phi) - 內部夾角(beta)
    # 注意：因為你的公式中 z 對應 cos，d 對應 sin，所以 atan2 的順序是 (d, z)
    phi = np.arctan2(d_target, z_target) 
    
    cos_beta = (l2**2 + D_sq - l3**2) / (2 * l2 * D)
    cos_beta = np.clip(cos_beta, -1.0, 1.0)
    beta = np.arccos(cos_beta)
    
    theta2 = phi - beta

    # 5. 求解 theta4 (根據約束條件)
    # theta2 + theta3 + theta4 = pi/2
    theta4 = (pi / 2) - theta2 - theta3

    print(float(theta1), float(theta2), float(theta3), float(theta4))
    return [float(theta1), float(theta2), float(theta3), float(theta4)]


def main():
    rclpy.init(args=sys.argv)

    try:
        path_object = MoveGroupPythonInterface()

        print("Press Ctrl+C to exit")

        while rclpy.ok():
            try:
                print("\n--- Enter Target Position ---")
                x_input = float(input("x: "))
                y_input = float(input("y: "))
                z_input = float(input("z: "))

                path_object.go_to_joint_state(Your_IK(x_input, y_input, z_input))

            except ValueError as e:
                print(f"Invalid input: {e}")
                print("Moving to Home Position...")
                path_object.go_to_joint_state((0.0, -pi / 2, pi / 2, 0.0))

            except Exception as e:
                print(f"Error occurred: {e}")
                print("Moving to Home Position...")
                path_object.go_to_joint_state((0.0, -pi / 2, pi / 2, 0.0))

    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    finally:
        if "path_object" in locals():
            path_object.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
