import threading
import time
from math import cos, pi, sin
from pathlib import Path

import numpy as np
import rclpy
import trimesh
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Bool, Header

# load functions from other file
from myrobot.hanoi_spawn_objects import load_mesh_from_file
from myrobot.IK_path_planning import Your_IK

Joint_NAMES = ("joint1", "joint2", "joint3", "joint4")
LINK_LENGTH = (0.0600, 0.0820, 0.1320, 0.1664, 0.0480, 0.0040)

"""Variable for end-effector"""
EefState = 0

"""Hanoi tower geometry"""
# You can measure these in Lab402
Tower_base = 0.0014  # Height of tower base
Tower_height = 0.025  # Height of each tower
Tower_overlap = 0.010  # Height of tower overlap

"""Hanoi tower station position"""
# You may want to slightly change this
STATION_POSITIONS = (
    (0.25, 0.15),
    (0.25, 0),
    (0.25, -0.15),
)

"""Hanoi tower mesh file path"""
WORKSPACE_PATH = str(Path(__file__).absolute().parent.parent.parent.parent)
MESH_DIR = f"{WORKSPACE_PATH}/src/myplan/mesh"
MESH_FILE_PATH = (
    f"{MESH_DIR}/tower1.stl",
    f"{MESH_DIR}/tower2.stl",
    f"{MESH_DIR}/tower3.stl",
)
for mesh in MESH_FILE_PATH:
    assert Path(mesh).exists(), "Mesh path error"


"""
Hint:
    The output of your "Hanoi-Tower-Function" can be a series of [x, y, z, eef-state], where
    1.xyz in world frame
    2.eef-state: 1 for magnet on, 0 for off
"""

class MoveGroupPythonInterface(Node):
    def __init__(self, executor: MultiThreadedExecutor):
        super().__init__("move_group_python_interface")

        self.joint_angles: list[float] | None = None

        self._executor = executor
        self.callback_group = ReentrantCallbackGroup()

        self.GROUP_NAME = "ldsc_arm"
        self.PLANNING_FRAME = "world"

        self.action_client = ActionClient(self, MoveGroup, "move_action")

        self.display_trajectory_publisher = self.create_publisher(
            DisplayTrajectory, "/move_group/display_planned_path", 20
        )

        self.pub_eef_state = self.create_publisher(Bool, "/SetEndEffector", 10)

        self.collision_object_publisher = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )

        self.attached_collision_object_publisher = self.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Waiting for joint states...")
        timeout = 10.0
        start_time = time.time()
        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_angles is not None:
                self.get_logger().info("Joint states received!")
                break
            if (time.time() - start_time) > timeout:
                self.get_logger().warn("Joint states not received within timeout")
                break

        self.get_logger().info("Waiting for trajectory action server...")
        if self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("Trajectory action server connected!")
        else:
            self.get_logger().error("Trajectory action server not available!")

        self.get_logger().info("MoveGroup Python Interface already initialized")

    def wait_for_state_update(self) -> None:
        self._executor.spin_once(timeout_sec=0.5)

    def add_box(
        self,
        *,
        box_name: str,
        box_pose: Pose,
        size: tuple[float, float, float],
    ) -> None:
        """
        Description:
            1. Add a box to rviz, Moveit_planner will think of which as an obstacle.
            2. An example is shown in the main function below.
            3. Google scene.add_box for more details
        """

        box = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=size,
        )

        collision_object = CollisionObject(
            header=Header(
                frame_id=self.PLANNING_FRAME,
                stamp=self.get_clock().now().to_msg(),
            ),
            id=box_name,
            primitives=[box],
            primitive_poses=[box_pose],
            operation=CollisionObject.ADD,
        )

        self.collision_object_publisher.publish(collision_object)

        self.get_logger().info(f"Added box: {box_name}")
        self.wait_for_state_update()

    def add_mesh(
        self,
        *,
        mesh_name: str,
        mesh_position: Point,
        file_path: str,
        scale: tuple[float, float, float],
    ) -> None:
        """
        Description:
            1. Add a mesh to rviz, Moveit_planner will think of which as an obstacle.
            2. An example is shown in the main function below.
        """
        pose = Pose(
            position=mesh_position,
            # adjust mesh orientation
            orientation=Quaternion(x=0.7071081, y=0.0, z=0.0, w=0.7071081),
        )
        collision_object = CollisionObject(
            header=Header(
                frame_id=self.PLANNING_FRAME,
                stamp=self.get_clock().now().to_msg(),
            ),
            id=mesh_name,
            meshes=[load_mesh_from_file(file_path, scale)],
            mesh_poses=[pose],
            operation=CollisionObject.ADD,
        )

        self.collision_object_publisher.publish(collision_object)

        self.get_logger().info(f"Added mesh: {mesh_name}")
        self.wait_for_state_update()

    def attach_object(self, *, object_name: str, link_name: str) -> None:
        """
        Description:
            1. Make sure the object has been added to rviz
            2. Attach a object to link_frame(usually 'link5'), and the object will move with the link_frame.
            3. Google scene.attach_box for more details
        """
        attached_object = AttachedCollisionObject(
            link_name=link_name,
            object=CollisionObject(id=object_name, operation=CollisionObject.ADD),
            touch_links=[link_name],
        )

        self.attached_collision_object_publisher.publish(attached_object)

        self.get_logger().info(f"Attached object: {object_name} to {link_name}")
        self.wait_for_state_update()

    def detach_object(self, *, object_name: str, link_name: str) -> None:
        """
        Description:
            1. Detach a object from link_frame(usually 'link5'), and the object will not move with the link_frame.
            2. An example is shown in the main function below.
            3. Google scene.detach_box for more details
        """
        attached_object = AttachedCollisionObject(
            link_name=link_name,
            object=CollisionObject(id=object_name, operation=CollisionObject.REMOVE),
        )

        self.attached_collision_object_publisher.publish(attached_object)

        self.get_logger().info(f"Detached object: {object_name} from {link_name}")
        self.wait_for_state_update()

    def joint_state_callback(self, msg: JointState):
        try:
            joint_pair: dict[str, float] = dict(zip(msg.name, msg.position))
            self.joint_angles = [joint_pair[name] for name in Joint_NAMES]
        except Exception as e:
            self.get_logger().error(f"Error in joint_state_callback: {str(e)}")

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

    def switch_magnet(self, on: bool) -> None:
        """
        Description:
            Because moveit only plans the path,
            you have to publish end-effector state for playing hanoi.
        """
        self.pub_eef_state.publish(Bool(data=on))
        self.get_logger().info(f"Published end effector state: {on}")

    def wait_for_state_update(self) -> None:
        self._executor.spin_once(timeout_sec=0.5)

def main(args=None):
    global EefState

    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    try:
        path_object = MoveGroupPythonInterface(executor)
        executor.add_node(path_object)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        path_object.add_mesh(
            mesh_name="tower1",
            mesh_position=Point(x=0.25, y=0.0, z=0.0),
            file_path=MESH_FILE_PATH[0],
            scale=(0.00095, 0.00095, 0.00095),
        )
        path_object.add_mesh(
            mesh_name="tower2",
            mesh_position=Point(x=0.25, y=0.15, z=0.0),
            file_path=MESH_FILE_PATH[1],
            scale=(0.00095, 0.00095, 0.00095),
        )
        path_object.add_mesh(
            mesh_name="tower3",
            mesh_position=Point(x=0.25, y=-0.15, z=0.0),
            file_path=MESH_FILE_PATH[2],
            scale=(0.00095, 0.00095, 0.00095),
        )

        path_object.add_box(
            box_name="box_1",
            box_pose=Pose(
                orientation=Quaternion(w=1.0),
                position=Point(x=0.25, y=0.075, z=0.25 / 2),
            ),
            size=(0.05, 0.005, 0.25),
        )
        path_object.add_box(
            box_name="box_2",
            box_pose=Pose(
                orientation=Quaternion(w=1.0),
                position=Point(x=0.25, y=-0.075, z=0.25 / 2),
            ),
            size=(0.05, 0.005, 0.25),
        )

        while rclpy.ok():
            try:
                """
                Modify this into a list of [x, y, z, 1 or 0]

                x_input=float(raw_input("x:  "))
                y_input=float(raw_input("y:  "))
                z_input=float(raw_input("z:  "))
                1 for end-effector on;0 for off
                
                
                path_object.joint_angles = Your_IK(x,y,z)
                path_object.go_to_joint_state() #path will automatically be published by moveit
                EefState = 0
                pub_EefState.publish(EefState)  #publish end-effector state


                """
                # TODO
                pass

            except ValueError as e:
                path_object.get_logger().error(f"Error: {str(e)}")
                path_object.go_to_joint_state((0.0, -pi / 2, pi / 2, 0.0))

            except KeyboardInterrupt:
                path_object.get_logger().info("Interrupted by user")
                break

    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback

        traceback.print_exc()

    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
