import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty

msg = """
控制四個關節的按鍵說明:
---------------------------
關節 1: W (+) / S (-)
關節 2: E (+) / D (-)
關節 3: R (+) / F (-)
關節 4: T (+) / G (-)

CTRL-C 退出
"""

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            # '/four_joints_position_controllers/commands',
            'real_robot_arm_joint',
            10
        )

        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_limit = [1.2, 2.0, 1.67, 1.57079632]
        self.step = 0.1

    def update_and_publish(self, joint_idx, delta):
        # 更新數值並限制在範圍內
        new_val = self.joint_positions[joint_idx] + delta
        limit = self.joint_limit[joint_idx]
        
        # 限制在 [-limit, limit] 之間
        self.joint_positions[joint_idx] = max(min(new_val, limit), -limit)
        
        # 建立並發佈消息
        joint_angle_msg = Float64MultiArray()
        joint_angle_msg.data = self.joint_positions
        self.publisher_.publish(joint_angle_msg)
        
        self.get_logger().info(f'發佈關節角度: {self.joint_positions}')

def get_key(settings):
    # 設置終端為原始模式以讀取單一按鍵
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardControl()

    print(msg)

    try:
        while rclpy.ok():
            key = get_key(settings).lower()
            
            if key == 'w':
                node.update_and_publish(0, node.step)
            elif key == 's':
                node.update_and_publish(0, -node.step)
            elif key == 'e':
                node.update_and_publish(1, node.step)
            elif key == 'd':
                node.update_and_publish(1, -node.step)
            elif key == 'r':
                node.update_and_publish(2, node.step)
            elif key == 'f':
                node.update_and_publish(2, -node.step)
            elif key == 't':
                node.update_and_publish(3, node.step)
            elif key == 'g':
                node.update_and_publish(3, -node.step)
            elif key == '\x03':  # CTRL-C
                break
                
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
