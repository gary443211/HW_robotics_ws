import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import termios
import tty

# 輔助函式：讀取單一按鍵而不需按 Enter
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        # 建立 Publisher，主題為 /real_robot_arm_joint
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/real_robot_arm_joint',
            10
        )

        # 初始關節位置與極限設定
        self.joint_positions = [0.0, 0.0, 0.0, 0.0]
        self.joint_limit = [1.2, 2.0, 1.67, 1.57079632]
        
        # 初始電磁鐵狀態 (0.0: 關閉, 1.0: 開啟)
        self.magnet_state = 0.0

    def publish_joints(self):
        msg = Float64MultiArray()
        # 將 4 個關節角度與 1 個電磁鐵狀態合併成 5 個元素的陣列，符合底層通訊要求
        msg.data = self.joint_positions + [self.magnet_state] 
        self.publisher_.publish(msg)
        
        # 調整 logger 輸出的格式，方便查看包含電磁鐵的完整陣列
        formatted_joints = [f"{j:.2f}" for j in self.joint_positions]
        self.get_logger().info(f'發送指令: 關節 [{", ".join(formatted_joints)}], 電磁鐵: {self.magnet_state}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()

    print("-" * 35)
    print("ROS2 機器手臂鍵盤控制已啟動 (含電磁鐵版)")
    print("w / e : 第 1 關節 +/- 0.1 rad")
    print("r / f : 第 2 關節 +/- 0.1 rad")
    print("t / g : 第 3 關節 +/- 0.1 rad")
    print("y / h : 第 4 關節 +/- 0.1 rad")
    print("m / n : 電磁鐵 開啟(1.0) / 關閉(0.0)")
    print("q     : 退出程式")
    print("-" * 35)

    try:
        while True:
            key = get_key()
            
            if key == 'q':
                break
            
            # 第 1 關節控制
            elif key == 'w':
                node.joint_positions[0] += 0.1
            elif key == 'e':
                node.joint_positions[0] -= 0.1
                
            # 第 2 關節控制
            elif key == 'r':
                node.joint_positions[1] += 0.1
            elif key == 'f':
                node.joint_positions[1] -= 0.1
                
            # 第 3 關節控制
            elif key == 't':
                node.joint_positions[2] += 0.1
            elif key == 'g':
                node.joint_positions[2] -= 0.1
                
            # 第 4 關節控制
            elif key == 'y':
                node.joint_positions[3] += 0.1
            elif key == 'h':
                node.joint_positions[3] -= 0.1

            # 電磁鐵控制
            elif key == 'm':
                node.magnet_state = 1.0
            elif key == 'n':
                node.magnet_state = 0.0

            # 限制關節角度不超過極限
            for i in range(4):
                if node.joint_positions[i] > node.joint_limit[i]:
                    node.joint_positions[i] = node.joint_limit[i]
                elif node.joint_positions[i] < -node.joint_limit[i]:
                    node.joint_positions[i] = -node.joint_limit[i]

            # 每次按鍵都會發布包含電磁鐵狀態的 5 個元素陣列
            node.publish_joints()

    except Exception as e:
        print(f"發生錯誤: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64MultiArray
# import sys
# import termios
# import tty

# # 輔助函式：讀取單一按鍵而不需按 Enter
# def get_key():
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch

# class KeyboardControl(Node):
#     def __init__(self):
#         super().__init__('keyboard_control')

#         # 建立 Publisher，主題為 /four_joints_position_controllers/commands [cite: 35, 52]
#         self.publisher_ = self.create_publisher(
#             Float64MultiArray,
#             '/real_robot_arm_joint',
#             # '/four_joints_position_controllers/commands',
#             10
#         )

#         # 初始關節位置與極限設定 [cite: 58]
#         self.joint_positions = [0.0, 0.0, 0.0, 0.0]
#         self.joint_limit = [1.2, 2.0, 1.67, 1.57079632]

#     def publish_joints(self):
#         msg = Float64MultiArray()
#         msg.data = self.joint_positions # [cite: 58]
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'發送關節角度: {self.joint_positions}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyboardControl()

#     print("-" * 30)
#     print("ROS2 機器手臂鍵盤控制已啟動")
#     print("w / e : 第 1 關節 +/- 0.1 rad [cite: 46, 47]")
#     print("r / f : 第 2 關節 +/- 0.1 rad [cite: 48]")
#     print("t / g : 第 3 關節 +/- 0.1 rad")
#     print("y / h : 第 4 關節 +/- 0.1 rad")
#     print("q     : 退出程式")
#     print("-" * 30)

#     try:
#         while True:
#             key = get_key()
            
#             if key == 'q':
#                 break
            
#             # 第 1 關節控制 [cite: 46, 47]
#             elif key == 'w':
#                 node.joint_positions[0] += 0.1
#             elif key == 'e':
#                 node.joint_positions[0] -= 0.1
                
#             # 第 2 關節控制 [cite: 48]
#             elif key == 'r':
#                 node.joint_positions[1] += 0.1
#             elif key == 'f':
#                 node.joint_positions[1] -= 0.1
                
#             # 第 3 關節控制
#             elif key == 't':
#                 node.joint_positions[2] += 0.1
#             elif key == 'g':
#                 node.joint_positions[2] -= 0.1
                
#             # 第 4 關節控制
#             elif key == 'y':
#                 node.joint_positions[3] += 0.1
#             elif key == 'h':
#                 node.joint_positions[3] -= 0.1

#             # 限制關節角度不超過極限（選擇性實作）
#             for i in range(4):
#                 if node.joint_positions[i] > node.joint_limit[i]:
#                     node.joint_positions[i] = node.joint_limit[i]
#                 elif node.joint_positions[i] < -node.joint_limit[i]:
#                     node.joint_positions[i] = -node.joint_limit[i]

#             node.publish_joints()

#     except Exception as e:
#         print(f"發生錯誤: {e}")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
