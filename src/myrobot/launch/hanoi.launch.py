import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    MY_ROBOT_PKG = 'myrobot'   
    MY_PLAN_PKG = 'myplan'
    VOICEGPT_PKG = 'voicegpt'

    myplan_share = get_package_share_directory(MY_PLAN_PKG)
    
    # =========================================================================
    # 0. 宣告命令列參數
    # =========================================================================
    camera_arg = DeclareLaunchArgument('camera', default_value='false', description='是否啟動影像辨識節點')
    voice_arg = DeclareLaunchArgument('voice', default_value='false', description='是否啟動語音GPT節點')

    use_camera = LaunchConfiguration('camera')
    use_voice = LaunchConfiguration('voice')

    # =========================================================================
    # 1. 第一步：直接啟動 MoveIt 2 模擬環境 (demo.launch.py)
    # =========================================================================
    start_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(myplan_share, 'launch', 'demo.launch.py'))
    )

    # =========================================================================
    # 2. 第二步：定義其餘三個後續節點（加上對應的啟動條件與參數）
    # =========================================================================
    visual_recognition_node = Node(
        package=MY_ROBOT_PKG,
        executable='visual_recognition',
        name='visual_recognition',
        output='screen',
        condition=IfCondition(use_camera) 
    )

    voicegpt_node = Node(
        package=VOICEGPT_PKG,
        executable='voicegpt_hanoi',
        name='voicegpt_hanoi',
        output='screen',
        condition=IfCondition(use_voice)
    )

    hanoi_planner_node = Node(
        package=MY_ROBOT_PKG,
        executable='hanoi_planner',
        name='hanoi_planner',
        output='screen',
        prefix='xterm -e',
        arguments=[
            PythonExpression(["'-camera' if '", use_camera, "' == 'true' else ''"]),
            PythonExpression(["'-voice' if '", use_voice, "' == 'true' else ''"])
        ]
    )

    # =========================================================================
    # 3. 核心設計：使用 TimerAction 強迫「等 5 秒」才釋放後續節點
    # =========================================================================
    
    # 5.0 秒一到，同時釋放影像與語音節點（如果命令行參數有開啟的話）
    delay_sub_nodes = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="[Launch] 已等待 5 秒，MoveIt 應已就緒。現在載入後續功能節點..."),
            visual_recognition_node,
            voicegpt_node
        ]
    )

    # 為了保證大腦決策節點（hanoi_planner）是最晚啟動的，我們讓它多等 1 秒（總共等 6 秒）
    delay_planner_node = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[Launch] 所有基礎通訊已就緒，壓軸載入 Hanoi Planner 決策核心！"),
            hanoi_planner_node
        ]
    )

    return LaunchDescription([
        camera_arg,
        voice_arg,
        start_moveit,       # 0秒：立刻開跑 MoveIt 模擬與 RViz
        delay_sub_nodes,    # 5秒：放行啟動相機、語音
        delay_planner_node  # 6秒：放行啟動大腦
    ])