import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'kalmanfilter'
    
    # 設定ファイル(YAML)のパスを取得
    config_file_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf_config.yaml'
    )

    # robot_localizationのekf_nodeを起動
    start_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file_path]
    )
    # RViz2の起動設定
    start_rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'rviz', 'imu_test.rviz')],
        # もしRViz設定ファイルがなければ上の行はコメントアウトし、下の行を使う
        output='screen'
    )
    
    # static_transform_publisherのノード定義を追加
    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_base_to_imu',
        output='screen',
        arguments=[
            # x, y, z, roll, pitch, yaw の順
            '0.223', '0.07', '0',  # 位置 (x, y, z) [m]
            '0', '0', '0',  # 回転 (roll, pitch, yaw) [rad]
            
            # 親フレームID と 子フレームID
            'base_link',
            'imu_link'
        ]
    )
    
    return LaunchDescription([
        start_robot_localization_node,
        start_rviz2_node,
        static_tf_publisher_node,
    ])