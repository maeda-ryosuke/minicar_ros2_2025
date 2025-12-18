import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import os

class PositionLogger(Node):
    def __init__(self):
        super().__init__('position_logger')
        # EKFの出力トピック名を指定
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',  
            self.listener_callback,
            10)
        
        # ホームディレクトリにCSVファイルを作成
        file_path = os.path.join(os.path.expanduser('~'), 'position_log.csv')
        self.csv_file = open(file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # ヘッダーを書き込む
        self.csv_writer.writerow(['timestamp', 'x', 'y'])
        self.start_time = None
        self.get_logger().info(f"Logging position data to {file_path}")
        self.get_logger().info("Press Ctrl+C to stop logging.")

    def listener_callback(self, msg):
        # タイムスタンプを取得
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.start_time is None:
            self.start_time = current_time
            
        # 経過時間を計算
        elapsed_time = current_time - self.start_time
        
        # 位置データを取得
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # CSVに書き込む
        self.csv_writer.writerow([elapsed_time, pos_x, pos_y])

    def destroy_node(self):
        # ノードが終了するときにファイルを閉じる
        self.csv_file.close()
        self.get_logger().info("File closed.")
        super().destroy_node()

def main(args=None):
    print("処理を開始します")
    rclpy.init(args=args)
    position_logger = PositionLogger()
    try:
        rclpy.spin(position_logger)
    except KeyboardInterrupt:
        pass
    finally:
        print("処理を終了します")
        position_logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()