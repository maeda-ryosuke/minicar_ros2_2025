import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 重い処理のためにマルチスレッド化
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import json

class SelfAreaSubscribe(Node):
    """
    オドメトリを購読し、自己位置のエリアを判定し、publishするノード
    """

    def __init__(self):
        super().__init__('triple_dragon_node')
                
        # 最新のオドメトリデータを保存する変数
        self.latest_self_area = String()
        
        # Subscriberの設定
        self.create_subscription(
            String,
            '/self_area/state',
            self.self_area_callback,
            10)        

    # --- コールバック関数: オドメトリ受信 ---
    def self_area_callback(self, msg):
        """
        /odometry/filtered トピックからデータを受け取り、最新の値を保存する
        """
        # データを受信したら、最新のデータをクラス変数に上書き保存する
        self.latest_self_area = msg
        # self.get_logger().info("odom_callback")
        print(self.latest_self_area)
        
    def on_shutdown(self):
        """
        ノード終了時に実行されるクリーンアップ処理
        """

def main(args=None):
    rclpy.init(args=args)
    node = SelfAreaSubscribe()
    try:
        node.get_logger().info('Starting spin_once loop to read data externally.')
        
        # rclpy.spin_once を使用して、イベントが発生した場合にのみ処理を行うループ
        while 1:
            # 1. rclpy.spin_once() で、保留中のコールバック（メッセージ受信など）を処理する
            rclpy.spin_once(node, timeout_sec=0.1) 
            
            current_state = node.latest_self_area.data
            
            print(f"[{time.strftime('%H:%M:%S')}] Current State (External Read): {current_state}")
            
            # 読み取り頻度を調整するため、適度に待機
            time.sleep(1.0) 
            
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理と安全なシャットダウン
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()