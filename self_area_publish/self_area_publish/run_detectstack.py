import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import time
import json
import warnings
from tf_transformations import euler_from_quaternion
import math

class SelfAreaPublishNode(Node):
    """
    オドメトリを購読し、自己位置のエリアを判定し、publishするノード
    スタック検知機能付き
    """

    def __init__(self, args=None):
        super().__init__('self_area_publish_node', cli_args=args)
                
        # 最新のオドメトリデータを保存する変数
        self.latest_odom = Odometry()
        
        # パブリッシャーの設定
        self.state_publisher = self.create_publisher(
            String,
            '/self_area/state',
            30
        )
        # Subscriberの設定
        self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback, 
            10)
        
        # 1. 状態管理変数
        self.current_state = 'STATE_1'
        self.pre_stack_state = 'STATE_1' # スタックからの復帰用
        
        # 周回数管理
        self.lap_count = 1
        self.lap_corn_count = 0
        self.last_relative_angle = None
        
        # スタック検知用変数
        self.pos_history = []          # 座標履歴 (x, y)
        self.stack_timer_count = 0     # STACK状態を維持するカウンタ
        self.STUCK_THRESHOLD = 0.02    # 1秒間にこの距離(m)以上動かなければスタックと判定
        self.HISTORY_SIZE = 30         # 30Hz設定のため30回で1秒分
        self.STACK_MAINTAIN_COUNT = 60 # 30Hz設定のため60回で2秒分

        # パラメータ設定
        self.declare_parameter('start_pos', 3) 
        self.declare_parameter('corn_lap', 0)
        
        start_pos_param = self.get_parameter('start_pos').get_parameter_value().integer_value
        self.corn_lap_param = self.get_parameter('corn_lap').get_parameter_value().integer_value

        # チェックポイント座標の定義
        self.CHECKPOINTS = {
            'CP_A': {'x1': 0.6, 'y1': -3.3, 'x2': 0.0, 'y2': -6.0},
            'CP_B': {'x1': -3.6, 'y1': -1.9, 'x2': -4.1, 'y2': -3.3},
            'CP_C': {'x1': -3.1, 'y1': 0.7, 'x2': -3.6, 'y2': -0.7},
            'CP_D': {'x1': -3.1, 'y1': 0.7, 'x2': -3.6, 'y2': -0.7}
        }
        self.CORN_POS = [0.0, -1.9]
        
        # チェックポイントをスタート位置に応じて更新
        self.checkpoint_update(start_pos_param)
        
        # 状態ごとのハンドラー
        self.state_handlers = {
            'STATE_1': self.handle_state_1_start,
            'STATE_2': self.handle_state_2_pass_a,
            'STATE_3': self.handle_state_3_pass_b,
            'STATE_4': self.handle_state_4_pass_c,
            'STATE_PARKING': self.handle_parking,
            'STACK': self.handle_stack, # スタック状態ハンドラー
        }
        
        # タイマー設定 (30Hz)
        timer_period = 1.0 / 30.0
        self.create_timer(timer_period, self.control_loop_callback)
        self.get_logger().info(f'Current State: {self.current_state}') 

    def is_goal_reached(self, current_x, current_y, checkpoint_key):
        cp = self.CHECKPOINTS[checkpoint_key]
        is_x_within = (cp['x2'] <= current_x <= cp['x1'])
        is_y_within = (cp['y2'] <= current_y <= cp['y1'])
        return is_x_within and is_y_within

    def check_stack_detection(self, x, y):
        """ 1秒間静止しているか判定するロジック """
        self.pos_history.append((x, y))
        
        if len(self.pos_history) > self.HISTORY_SIZE:
            # 1秒前(最も古いデータ)を取得
            old_x, old_y = self.pos_history.pop(0)
            # ユーークリッド距離で移動量を計算
            dist = math.sqrt((x - old_x)**2 + (y - old_y)**2)
            
            # 距離が閾値以下ならスタック判定
            if dist < self.STUCK_THRESHOLD:
                return True
        return False

    def handle_stack(self):
        """ 状態: STACK (2秒間維持) """
        self.stack_timer_count += 1
        if self.stack_timer_count >= self.STACK_MAINTAIN_COUNT:
            self.get_logger().info("Stack recovered. Returning to previous state.")
            self.stack_timer_count = 0
            self.pos_history.clear() # 判定履歴をリセット
            self.transition_to(self.pre_stack_state)

    def corn_lap_judge(self, x, y):
        diff_x = x - self.CORN_POS[0]
        diff_y = y - self.CORN_POS[1]
        angle_rad = math.atan2(diff_y, diff_x)
        
        if self.last_relative_angle is None:
            self.last_relative_angle = angle_rad
            return False
        
        if angle_rad > 0.0 and self.last_relative_angle <= 0.0:
            self.lap_corn_count += 1
        self.last_relative_angle = angle_rad

        if (self.lap_corn_count >= 1) and self.corn_lap_param == 0:
            return True
        if (self.lap_corn_count >= 3) and self.corn_lap_param != 0:
            return True
        return False
        
    def handle_state_1_start(self):
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
        if self.is_goal_reached(current_x, current_y, 'CP_A'):
            self.transition_to('STATE_2')

    def handle_state_2_pass_a(self):
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
        if self.is_goal_reached(current_x, current_y, 'CP_B'):
            if self.corn_lap_param == 0:
                self.transition_to('STATE_4')
            else:
                self.transition_to('STATE_3')

    def handle_state_3_pass_b(self):
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
        if self.corn_lap_judge(current_x, current_y):
            self.transition_to('STATE_4')
               
    def handle_state_4_pass_c(self):
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
        if self.is_goal_reached(current_x, current_y, 'CP_D'):
            if self.lap_count >= 3:
                self.transition_to('STATE_PARKING')
                return
            self.transition_to('STATE_1')
            self.lap_count += 1
            self.lap_corn_count = 0

    def handle_parking(self):
        pass

    def transition_to(self, new_state):
        if self.current_state != new_state:
            self.get_logger().info(f'Transition: {self.current_state} -> {new_state}')
            self.current_state = new_state

    def checkpoint_update(self, start_pos):
        if not isinstance(start_pos, int):
            raise TypeError(f"Invalid type: {type(start_pos).__name__}")
        
        offset_x = 0.0 
        if start_pos == 1:
            offset_x = 0.0
        elif start_pos == 2:
            offset_x = 1.8
        elif start_pos == 3:
            offset_x = 3.6
        else:
            offset_x = 3.6
            
        for checkpoint_key, coords in self.CHECKPOINTS.items():
            coords['x1'] += offset_x
            coords['x2'] += offset_x
    
        self.CORN_POS[0] += offset_x
        self.get_logger().info(f"Checkpoints updated with offset: {offset_x}")
        
    def odometry_callback(self, msg):
        self.latest_odom = msg

    def control_loop_callback(self):
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y

        # --- スタック検知処理 ---
        # すでにSTACK状態でない場合のみ検知を行う
        if self.current_state != 'STACK':
            if self.check_stack_detection(current_x, current_y):
                self.get_logger().warn("STACK DETECTED!")
                self.pre_stack_state = self.current_state # 復帰先を保存
                self.transition_to('STACK')
        
        # 状態ハンドラーの実行
        handler = self.state_handlers.get(self.current_state)
        if handler:
            handler()
        else:
            self.get_logger().error(f"Unknown state: {self.current_state}")

        # パブリッシュ
        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)
        
    def on_shutdown(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SelfAreaPublishNode(args=args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()