import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 重い処理のためにマルチスレッド化
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
    """

    def __init__(self, args=None):
        super().__init__('self_area_publish_node', cli_args=args)
                
        # 最新のオドメトリデータを保存する変数
        self.latest_odom = Odometry()
        
        # パブリッシャーの設定
        self.state_publisher = self.create_publisher(
            String,
            '/self_area/state', # トピック名
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
        
        # 周回数
        self.lap_count = 1
        # コーン周回数
        self.lap_corn_count = 0
        # 前回の角度(コーン周回判定用)
        self.last_relative_angle = None
        # 2. 'start_pos' パラメータを宣言し、デフォルト値を 3 と設定
        self.declare_parameter('start_pos', 3) 
        # コーン周回させるかどうか。デフォルトは0(周回しない)
        self.declare_parameter('corn_lap', 0)
        
        # 3. パラメータ値を取得
        start_pos_param = self.get_parameter('start_pos').get_parameter_value().integer_value
        self.corn_lap_param = self.get_parameter('corn_lap').get_parameter_value().integer_value
        # チェックポイント座標の定義
        # スタート3の中央からスタートしたと仮定したときの座標
        # スタート位置補正など入れるか、スタート位置を必ず同じにするようにする
        # x1 > x2, y1 > y2
        self.CHECKPOINTS = {
            'CP_A': {'x1': 0.6, 'y1': -3.3, 'x2': 0.0, 'y2': -6.0}, # 状態1 -> 状態2
            'CP_B': {'x1': -3.6, 'y1': -1.9, 'x2': -4.1, 'y2': -3.3}, # 状態2 -> 状態3
            'CP_C': {'x1': -3.1, 'y1': 0.7, 'x2': -3.6, 'y2': -0.7},  # 状態3 -> 状態1 (ループ)
            'CP_D': {'x1': -3.1, 'y1': 0.7, 'x2': -3.6, 'y2': -0.7}  # 状態3 -> 状態1 (ループ)
        }
        
        self.CORN_POS = [0.0, -1.9]
        # 5. 取得したパラメータ値でチェックポイントを更新
        self.checkpoint_update(start_pos_param)
        
        # 2. 状態ごとの実行関数を格納した辞書 (Pythonにおけるswitchの代わり)
        self.state_handlers = {
            'STATE_1': self.handle_state_1_start,
            'STATE_2': self.handle_state_2_pass_a,
            'STATE_3': self.handle_state_3_pass_b,
            'STATE_4': self.handle_state_4_pass_c,
            'STATE_PARKING': self.handle_parking,
        }
        
        # 3. タイマー設定 (制御ループ)
        timer_period = 1.0 / 30.0
        self.create_timer(timer_period, self.control_loop_callback)
        self.get_logger().info(f'Current State: {self.current_state}') 
        # self.get_logger().info('Odometry PWM Control Node initialized. Control frequency: 10 Hz')
        
    def is_goal_reached(self, current_x, current_y, checkpoint_key):
            """
            現在の座標が指定されたチェックポイントの許容誤差内にあるか判定する
            """
            # ゴールフラグ
            reach_goal = False
            
            # チェックポイントの座標を取得
            cp = self.CHECKPOINTS[checkpoint_key]
            
            # xの範囲、yの範囲に入っているか判定
            is_x_within = (cp['x2'] <= current_x <= cp['x1'])
            is_y_within = (cp['y2'] <= current_y <= cp['y1'])
            
            # x,yともに範囲に入っていればフラグをTrueに
            if is_x_within and is_y_within:
                reach_goal = True
            
            return reach_goal
    
    def corn_lap_judge(self, x, y):
        """
        コーン周回をしたかどうか判定する関数
        """
        diff_x = x - self.CORN_POS[0]
        diff_y = y - self.CORN_POS[1]
        # 今のコーンとの相対角度を計算
        angle_rad = math.atan2(diff_y, diff_x)
        
        # 初回実行時の処理
        if self.last_relative_angle is None:
            self.last_relative_angle = angle_rad
            # 初回は周回ではないため False を返す
            return False
        
        # コーンとの相対角度が0以上かつ、前回角度が0以下だったらコーン周回カウント+1
        print("current_x : " + str(x))
        print("current_y : " + str(y))
        print("diff_x : " + str(diff_x))
        print("diff_y : " + str(diff_y))
        print("angle_rad : " + str(angle_rad))
        # print("last_angle : " + str(self.last_relative_angle))
        if angle_rad > 0.0 and self.last_relative_angle <= 0.0:
            self.lap_corn_count += 1
        self.last_relative_angle = angle_rad

        # 6. 3周完了したか判定
        if self.lap_corn_count >= 3:
            return True
        else:
            return False
        
    def handle_state_1_start(self):
        """ 
        状態 1: スタート (CP_Aに向かって走行) 
        """
        # 現在のオドメトリを取得
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
                
        # 2. 遷移判定: CP_Aに到達したか
        if self.is_goal_reached(current_x, current_y, 'CP_A'):
            print("current_x: " + str(current_x))
            print("current_y: " + str(current_y))
            self.transition_to('STATE_2') # 状態2へ

    def handle_state_2_pass_a(self):
        """ 
        状態 2: CP_A通過後 (CP_Bに向かって走行) 
        """
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
                
        # 2. 遷移判定: CP_Bに到達したか
        if self.is_goal_reached(current_x, current_y, 'CP_B'):
            print("current_x: " + str(current_x))
            print("current_y: " + str(current_y))

            if self.corn_lap_param==0:
                self.transition_to('STATE_4') # 状態3へ
            else:
                self.transition_to('STATE_3') # 状態4へ

    def handle_state_3_pass_b(self):
        """ 
        状態 3: CP_B通過後 コーンを周回したらSTATE_4へ
        """
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
        orientation_q = self.latest_odom.pose.pose.orientation
        # # euler_from_quaternion 関数は、[x, y, z, w] の順序のリストを引数に取ります。
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # # 3. euler_from_quaternion関数でロール(roll), ピッチ(pitch), ヨー(yaw)に変換
        # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        # current_th = yaw  # ラジアン値 (-π から +π)     
        
        # 2. 遷移判定: CP_Cに到達したか
        # print("lap_corn_count : " + str(self.lap_corn_count))
        if self.corn_lap_judge(current_x, current_y):
            print("lap_corn_count : " + str(self.lap_corn_count))
            print("コーン周回数が3になったためSTATE_4へ遷移します")

            self.transition_to('STATE_4') # 状態1に戻る
               
               
    def handle_state_4_pass_c(self):
        """
        状態4: CP_C通過後
        """
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
                        
        # 2. 遷移判定: CP_Cに到達したか
        if self.is_goal_reached(current_x, current_y, 'CP_D'):
            print("current_x: " + str(current_x))
            print("current_y: " + str(current_y))

            # lap_countが3だったら駐車に移行
            if self.lap_count >= 3:
                self.transition_to('STATE_PARKING')
                return

            self.transition_to('STATE_1') # 状態1に戻る
            # 周回数を+1
            self.lap_count += 1
            # コーン周回カウントをリセット(各ラップでカウントするため)
            self.lap_corn_count = 0

    def handle_parking(self):
        """
        状態 4: 状態3→1に2回遷移したら駐車に移行 
        今は何もしない 今後追加するかも？
        """
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y

    def transition_to(self, new_state):
            """ 状態を遷移させ、ログを出力する """
            self.get_logger().info(f'Transition: {self.current_state} -> {new_state}')
            self.current_state = new_state

    def checkpoint_update(self, start_pos):
        """
        チェックポイントをアップデートする関数
        """
    # エリア判定ロジックが int 以外の型を受け付けないように、例外を発生させるのがより堅牢
        if not isinstance(start_pos, int):
            raise TypeError(f"引数の型が不正です。int型が期待されましたが、{type(start_pos).__name__}型が渡されました。")
        
        # 補正値を初期化
        offset_x = 0.0 

        if start_pos == 1:
            print("スタート位置1としてチェックポイントを設定します (オフセット: 0.0)")
            offset_x = 0.0 # 補正なし
        elif start_pos == 2:
            print("スタート位置2としてチェックポイントを更新します (オフセット: +1.8)")
            offset_x = 1.8
        elif start_pos == 3:
            print("スタート位置3としてチェックポイントを更新します (オフセット: +3.6)")
            offset_x = 3.6
        else:
            # 1, 2, 3 以外の値が渡された場合の処理（この構成では発生しにくいが安全のために）
            print("警告: 1, 2, 3 以外の値が入力されました。デフォルトのスタート位置3を使用します。")
            offset_x = 3.6
            
        # --- 全てのチェックポイントにオフセットを適用 ---
        for checkpoint_key, coords in self.CHECKPOINTS.items():
            coords['x1'] += offset_x
            coords['x2'] += offset_x
    
        # コーンにも適用
        self.CORN_POS[0] += offset_x
        self.get_logger().info(f"最終チェックポイント座標: {self.CHECKPOINTS}")
        
    # --- コールバック関数: オドメトリ受信 ---
    def odometry_callback(self, msg):
        """
        /odometry/filtered トピックからデータを受け取り、最新の値を保存する
        """
        # データを受信したら、最新のデータをクラス変数に上書き保存する
        self.latest_odom = msg
        # self.get_logger().info("odom_callback")
    # --- コールバック関数: 制御ループ (タイマー駆動) ---
    def control_loop_callback(self):
        """
        タイマーによって定周期で呼び出される。エリアの判定とpublish
        """        
        # 現在の状態に対応するハンドラー関数を実行
        print(self.current_state)
        handler = self.state_handlers.get(self.current_state)
        
        if handler:
            handler()
        else:
            self.get_logger().error(f"Unknown state: {self.current_state}")

        # パブリッシュ
        msg = String()
        msg.data = self.current_state # 現在の状態文字列をセット
        self.state_publisher.publish(msg)
        self.get_logger().debug(f'Published State: {self.current_state}')
        
    def on_shutdown(self):
        """
        ノード終了時に実行されるクリーンアップ処理
        """

def main(args=None):
    
    # # --- 標準入力からスタート位置を取得 ---
    # while True:
    #     try:
    #         # 標準入力でスタート位置を選択
    #         input_pos = input("スタート位置を選択してください (1, 2, 3): ")
            
    #         # 入力が整数かチェック
    #         start_pos = int(input_pos)
            
    #         if start_pos in [1, 2, 3]:
    #             break
    #         else:
    #             print("1, 2, 3 のいずれかを入力してください。")
    #     except ValueError:
    #         print("無効な入力です。整数を入力してください。")
    
    rclpy.init(args=args)
    node = SelfAreaPublishNode(args=args)
    
    # 重い制御（推論やPWM書き込み）とデータ受信を並列処理するためにMultiThreadedExecutorを使用
    # executor = MultiThreadedExecutor(num_threads=2)
    # executor.add_node(node)

    try:
        # executor.spin()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了処理と安全なシャットダウン
        node.on_shutdown()
        # executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()