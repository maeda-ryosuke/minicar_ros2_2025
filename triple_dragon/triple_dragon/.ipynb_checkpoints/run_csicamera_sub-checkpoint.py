import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 重い処理のためにマルチスレッド化
from std_msgs.msg import String
from sensor_msgs.msg import Image # ★ 追加: Imageメッセージのインポート
from cv_bridge import CvBridge # ★ 追加: cv_bridgeのインポート
import time
import json
import os
import glob
import argparse
import time
import threading
import json
import subprocess
import datetime
from pathlib import Path

import Fabo_PCA9685
import smbus
# from jetcam.csi_camera import CSICamera # ★ 削除: カメラ直接操作
import torch
from torch2trt import TRTModule
import cv2
from utils import preprocess

# ---------- Jetson モデル判定 (元コードを踏襲) ----------
try:
    import Jetson.GPIO as GPIO
    BOARD_NAME = GPIO.gpio_pin_data.get_data()[0]
except Exception as e:
    print(f"[WARN] Jetson モデル判定エラー: {e} → 強制的に JETSON_ORIN_NANO として続行")
    os.environ["JETSON_MODEL_NAME"] = "JETSON_ORIN_NANO"
    import Jetson.GPIO as GPIO
    BOARD_NAME = "JETSON_ORIN_NANO"

mode_descriptions = {
    "JETSON_NX":       ["15W_2CORE", "15W_4CORE", "15W_6CORE", "10W_2CORE", "10W_4CORE"],
    "JETSON_XAVIER":   ["MAXN", "MODE_10W", "MODE_15W", "MODE_30W"],
    "JETSON_NANO":     ["MAXN", "5W"],
    "JETSON_ORIN":     ["MAXN", "MODE_15W", "MODE_30W", "MODE_40W"],
    "JETSON_ORIN_NANO":["MODE_15W", "MODE_25W", "MODE_MAX"]
}

product_names = {
    "JETSON_NX":        "Jetson Xavier NX",
    "JETSON_XAVIER":    "Jetson AGX Xavier",
    "JETSON_NANO":      "Jetson Nano",
    "JETSON_ORIN":      "Jetson AGX Orin",
    "JETSON_ORIN_NANO": "Jetson Orin Nano"
}

board_settings = {
    "JETSON_NX":        (8, 3),
    "JETSON_XAVIER":    (8, 2),
    "JETSON_NANO":      (1, 0),
    "JETSON_ORIN":      (7, 0),
    "JETSON_ORIN_NANO": (7, 2)
}

i2c_busnum, power_mode = board_settings.get(BOARD_NAME, (None, None))
mode_list       = mode_descriptions.get(BOARD_NAME, [])
product_name    = product_names.get(BOARD_NAME, "未知のボード")

if i2c_busnum is not None and 0 <= power_mode < len(mode_list):
    mode_str = mode_list[power_mode]
    print("------------------------------------------------------------")
    print(f"{product_name} を認識: I2C バス番号 = {i2c_busnum}, Power モード = {mode_str} ({power_mode})")
    print("------------------------------------------------------------")
else:
    raise RuntimeError(f"未対応の Jetson モデル、または Power モード定義不足: {BOARD_NAME}")

FPS_60 = 1
FPS_30 = 2
if (product_name == "Jetson Orin Nano") or (product_name == "Jetson AGX Orin"):
    fps_type = FPS_60
else:
    fps_type = FPS_30

# ---------- PCA9685 (PWM) 初期化 ----------
BUSNUM = i2c_busnum
SERVO_HZ = 60
INITIAL_VALUE = 300
bus = smbus.SMBus(BUSNUM)
PCA9685 = Fabo_PCA9685.PCA9685(bus, INITIAL_VALUE, address=0x40)
PCA9685.set_hz(SERVO_HZ)

STEERING_CH = 0
THROTTLE_CH = 1

with open('pwm_params.json') as f:
    json_str = json.load(f)
    pwm_stop = json_str["pwm_speed"]["stop"]
    pwm_front = json_str["pwm_speed"]["front"]
    pwm_back = json_str["pwm_speed"]["back"]
    pwm_left_default = json_str["pwm_steering"]["left"] # 変数名を変更
    pwm_center = json_str["pwm_steering"]["center"]
    pwm_right_default = json_str["pwm_steering"]["right"] # 変数名を変更

direction = 0
REVERSE = 0
NORMAL = 1
if pwm_front >= pwm_back:
    direction = REVERSE
else:
    direction = NORMAL

PCA9685.set_channel_value(STEERING_CH, pwm_center)
PCA9685.set_channel_value(THROTTLE_CH, pwm_stop)

IMG_WIDTH = 224

# グローバル状態 (★ クラス変数に移行するために一部残しつつ調整)
process_no = 0
model_trt = None
model_loaded = False
# running, execute_thread, camera, save_dir はクラス変数へ移行/削除

def write_log(msg):
    global process_no
    process_no += 1
    print(f"{process_no}: {msg}")

def map_rc(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def handle(x, pwm_left, pwm_right):
    x = map_rc(x, 224, 0, pwm_right, pwm_left)
    PCA9685.set_channel_value(STEERING_CH, x)

def throttle(speed):
    speed = map_rc(speed, 224, 0, pwm_front, pwm_stop)
    PCA9685.set_channel_value(THROTTLE_CH, speed)

# ... (list_models, load_model_from_path, get_memory_usage は変更なし)

# def stop_camera(): # ★ 削除: カメラ直接操作の関数は不要
#     global camera
#     try:
#         camera.running = False
#         time.sleep(1)
#         camera.cap.release()
#         write_log("カメラを開放しました。")
#     except Exception as e:
#         write_log(f"カメラ開放時エラー: {e}")

# def live(...) : # ★ 削除: スレッド関数 live は不要、image_callbackに移行

class TripleDragonInference(Node): # ★ クラス名変更
    """
    自己位置エリアとカメラ画像を購読し、AI推論に基づきモーターを制御するノード
    """

    def __init__(self, args_data): # ★ 引数を受け取るように変更
        super().__init__('triple_dragon_inference')
        self.bridge = CvBridge()
        
        # ROS 2 Subscribers
        self.create_subscription(
            String,
            '/self_area/state',
            self.self_area_callback,
            10)        
        
        # ★ 追加: 画像トピックの購読
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1) # QoS depth 1 で最新フレームのみ処理

        # 状態変数
        self.latest_self_area = String(data="STATE_1") # デフォルト値を設定
        self.model_trt = None
        self.model_loaded = False
        self.running = False
        
        # パラメータ (元の run_main の設定を移植)
        self.args = args_data
        self.speed_mode = 'fixed' if self.args.speed_mode in ('fixed', '固定値') else 'infer'
        self.speed_raw_val = self.args.speed_raw
        self.speed_gain_val = self.args.speed_gain
        self.steering_gain_val = self.args.steering_gain
        self.pwm_left_val = self.args.pwm_left if self.args.pwm_left is not None else pwm_left_default
        self.pwm_right_val = self.args.pwm_right if self.args.pwm_right is not None else pwm_right_default

        self.record = self.args.record
        self.save_dir = None
        if self.record:
            if not self.args.save_name:
                write_log("録画する場合は --save-name を指定してください")
            else:
                self.save_dir = os.path.join('camera', self.args.save_name, 'xy')
                if not os.path.exists(self.save_dir):
                    os.makedirs(self.save_dir, exist_ok=True)
                write_log(f"録画データを {self.save_dir} に保存します")
        
        # FPS計測と録画用のカウンタ
        self.count = 1
        self.num = 1
        self.frame_count = 0
        self.start_time_local = time.time()
        self.total_process_time = 0

    def load_model(self, path):
        # グローバル関数をクラスメソッドに組み込み
        global model_trt, model_loaded
        p = Path(path)
        if not p.exists():
            self.get_logger().error(f"【Error】 {p} が見つかりません")
            return False
        try:
            self.get_logger().info(f"{p} をロード中… (初回は時間がかかります)")
            self.model_trt = TRTModule()
            state_dict = torch.load(p, weights_only=True)
            self.model_trt.load_state_dict(state_dict)
            self.model_loaded = True
            self.get_logger().info(f"{p} の読込に成功しました。")
            return True
        except Exception as e:
            self.get_logger().error(f"【Error】{p} の読込に失敗: {e}")
            self.model_loaded = False
            return False

    def self_area_callback(self, msg):
        """
        /self_area/state トピックからデータを受け取り、最新の値を保存する
        """
        self.latest_self_area = msg
        # self.get_logger().info(f"Received self_area: {self.latest_self_area.data}") # 頻繁な出力はデバッグ時のみ

    def image_callback(self, ros_image_msg):
        """
        /camera/image_raw トピックから画像を受け取り、推論・制御を実行する
        （元の live 関数のメインロジック）
        """
        if not self.model_loaded or not self.running:
            return

        # 1. ROS ImageをOpenCV画像 (numpy array) に変換
        try:
            image = self.bridge.imgmsg_to_cv2(ros_image_msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge conversion error: {e}")
            return
            
        area_num = self.latest_self_area.data
        # self.get_logger().info(f"[{time.strftime('%H:%M:%S')}] Current State (Callback): {area_num}")

        if self.record:
            remarked_img = image.copy()

        t_proc = time.time()
        
        # 2. 推論処理
        img = preprocess(image).half()
        out1, out2, out3 = self.model_trt(img)
        out1 = out1.detach().cpu().numpy().flatten()
        out2 = out2.detach().cpu().numpy().flatten()
        out3 = out3.detach().cpu().numpy().flatten()

        # 3. 推論結果に基づく制御値の計算
        if area_num == "STATE_1":
            x_raw, y, _, speed_raw = out1[0], out1[1], out1[2], out1[3]
        elif area_num == "STATE_2":
            x_raw, y, _, speed_raw = out2[0], out2[1], out2[2], out2[3]
        elif area_num == "STATE_3":
            x_raw, y, _, speed_raw = out3[0], out3[1], out3[2], out3[3]
        else:
            # 認識されていない場合やエラーの場合、停止
            PCA9685.set_channel_value(STEERING_CH, pwm_center)
            PCA9685.set_channel_value(THROTTLE_CH, pwm_stop)
            self.get_logger().warn(f"Unknown area state: {area_num}. Stopping vehicle.")
            return

        x = float(x_raw) * self.steering_gain_val
        x = int(IMG_WIDTH * (x / 2.0 + 0.5))
        
        speed = float(speed_raw)
        speed = int(IMG_WIDTH * (speed / 2.0 + 0.5) * self.speed_gain_val)

        # 4. PWM制御
        handle(x, self.pwm_left_val, self.pwm_right_val)

        if self.speed_mode == 'fixed':
            speed = self.speed_raw_val
        else:
            if speed > 224:
                speed = 224

        throttle(speed)
        total_process_time_instant = (time.time() - t_proc)
        self.total_process_time += total_process_time_instant

        # 5. ログ/録画処理 (元の live 関数のロジックを移植)
        if self.record and self.save_dir:
            # 画像ファイル名は連番で、エリア情報は含めない（元のコードを踏襲）
            if fps_type == FPS_30:
                name = "0_0_{:0=5}.jpg".format(self.count)
                image_path = os.path.join(self.save_dir, name)
                cv2.imwrite(image_path, remarked_img)
            else:
                if self.count % 2 == 0:
                    name = "0_0_{:0=5}.jpg".format(self.num)
                    image_path = os.path.join(self.save_dir, name)
                    cv2.imwrite(image_path, remarked_img)
                    self.num += 1
        
        self.count += 1
        self.frame_count += 1

        if time.time() - self.start_time_local > 3.0:
            # 3秒ごとのFPSログ表示
            fps = self.frame_count / 3.0
            speed_type = "(固定)" if self.speed_mode == 'fixed' else "(推論)"
            self.get_logger().info(
                f"推論処理: {self.total_process_time*1000/self.frame_count:.1f}ms, "
                f"Speed: {speed:.1f} {speed_type}, Steering Gain: {self.steering_gain_val}, "
                f"FPS: {fps:.1f} (3秒ごとに表示)"
            )
            self.frame_count = 0
            self.start_time_local = time.time()
            self.total_process_time = 0


    def on_shutdown(self):
        """
        ノード終了時に実行されるクリーンアップ処理
        """
        self.running = False
        self.get_logger().info("AI制御を停止しました。")
        # モーターを停止
        PCA9685.set_channel_value(STEERING_CH, pwm_center)
        PCA9685.set_channel_value(THROTTLE_CH, pwm_stop)
        
        if self.record:
            if fps_type == FPS_30:
                write_log(f"{fps_type}: 画像を{self.count}枚保存しました。")
            else:
                write_log(f"{fps_type}: 画像を{self.num}枚保存しました。")

def run_main(args):
    rclpy.init(args=None)
    node = TripleDragonInference(args) # ★ 修正

    if args.list_models:
        list_models()
        return

    if args.model is None:
        write_log("--model を指定してください。モデルを指定して実行できます。")
        return

    if not node.load_model(args.model): # ★ 修正
        return

    if args.run:
        node.running = True # ★ 修正: 推論開始フラグを立てる
        write_log("AI が起動しました。/camera/image_raw をサブスクライブして推論を開始します。Ctrl-C で停止します。")

        try:
            # rclpy.spin() でコールバック駆動のメインループを開始
            rclpy.spin(node) 
        except KeyboardInterrupt:
            write_log("Ctrl-C 受信、停止処理を開始します")
        finally:
            node.on_shutdown() # ★ 修正: クリーンアップ
            node.destroy_node()
            rclpy.shutdown()

    else:
        write_log("--run を指定しなかったためモデルを読み込んで終了します")


def parse_args():
    p = argparse.ArgumentParser(description='Jetson 推論実行 (ROS2 サブスクライバ版)')
    p.add_argument('--list-models', action='store_true', help='model_trt フォルダの .pth を一覧表示')
    p.add_argument('--model', type=str, help='読み込む TRT モデルのパス（.pth）')
    p.add_argument('--run', action='store_true', help='読み込んだモデルで推論を開始')
    p.add_argument('--record', action='store_true', help='走行映像を保存する')
    p.add_argument('--save-name', type=str, default='', help='録画時の保存名（camera/<name>/xy に保存）')
    p.add_argument('--speed-mode', type=str, default='fixed', choices=['fixed','infer','固定値','推論値'], help='速度を固定値にするか推論にするか')
    p.add_argument('--speed-raw', type=int, default=80, help='固定速度 (1-224)')
    p.add_argument('--speed-gain', type=float, default=1.0, help='推論からの速度乗算 gain')
    p.add_argument('--steering-gain', type=float, default=1.0, help='ステアリングのゲイン')
    p.add_argument('--pwm-left', type=int, default=None, help='ステアリング左 PWM 値 (省略で pwm_params.json の値)')
    p.add_argument('--pwm-right', type=int, default=None, help='ステアリング右 PWM 値 (省略で pwm_params.json の値)')
    return p.parse_args()

if __name__ == '__main__':
    args = parse_args()
    run_main(args)