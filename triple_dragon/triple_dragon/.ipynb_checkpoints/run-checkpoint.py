import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor # 重い処理のためにマルチスレッド化
from nav_msgs.msg import Odometry
from std_msgs.msg import String
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
from jetcam.csi_camera import CSICamera
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
    pwm_left = json_str["pwm_steering"]["left"]
    pwm_center = json_str["pwm_steering"]["center"]
    pwm_right = json_str["pwm_steering"]["right"]

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

# グローバル状態
process_no = 0
model_trt = None
running = False
record = False
save_dir = None
execute_thread = None
start_time = None
count = 0
model_loaded = False

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

def list_models():
    files = glob.glob('./model_trt/*.pth', recursive=True)
    if not files:
        print('model_trt ディレクトリに .pth ファイルが見つかりません')
        return
    for f in files:
        ts = os.path.getctime(f)
        d = datetime.datetime.fromtimestamp(ts)
        s = d.strftime('%Y-%m-%d %H:%M:%S')
        print(f"{f}    (created: {s})")

def load_model_from_path(path):
    global model_trt, model_loaded
    p = Path(path)
    if not p.exists():
        write_log(f"【Error】 {p} が見つかりません")
        return False
    try:
        write_log(f"{p} をロード中… (初回は時間がかかります)")
        model_trt = TRTModule()
        state_dict = torch.load(p, weights_only=True)
        model_trt.load_state_dict(state_dict)
        model_loaded = True
        write_log(f"{p} の読込に成功しました。")
        return True
    except Exception as e:
        write_log(f"【Error】{p} の読込に失敗: {e}")
        model_loaded = False
        return False

def get_memory_usage():
    command = 'tegrastats'
    try:
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        # 簡易的に最初の数行を探す
        for _ in range(10):
            line = process.stdout.readline()
            if not line:
                break
            if 'RAM' in line:
                import re
                m = re.search(r'RAM (\d+)/(\d+)MB', line)
                if m:
                    used = int(m.group(1))
                    total = int(m.group(2))
                    process.kill()
                    return used, total
        process.kill()
    except Exception:
        return None, None
    return None, None

def stop_camera():
    global camera
    try:
        camera.running = False
        time.sleep(1)
        camera.cap.release()
        write_log("カメラを開放しました。")
    except Exception as e:
        write_log(f"カメラ開放時エラー: {e}")

def live(speed_mode, speed_raw_val, speed_gain_val, steering_gain_val, pwm_left_val, pwm_right_val, node):
    global running, camera, record, save_dir, count
    count = 1
    num = 1
    frame_count = 0
    start_time_local = time.time()
    total_process_time = 0
    while running:
        # area_num = node.latest_self_area.data
        area_num = "STATE_2"
        print(f"[{time.strftime('%H:%M:%S')}] Current State (External Read): {area_num}")

        image = camera.read()
        if record:
            remarked_img = image.copy()

        t_proc = time.time()
        img = preprocess(image).half()
        out = model_trt(img).detach().cpu().numpy().flatten()
        #out1 = out1.detach().cpu().numpy().flatten()
        #out2 = out2.detach().cpu().numpy().flatten()
        #out3 = out3.detach().cpu().numpy().flatten()
        # out1 = model_trt(img).detach().cpu().numpy().flatten()

        # x = float(out1[0]) * steering_gain_val
        # y = float(out1[1])
        # x = int(IMG_WIDTH * (x / 2.0 + 0.5))
        # y = int(IMG_WIDTH * (y / 2.0 + 0.5))
        # speed = float(out1[3])

        if area_num == "STATE_1":
            x = float(out[0]) * steering_gain_val
            y = float(out[1])
            x = int(IMG_WIDTH * (x / 2.0 + 0.5))
            y = int(IMG_WIDTH * (y / 2.0 + 0.5))
            speed = float(out[3])
        elif area_num == "STATE_2":
            x = float(out[16]) * steering_gain_val
            y = float(out[17])
            x = int(IMG_WIDTH * (x / 2.0 + 0.5))
            y = int(IMG_WIDTH * (y / 2.0 + 0.5))
            speed = float(out[6])
        elif area_num == "STATE_4":
            x = float(out[20]) * steering_gain_val
            y = float(out[21])
            x = int(IMG_WIDTH * (x / 2.0 + 0.5))
            y = int(IMG_WIDTH * (y / 2.0 + 0.5))
            speed = float(out[22])
        
        handle(x, pwm_left_val, pwm_right_val)

        speed = int(IMG_WIDTH * (speed / 2.0 + 0.5)) * speed_gain_val
        if speed_mode == 'fixed':
            speed = speed_raw_val
        else:
            if speed > 224:
                speed = 224

        throttle(speed)
        total_process_time += (time.time() - t_proc)

        if record:
            if fps_type == FPS_30:
                name = "0_0_{:0=5}.jpg".format(count)
                image_path = os.path.join(save_dir, name)
                cv2.imwrite(image_path, remarked_img)
            else:
                if count % 2 == 0:
                    name = "0_0_{:0=5}.jpg".format(num)
                    image_path = os.path.join(save_dir, name)
                    cv2.imwrite(image_path, remarked_img)
                    num += 1
        count += 1
        frame_count += 1

        if time.time() - start_time_local > 3.0:
            fps = frame_count / 3.0
            speed_type = "(固定)" if speed_mode == 'fixed' else "(推論)"
            write_log(f"推論処理: {total_process_time*1000/frame_count:.1f}ms, Speed: {speed:.1f} {speed_type}, Steering Gain: {steering_gain_val}, FPS: {fps:.1f} (3秒ごとに表示)")
            frame_count = 0
            start_time_local = time.time()
            total_process_time = 0

    if record:
        if fps_type == FPS_30:
            write_log(f"{fps_type}: 画像を{count}枚保存しました。")
        else:
            write_log(f"{fps_type}: 画像を{num}枚保存しました。")



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
        # print(self.latest_self_area)
        
    def on_shutdown(self):
        """
        ノード終了時に実行されるクリーンアップ処理
        """
def run_main(args):
    # ros関係
    rclpy.init(args=None)
    node = SelfAreaSubscribe()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    global camera, running, execute_thread, save_dir

    # カメラ初期化
    if (product_name == "Jetson Orin Nano") or (product_name == "Jetson AGX Orin"):
        camera = CSICamera(capture_device=1, width=224, height=224, capture_fps=60)
    else:
        camera = CSICamera(capture_device=1, width=224, height=224, capture_fps=30)

    if args.list_models:
        list_models()
        return

    if args.model is None:
        write_log("--model を指定してください。モデルを指定して実行できます。")
        return

    if not load_model_from_path(args.model):
        return

    # 設定値
    speed_mode = 'fixed' if args.speed_mode in ('fixed', '固定値') else 'infer'
    speed_raw_val = args.speed_raw
    speed_gain_val = args.speed_gain
    steering_gain_val = args.steering_gain
    pwm_left_val = args.pwm_left if args.pwm_left is not None else pwm_left
    pwm_right_val = args.pwm_right if args.pwm_right is not None else pwm_right

    if args.run:
        if args.record:
            if not args.save_name:
                write_log("録画する場合は --save-name を指定してください")
                return
            save_dir = os.path.join('camera', args.save_name, 'xy')
            if not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
            write_log(f"録画データを {save_dir} に保存します")

        running = True
        execute_thread = threading.Thread(target=live, args=(speed_mode, speed_raw_val, speed_gain_val, steering_gain_val, pwm_left_val, pwm_right_val, node))
        execute_thread.start()
        write_log("AI が起動しました。Ctrl-C で停止します。")

        try:
            while execute_thread.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            write_log("Ctrl-C 受信、停止処理を開始します")
            running = False
            execute_thread.join()
            stop_camera()
            node.on_shutdown()
            node.destroy_node()
            rclpy.shutdown()


    else:
        write_log("--run を指定しなかったためモデルを読み込んで終了します")


def parse_args():
    p = argparse.ArgumentParser(description='Jetson 推論実行 (非 ipywidgets 版)')
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

# def main(args=None):
#     rclpy.init(args=args)
#     node = SelfAreaSubscribe()
#     try:
#         node.get_logger().info('Starting spin_once loop to read data externally.')
        
#         # rclpy.spin_once を使用して、イベントが発生した場合にのみ処理を行うループ
#         while 1:
#             # 1. rclpy.spin_once() で、保留中のコールバック（メッセージ受信など）を処理する
#             rclpy.spin_once(node, timeout_sec=0.1) 
            
#             current_state = node.latest_self_area.data
            
#             print(f"[{time.strftime('%H:%M:%S')}] Current State (External Read): {current_state}")
            
#             # 読み取り頻度を調整するため、適度に待機
#             time.sleep(1.0) 
            
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # 終了処理と安全なシャットダウン
#         node.on_shutdown()
#         node.destroy_node()
#         rclpy.shutdown()
if __name__ == '__main__':
    args = parse_args()
    # try:
    run_main(args)
    # except Exception as e:
    #     write_log(f"致命的エラー: {e}")
