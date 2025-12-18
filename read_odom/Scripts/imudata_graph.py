import re
import numpy as np
import matplotlib.pyplot as plt # グラフ描画ライブラリをインポート

# ros2 topic echo の出力から数値を抽出するための正規表現
stamp_sec_pattern = re.compile(r"sec:\s(\d+)")
stamp_nanosec_pattern = re.compile(r"nanosec:\s(\d+)")
val_pattern = re.compile(r"[xyz]:\s(-?\d+\.\d+)")

# データを格納するリスト
timestamps, acc_x_vals, acc_y_vals, acc_z_vals = [], [], [], []
gyro_x_vals, gyro_y_vals, gyro_z_vals = [], [], []

# どのセクションを読んでいるかのフラグ
is_in_accel = False
is_in_gyro = False
current_time = 0.0

try:
    with open('/home/ryosuke/ros2_log/imu_data.txt', 'r') as f:
        for line in f:
            # --- タイムスタンプの読み取り処理を追加 ---
            if 'sec:' in line:
                sec_match = stamp_sec_pattern.search(line)
                if sec_match:
                    current_time = float(sec_match.group(1))
            elif 'nanosec:' in line:
                nanosec_match = stamp_nanosec_pattern.search(line)
                if nanosec_match:
                    current_time += float(nanosec_match.group(1)) / 1e-9
            # ----------------------------------------

            elif 'linear_acceleration:' in line:
                is_in_accel = True
                is_in_gyro = False
                timestamps.append(current_time) # タイムスタンプを記録
                continue
            elif 'angular_velocity:' in line:
                is_in_accel = False
                is_in_gyro = True
                continue

            val_match = val_pattern.search(line)
            if not val_match:
                continue
            
            value = float(val_match.group(1))

            if is_in_accel:
                if 'x:' in line: acc_x_vals.append(value)
                elif 'y:' in line: acc_y_vals.append(value)
                elif 'z:' in line: acc_z_vals.append(value)
            elif is_in_gyro:
                if 'x:' in line: gyro_x_vals.append(value)
                elif 'y:' in line: gyro_y_vals.append(value)
                elif 'z:' in line: gyro_z_vals.append(value)

    # データの長さを揃える (メッセージが途中で切れている場合に対応)
    min_len = min(len(timestamps), len(acc_x_vals), len(gyro_x_vals))
    timestamps = np.array(timestamps[:min_len])
    acc_x_vals, acc_y_vals, acc_z_vals = acc_x_vals[:min_len], acc_y_vals[:min_len], acc_z_vals[:min_len]
    gyro_x_vals, gyro_y_vals, gyro_z_vals = gyro_x_vals[:min_len], gyro_y_vals[:min_len], gyro_z_vals[:min_len]

    # 最初の時刻を0にする（相対時間へ変換）
    timestamps -= timestamps[0]

    # --- ここからオフセット計算 (従来通り) ---
    avg_ax, avg_ay, avg_az = np.mean(acc_x_vals), np.mean(acc_y_vals), np.mean(acc_z_vals)
    avg_gx, avg_gy, avg_gz = np.mean(gyro_x_vals), np.mean(gyro_y_vals), np.mean(gyro_z_vals)
    
    print("--- Copy these lines into your M5Stack code ---")
    print("\n// OFFSETS CALCULATED FROM /imu TOPIC")
    print(f"const float ACC_X_OFFSET = {avg_ax / 9.8:.6f};")
    print(f"const float ACC_Y_OFFSET = {avg_ay / 9.8:.6f};")
    print(f"const float ACC_Z_OFFSET = {(avg_az / 9.8) - 1.0:.6f};")
    print(f"const float GYRO_X_OFFSET = {avg_gx / 0.0174533:.6f};")
    print(f"const float GYRO_Y_OFFSET = {avg_gy / 0.0174533:.6f};")
    print(f"const float GYRO_Z_OFFSET = {avg_gz / 0.0174533:.6f};")
    print("\nVisualizing data... Close the plot window to exit.")

    # --- ここからグラフ描画処理 ---
    fig, axs = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('IMU Sensor Data Over Time', fontsize=16)

    # 加速度
    axs[0, 0].plot(timestamps, acc_x_vals, label='Accel X')
    axs[0, 0].set_title('Linear Acceleration X')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('m/s^2')
    axs[0, 0].grid(True)

    axs[0, 1].plot(timestamps, acc_y_vals, label='Accel Y', color='g')
    axs[0, 1].set_title('Linear Acceleration Y')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].grid(True)

    axs[0, 2].plot(timestamps, acc_z_vals, label='Accel Z', color='r')
    axs[0, 2].set_title('Linear Acceleration Z')
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].grid(True)

    # 角速度
    axs[1, 0].plot(timestamps, gyro_x_vals, label='Gyro X')
    axs[1, 0].set_title('Angular Velocity X (Roll)')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('rad/s')
    axs[1, 0].grid(True)

    axs[1, 1].plot(timestamps, gyro_y_vals, label='Gyro Y', color='g')
    axs[1, 1].set_title('Angular Velocity Y (Pitch)')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].grid(True)

    axs[1, 2].plot(timestamps, gyro_z_vals, label='Gyro Z', color='r')
    axs[1, 2].set_title('Angular Velocity Z (Yaw)')
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()

except FileNotFoundError:
    print("Error: imu_data.txt not found. Make sure you run 'ros2 topic echo' first.")
except Exception as e:
    print(f"An error occurred: {e}")