import re
import csv
import numpy as np

# --- 設定項目 ---
INPUT_FILENAME = '/home/ryosuke/ros2_log/imu_data.txt'
OUTPUT_FILENAME = '/home/ryosuke/ros2_log/imu_data.csv'
# ----------------

# 正規表現パターン
patterns = {
    'sec': re.compile(r"sec:\s*(\d+)"),
    'nanosec': re.compile(r"nanosec:\s*(\d+)"),
    'val': re.compile(r"[xyz]:\s*(-?\d+\.?\d*e?-?\d*)"),
}

# CSVのヘッダー
header = [
    'timestamp', 
    'accel_x', 'accel_y', 'accel_z', 
    'gyro_x', 'gyro_y', 'gyro_z'
]

# 全データを一時的に保存するリスト
all_rows_for_csv = []
acc_x_vals, acc_y_vals, acc_z_vals = [], [], []
gyro_x_vals, gyro_y_vals, gyro_z_vals = [], [], []

print(f"Processing '{INPUT_FILENAME}'...")

try:
    with open(INPUT_FILENAME, 'r') as infile:
        content = infile.read()
        # テキスト全体をメッセージごとの塊に分割
        message_chunks = content.split('---')

        for chunk in message_chunks:
            if not chunk.strip():
                continue

            try:
                # タイムスタンプを抽出
                sec = float(patterns['sec'].search(chunk).group(1))
                nanosec = float(patterns['nanosec'].search(chunk).group(1))
                timestamp = sec + nanosec / 1e9

                # --- 修正箇所：順序に依存しないように修正 ---
                # 正規表現で各ブロックを直接探しに行く
                accel_match = re.search(r"linear_acceleration:(.*?)(?=\w+:)", chunk, re.DOTALL)
                gyro_match = re.search(r"angular_velocity:(.*?)(?=\w+:)", chunk, re.DOTALL)

                if accel_match and gyro_match:
                    accel_chunk = accel_match.group(1)
                    gyro_chunk = gyro_match.group(1)
                    
                    acc_vals = patterns['val'].findall(accel_chunk)
                    gyro_vals = patterns['val'].findall(gyro_chunk)

                    if len(acc_vals) == 3 and len(gyro_vals) == 3:
                        ax, ay, az = float(acc_vals[0]), float(acc_vals[1]), float(acc_vals[2])
                        gx, gy, gz = float(gyro_vals[0]), float(gyro_vals[1]), float(gyro_vals[2])
                        
                        # データを保存
                        all_rows_for_csv.append([timestamp, ax, ay, az, gx, gy, gz])
                        acc_x_vals.append(ax)
                        acc_y_vals.append(ay)
                        acc_z_vals.append(az)
                        gyro_x_vals.append(gx)
                        gyro_y_vals.append(gy)
                        gyro_z_vals.append(gz)

            except (AttributeError, IndexError):
                continue

    if not all_rows_for_csv:
        raise ValueError("No valid IMU messages were parsed.")

    # CSVファイルへの出力
    with open(OUTPUT_FILENAME, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(header)
        writer.writerows(all_rows_for_csv)
    print(f"\nSuccessfully processed {len(all_rows_for_csv)} messages.")
    print(f"Data has been saved to '{OUTPUT_FILENAME}'")

    # 平均値（バイアス）を計算
    avg_ax, avg_ay, avg_az = np.mean(acc_x_vals), np.mean(acc_y_vals), np.mean(acc_z_vals)
    avg_gx, avg_gy, avg_gz = np.mean(gyro_x_vals), np.mean(gyro_y_vals), np.mean(gyro_z_vals)
    
    # C++コード形式でオフセットを出力（単位変換なし）
    print("\n--- Copy these lines into your M5Stack code ---")
    print("\n// OFFSETS in [m/s^2] and [rad/s]")
    print(f"const float ACC_X_OFFSET = {avg_ax:.6f};")
    print(f"const float ACC_Y_OFFSET = {avg_ay:.6f};")
    print(f"const float ACC_Z_OFFSET = {avg_az:.6f};") # Z軸は重力からのズレ
    print(f"const float GYRO_X_OFFSET = {avg_gx:.6f};")
    print(f"const float GYRO_Y_OFFSET = {avg_gy:.6f};")
    print(f"const float GYRO_Z_OFFSET = {avg_gz:.6f};")

except FileNotFoundError:
    print(f"Error: Input file '{INPUT_FILENAME}' not found.")
# except Exception as e:
#     print(f"An error occurred: {e}")