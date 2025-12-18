import re
import csv

# --- 設定項目 ---
INPUT_FILENAME = '/home/ryosuke/ros2_log/imu_data_04.txt'
OUTPUT_FILENAME = '/home/ryosuke/ros2_log/imu_data_04.csv'
# ----------------

# 正規表現パターンを事前にコンパイル
# これらはros2 topic echoの出力形式に依存します
patterns = {
    'sec': re.compile(r"sec:\s*(\d+)"),
    'nanosec': re.compile(r"nanosec:\s*(\d+)"),
    'ax': re.compile(r"linear_acceleration:\s*\r?\n\s*x:\s*(-?\d+\.?\d*e?-?\d*)"),
    'ay': re.compile(r"y:\s*(-?\d+\.?\d*e?-?\d*)"),
    'az': re.compile(r"z:\s*(-?\d+\.?\d*e?-?\d*)"),
    'gx': re.compile(r"angular_velocity:\s*\r?\n\s*x:\s*(-?\d+\.?\d*e?-?\d*)"),
    'gy': re.compile(r"y:\s*(-?\d+\.?\d*e?-?\d*)"),
    'gz': re.compile(r"z:\s*(-?\d+\.?\d*e?-?\d*)"),
}

# CSVのヘッダー
header = [
    'timestamp', 
    'accel_x', 'accel_y', 'accel_z', 
    'gyro_x', 'gyro_y', 'gyro_z'
]

print(f"Processing '{INPUT_FILENAME}'...")

try:
    with open(INPUT_FILENAME, 'r') as infile, open(OUTPUT_FILENAME, 'w', newline='') as outfile:
        # CSVライターを作成し、ヘッダーを書き込む
        writer = csv.writer(outfile)
        writer.writerow(header)

        # テキストファイルをメッセージごとの塊（'---'で区切られる）に分割
        content = infile.read()
        message_chunks = content.split('---')

        processed_count = 0
        for chunk in message_chunks:
            if not chunk.strip():
                continue

            try:
                # 各パターンでデータを抽出
                sec = float(patterns['sec'].search(chunk).group(1))
                nanosec = float(patterns['nanosec'].search(chunk).group(1))
                
                # 正規表現を少し工夫して、accelとgyroの'x'などを区別
                accel_chunk = chunk.split('linear_acceleration:')[1].split('angular_velocity:')[0]
                gyro_chunk = chunk.split('angular_velocity:')[1]

                ax = float(patterns['ax'].search(chunk).group(1))
                ay = float(patterns['ay'].search(accel_chunk).group(1))
                az = float(patterns['az'].search(accel_chunk).group(1))
                
                gx = float(patterns['gx'].search(chunk).group(1))
                gy = float(patterns['gy'].search(gyro_chunk).group(1))
                gz = float(patterns['gz'].search(gyro_chunk).group(1))
                
                # タイムスタンプを秒単位に統一
                timestamp = sec + nanosec / 1e9
                
                # 1行分のデータを作成して書き込む
                row = [timestamp, ax, ay, az, gx, gy, gz]
                writer.writerow(row)
                processed_count += 1

            except (AttributeError, IndexError):
                # データが不正な形式の塊はスキップ
                # print(f"Skipping malformed chunk:\n{chunk}")
                continue
    
    print(f"Successfully processed {processed_count} messages.")
    print(f"Data has been saved to '{OUTPUT_FILENAME}'")

except FileNotFoundError:
    print(f"Error: Input file '{INPUT_FILENAME}' not found.")
except Exception as e:
    print(f"An error occurred: {e}")