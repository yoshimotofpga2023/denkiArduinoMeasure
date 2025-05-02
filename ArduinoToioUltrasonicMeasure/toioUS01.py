import serial
import csv
from datetime import datetime
import asyncio
from toio import *
import math
import configparser
import os
import matplotlib.pyplot as plt
import numpy as np

# config.ini を読み込み
config = configparser.ConfigParser()
config.read(os.path.join(os.path.dirname(__file__), 'config.ini'), encoding='utf-8')
serial_port = config.get('Serial', 'port')
baudrate = config.getint('Serial', 'baudrate')

ser = serial.Serial(serial_port, baudrate, timeout=2)
csv_file = os.path.join(os.path.dirname(__file__), 'distance_log.csv')
previous_time = datetime.now()
previous_position = None

with open(csv_file, mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Elapsed_sec", "Ultrasonic_cm", "Toio_moved_mm", "Duration_us", "Distance_m", "Elapsed_s", "Cumulative_m"])

async def move_toio_in_steps():
    global previous_time, previous_position
    dev_list = await BLEScanner.scan(num=1)
    assert len(dev_list)

    cube = ToioCoreCube(dev_list[0].interface)
    await cube.connect()

    x_start = 170
    y = 180
    steps = 10
    step_size = 15
    delay_sec = 3

    cumulative_m = 0.0

    try:
        for i in range(steps):
            x_target = x_start + i * step_size

            await cube.api.motor.motor_control(10, 10)
            await asyncio.sleep(1)
            await cube.api.motor.motor_control(0, 0)
            toiopos = await cube.api.id_information.read()

            x = toiopos.sensor.point.x
            y = toiopos.sensor.point.y             

            current_position = (x, y)
            moved_mm = 0.0
            if previous_position is not None:
                dx = current_position[0] - previous_position[0]
                dy = current_position[1] - previous_position[1]
                moved_mm = math.sqrt(dx * dx + dy * dy)
            previous_position = current_position

            ser.reset_input_buffer()
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            parts = line.split(',')
            if len(parts) < 3:
                continue
            distance, duration, button_state = parts
            try:
                distance = float(distance)
                duration = float(duration)
                button_state = int(button_state)
            except ValueError:
                continue

            distance_m = distance * 0.01
            duration_s = duration * 0.000001
            cumulative_m += distance_m

            current_time = datetime.now()
            elapsed_sec = (current_time - previous_time).total_seconds()
            previous_time = current_time
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

            with open(csv_file, mode='a', newline='', encoding='utf-8') as file:
                writer = csv.writer(file)
                writer.writerow([
                    timestamp,
                    elapsed_sec,
                    distance,
                    moved_mm + 5.0,
                    duration,
                    distance_m,
                    duration_s,
                    cumulative_m
                ])

            print(f"{timestamp} - 距離: {distance:.2f} cm, 時間: {duration:.2f} us, toio移動: {moved_mm + 5.0:.1f} mm")

            await asyncio.sleep(delay_sec)
    finally:
        await cube.disconnect()
        ser.close()

    # --- CSV読み込み → 回帰直線描画 ---
    times = []
    distances = []

    with open(csv_file, newline='', encoding='utf-8') as f:
        reader = csv.reader(f)
        next(reader)  # ヘッダー
        for i, row in enumerate(reader):
            if i == 0:
                continue  # 最初のデータ行は除く
            if i >= 10:
                break
            try:
                times.append(float(row[6]))  # Elapsed_s
                distances.append(float(row[7]))  # Cumulative_m
            except ValueError:
                continue

    times_cumsum = np.cumsum(times)

    if len(times_cumsum) >= 2 and np.std(times_cumsum) > 0:
        slope, intercept = np.polyfit(times_cumsum, distances, 1)
        regression_line = slope * times_cumsum + intercept

        plt.plot(times_cumsum, distances, 'o', label='Data')
        plt.plot(times_cumsum, regression_line, '-', label=f'y = {slope:.4f}x + {intercept:.4f}')
        plt.xlabel('Time (s)')
        plt.ylabel('Cumulative Distance (m)')
        plt.title('Cumulative Distance vs Time (excluding 1st point)')
        plt.legend()
        plt.grid(True)
        plt.show()
    else:
        print("⚠ 十分なデータがなく回帰線を描画できません。")

if __name__ == "__main__":
    try:
        asyncio.run(move_toio_in_steps())
    except KeyboardInterrupt:
        ser.close()
