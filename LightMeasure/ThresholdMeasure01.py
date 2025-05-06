# -*- coding: shift_jis -*-
import serial
import matplotlib.pyplot as plt
import numpy as np
import keyboard  # pip install keyboard

plt.rcParams['font.family'] = 'MS Gothic'

# シリアルポートの設定（必要に応じて 'COM3' を変更）
# ser = serial.Serial('COM21', 9600, timeout=1)
ser = serial.Serial('/dev/cu.usbmodem14101', 9600, timeout=1)

print("スペースキーを押すと測定を開始します...")
keyboard.wait("space")

# 測定開始指令送信
ser.write(b'S')

# 測定データ受信（500回分）
values = []
while len(values) < 500:
    line = ser.readline().decode('shift_jis').strip()
    if line.isdigit():
        values.append(int(line))

# データ処理
x = np.arange(len(values))
y = np.array(values)

# 平均値と判定ライン
avg = np.mean(y)
avg_line = np.ones_like(x) * avg
threshold = 600  # 白黒判定閾値
judged_value = 900 if avg > threshold else 100

# グラフ表示
plt.figure(figsize=(10, 5))
plt.scatter(x, y, label='反射光の測定値', s=10)
plt.plot(x, avg_line, 'r--', label=f'平均値 = {avg:.1f}')
plt.plot(x, [judged_value]*len(x), 'g-', label='判定値（白:900 / 黒:100）')
plt.xlabel('測定回数')
plt.ylabel('反射光の強さ')
plt.title('LED反射光の測定結果（500回）')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
