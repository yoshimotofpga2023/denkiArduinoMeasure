import serial
import csv
import matplotlib.pyplot as plt
import numpy as np

# ser = serial.Serial('COM3', 9600, timeout=1)
ser = serial.Serial('/dev/cu.usbmodem14101', 9600, timeout=1)
csv_filename = 'motor_pwm_measurements.csv'

data = []

with open(csv_filename, 'w', newline='', encoding='utf-8') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['PWM', 'Time_sec'])

    for _ in range(4):
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print("recieve:", line)
                try:
                    pwm_str, sec_str = line.split(',')
                    pwm = int(pwm_str)
                    sec = float(sec_str)
                    writer.writerow([pwm, sec])
                    data.append((pwm, sec))
                    break
                except ValueError:
                    print("error")
                    continue

pwm_vals = [d[0] for d in data]
sec_vals = [d[1] for d in data]
coeff = np.polyfit(pwm_vals, sec_vals, 1)
fit_line = np.poly1d(coeff)

plt.figure()
plt.plot(pwm_vals, sec_vals, 'o', label='data')
plt.plot(pwm_vals, fit_line(pwm_vals), '-', label=f'line(coeff={coeff[0]:.3f})')
plt.xlabel('PWM')
plt.ylabel('time')
plt.title('pwm-time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('motor_pwm_plot.png')
plt.show()
