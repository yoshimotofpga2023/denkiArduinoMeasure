# �����g�Z���T�[�ɂ�鉹������v���O����

## �T�v
Arduino�X�^�[�^�[�L�b�g�ɕt������Ă��钴���g�Z���T�[����擾�ł���l�i���g�̒��˕Ԃ莞�ԁj���特�����v������D
�v���O�����ɂ��c���ɋ����A�����Ɏ��Ԃ��v���b�g���A�ŏ����@�iExcel�Amatlab, python�����p���āj���璼�����v���b�g���ߎ������𓾂�D
�����̌X������340m/s�i�����j�ƂȂ��Ă��邩�m�F����D


## ���e
- �����g�Z���T�[�ɐڑ������Arduino����AArduino�d���������ォ����̊Ԋu�ŃV���A���ʐM�Œ����g�Z���T�[�̉��g���˕Ԃ莞�ԁius�j��
PC�֑��M����D

- PC�ł�Python�ɂ��v���O���~���O���s���D
Python��Arduino���瑗����V���A���ʐM�l����M����D
�����ɁAtoio�Ƃ������{�b�g�E�v���O���~���O���ރ��{�b�g��Python�o�R�Ő��䂷��D
toio�͐�p�}�b�g�ɂ�萳�m�Ȉړ����������߂邱�Ƃ��ł���D�iA3�T�C�Y���x�̋������ړ��\�j
��Pyinstller�Ƃ���Python���C�u������exe���\�i���Ԃ�A�w����Python�̎��s���𐮂���K�v�Ȃ��j

- Python�ŏ�L�AArduino����̉��g���˕Ԃ莞�ԁius��s�֕ϊ��j�Ƃ���ɓ������Ĉړ�����toio�̈ړ�����(cm��m)��
CSV�t�@�C���ɕۑ�����D

- ����CSV�t�@�C����t����Excel�̂悤�ɍŏ�����@�ŋߎ��Ȑ��̃v���b�g�ƌX�������߂邱�Ƃɗ��p�ł���D
- �܂��Apython�v���O�����ł͌v���Ɠ����ɑ��肵���l����ŏ�����@�𗘗p���Ē��ڋߎ��������v���b�g�����悤�ɂ��Ă���D

### �����X�P�W���[��

- 1��̎����i4����z��j
- 1, 2���F�����P�̃`���[�g���A���I�ȃV���A���ʐM�̃v���O�������쐬�����x���v���ł���Ƃ���܂ōs���D
�܂��A3,4���Œ񎦂���ۑ�܂ł��T������D
- 3, 4���F�����Q���s�������g�Z���T�[�ɂ�艹�����ŏ����@�̌X�����狁�߂�D

## �����P
### ���x�Z���T�[��p����PC�ƃV���A���ʐM�v���O�����̍쐬

- �V���A���ʐM���̎d�l��񎦂��A����ɑ�����Arduino�̃v���O�������쐬����D
- ��{�I�ɂ�Arduino�̃v���O������񎦂��Ă����i�v���O�����͌����ߌ`���A�V���A���ʐM��A���x�Z���T�[����擾�����f�W�^���l�����x�ɕϊ�����Ƃ���Ȃǁj

#### �d�l�ƃv���O����

- �V���A���ʐM��M�p�̃v���O����(python��GUI)
- �������DGUI��p���Ȃ�Python�v���O����

```
import serial
import time

# �V���A���|�[�g�ƃ{�[���[�g��ݒ�
ser = serial.Serial('COM3', 9600, timeout=1)  # COM�|�[�g�͓K�X�ύX�i��F'COM3'��'/dev/ttyACM0'�j

time.sleep(2)  # Arduino�����Z�b�g����̂�҂�

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(line)
except KeyboardInterrupt:
    print("�I�����܂��B")
finally:
    ser.close()

```

- Arduino�̃v���O����

```
#define SENSOR_PIN 0

void setup() {
  Serial.begin(9600);
}

void loop() {

  float temp, volt;
  int value;

  value = analogRead(SENSOR_PIN);

  volt = (float) value * 5.0 / 1023.0;
  temp = volt * 100 - 60;

  Serial.print("Temperature:  ");
  Serial.println( temp );

  
  delay(500);  // 0.5�b���Ƃɑ���
}
```

## ����2.1
### �����g�Z���T�[��toio�𗘗p������������v���O����

- �V���A���ʐM���̎d�l�݂̂�񎦂��A����ɑ�����Arduino�̃v���O�������쐬����D
- �d�l�ɉ�����Arduino���̃v���O�������w�����g���쐬����D
- Python����GUI�\�t�g�̔z�z�ƁA�g�����Atoio�Ƃ̘A�g�菇���͕ʓr��������D
- Arduino�����瑗�M���ꂽ�����g�Z���T�[�̉��g���˕Ԃ莞�Ԃ�toio�̗ݐψړ�������PC��CSV�t�@�C���`���ŏo�͂����D
�w����PC��Excel(���邢�́Amatlab?)�𗘗p����CSV�f�[�^���C���|�[�g���A�ŏ����@�̎����i�X���A�ؕЂ̓��o�j�A�O���t�̍쐬���s���D

## ����2.2
### ����_���ɂ��\���c���̓x�����z�쐬

- �ŏ����@�ŗp����\���c���́A���K���z�����肵�Ă���D
- ���̂��߁A�x�����z���쐬���邱�ƂŐ��K���z�̌`�ɂȂ��Ă���͂��ł��D
- �܂��A����_���������Ȃ邱�Ƃő吔�̖@���ɂ��A���K���z�A����ɕ���0, �W���΍�1�̕W�����K���z�̌`�ɋ߂Â��H

- ��L�m�F�̂��߂ɁA�ŏ��ɑ��肵��10�_�̃f�[�^�����Ƃɓx�����z���쐬����D
- ����ɁA100�_�v�����A���l�̏����������Ȃ��D
- ���ꂼ��ɐ��K���z�̃v���b�g���d�˂čl�@�ǂ���ɂȂ邩�m�F����D
- �]�͂������50�_������s���D


#### �d�l�ƃv���O����

- �V���A���ʐM��M�p�̃v���O����(python��GUI)
- GUI��ʗ�

-- �N�����

--- �v���{�^���������O�ɃR���{�{�b�N�X�ő���񐔂�I������D
--- �I���񐔂ɉ����āAtoio�̈ړ������A���x�𐧌�D�}�b�g�iA3�^�e�j��3�����炢�Ȃ��đ��肷��D

![GUI03](./pic/gui_100.png "GUI03")



-- �v�����ʁi����_�I���R���{�{�b�N�X�Ȃ��j

![GUI02](./pic/gui02.png "GUI02")

- Python�v���O����

```
import serial
import csv
import os
import math
import asyncio
import configparser
from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
from toio import *
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading

# �����ݒ�
print("�ݒ�t�@�C���ǂݍ��ݒ�...")
dir_path = os.path.dirname(__file__)
config = configparser.ConfigParser()
config.read(os.path.join(dir_path, 'config.ini'), encoding='utf-8')
serial_port = config.get('Serial', 'port')
baudrate = config.getint('Serial', 'baudrate')
csv_file = os.path.join(dir_path, 'distance_log.csv')
print(f"�g�p�V���A���|�[�g: {serial_port}, �{�[���[�g: {baudrate}")

class ToioMeasurementApp:
    def __init__(self, root):
        print("GUI��������...")
        self.root = root
        self.root.title("Toio�����v���A�v��")

        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        control_frame = tk.Frame(self.root)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)

        self.start_button = tk.Button(control_frame, text="�v���J�n", command=self.start_measurement)
        self.start_button.pack(pady=10)

        self.quit_button = tk.Button(control_frame, text="�I��", command=self.quit_application)
        self.quit_button.pack(pady=10)

        self.tree = ttk.Treeview(control_frame, columns=("#1", "#2", "#3"), show="headings")
        self.tree.heading("#1", text="��")
        self.tree.heading("#2", text="���莞��[us]")
        self.tree.heading("#3", text="�ݐϋ���[m]")
        self.tree.pack(fill=tk.BOTH, expand=True)

        self.ser = serial.Serial(serial_port, baudrate, timeout=2)
        print("�V���A���|�[�g���I�[�v�����܂���")
        self.previous_time = datetime.now()
        self.previous_position = None

    def start_measurement(self):
        print("�v���X���b�h�J�n")
        thread = threading.Thread(target=self.run_asyncio_loop)
        thread.start()

    def run_asyncio_loop(self):
        print("�񓯊����[�v�J�n")
        asyncio.run(self.move_toio_in_steps())

    def quit_application(self):
        print("�A�v���P�[�V�����I�������J�n")
        try:
            if self.ser.is_open:
                print("�V���A���|�[�g����܂�")
                self.ser.close()
        except Exception as e:
            print(f"�V���A���|�[�g�N���[�Y���G���[: {e}")
        self.root.quit()

    async def move_toio_in_steps(self):
        print("toio�X�L�����J�n")
        dev_list = await BLEScanner.scan(num=1)
        assert len(dev_list)

        print("toio�ڑ���...")
        cube = ToioCoreCube(dev_list[0].interface)
        await cube.connect()
        print("toio�ڑ�����")

        x_start = 170
        y = 180
        steps = 10
        step_size = 15
        delay_sec = 3

        cumulative_m = 0.0
        distances = []
        times = []

        print("CSV�t�@�C���쐬��...")
        with open(csv_file, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(["Timestamp", "Elapsed_sec", "Ultrasonic_cm", "Toio_moved_mm", "Duration_us", "Distance_m", "Elapsed_s", "Cumulative_m"])

        try:
            for i in range(steps):
                print(f"�X�e�b�v {i+1}/{steps}: ���[�^�[����J�n")
                if steps < 30:
                    await cube.api.motor.motor_control(10, 10)
                elif steps >= 30:
                    await cube.api.motor.motor_control(30, 30)

                await asyncio.sleep(10/steps)
                # await asyncio.sleep(1)

                await cube.api.motor.motor_control(0, 0)

                print("���[�^�[��~�A�ʒu�擾��...")
                toiopos = await cube.api.id_information.read()

                x = toiopos.sensor.point.x
                y = toiopos.sensor.point.y
                print(f"���݈ʒu: x={x}, y={y}")

                current_position = (x, y)
                moved_mm = 0.0
                if self.previous_position is not None:
                    dx = current_position[0] - self.previous_position[0]
                    dy = current_position[1] - self.previous_position[1]
                    moved_mm = math.sqrt(dx * dx + dy * dy)
                    print(f"�ړ�����: {moved_mm:.2f} mm")
                self.previous_position = current_position

                self.ser.reset_input_buffer()
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"�V���A����M: {line}")
                parts = line.split(',')
                if len(parts) < 3:
                    print("�f�[�^�`���s���A�X�L�b�v")
                    continue
                distance, duration, button_state = parts
                try:
                    distance = float(distance)
                    duration = float(duration)
                    button_state = int(button_state)
                except ValueError:
                    print("���l�ϊ����s�A�X�L�b�v")
                    continue

                distance_m = distance * 0.01
                duration_s = duration * 0.000001

                if i == 0:
                    cumulative_m = distance_m
                else:
                    cumulative_m += (moved_mm + 5.0) * 0.001

                current_time = datetime.now()
                elapsed_sec = (current_time - self.previous_time).total_seconds()
                self.previous_time = current_time

                with open(csv_file, mode='a', newline='', encoding='utf-8') as file:
                    writer = csv.writer(file)
                    writer.writerow([
                        current_time.strftime("%Y-%m-%d %H:%M:%S"),
                        elapsed_sec,
                        distance,
                        moved_mm + 5.0,
                        duration / 2,
                        distance_m,
                        duration_s,
                        cumulative_m
                    ])

                times.append(duration_s)
                distances.append(cumulative_m)

                self.tree.insert("", "end", values=(i + 1, f"{duration:.2f}", f"{cumulative_m:.3f}"))

                await asyncio.sleep(delay_sec)
        finally:
            print("toio�ؒf�ƃV���A���N���[�Y")
            await cube.disconnect()
            if self.ser.is_open:
                self.ser.close()

        if len(times) >= 2:
            print("�O���t�`�揈���J�n")
            times_np = np.array(times)
            distances_np = np.array(distances)
            slope, intercept = np.polyfit(times_np, distances_np, 1)
            regression_line = slope * times_np + intercept

            self.ax.clear()
            self.ax.plot(times_np, distances_np, 'o', label='Data')
            self.ax.plot(times_np, regression_line, '-', label=f'y = {slope:.4f}x + {intercept:.4f}')
            self.ax.set_xlabel('Time (s)')
            self.ax.set_ylabel('Cumulative Distance (m)')
            self.ax.set_title('Cumulative Distance vs Time')
            self.ax.legend()
            self.ax.grid(True)
            self.canvas.draw()
            print("�O���t�`�抮��")

if __name__ == "__main__":
    print("�A�v���N��")
    root = tk.Tk()
    app = ToioMeasurementApp(root)
    root.mainloop()

```

- Arduino�̃v���O����

```
#define TRIG_PIN 9
#define ECHO_PIN 2
#define BUTTON_PIN 3

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  // �����g�p���X�𑗐M
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // ���˕Ԃ�̎��Ԃ��擾�i�P�ʁF�}�C�N���b�j
  long duration = pulseIn(ECHO_PIN, HIGH);

  // �������Z�o�icm�j
  float distance = duration * 0.034 / 2;
  int buttonState = digitalRead(BUTTON_PIN);
  // ���ʂ�\��
  Serial.print(distance);
  Serial.print(",");
  Serial.print(duration);
  Serial.print(",");
  Serial.println(buttonState);

  delay(500);  // 0.5�b���Ƃɑ���
}
```

- Excel�ɂ��ŏ����������

-- �f�[�^�W�v�\

![ex01](./pic/ex01.png "ex01")

-- �ŏ����@�ɂ�钼���t�B�b�e�B���O�ƃO���t�쐬��

![ex02](./pic/ex02.png "ex02")

- Excel�ɂ��c���̓x�����z

-- 10�_�v��

![g10](./pic/gauss10.png "g10")

-- 100�_�v��

![g100](./pic/gauss100.png "g100")