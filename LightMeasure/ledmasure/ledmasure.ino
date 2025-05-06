const int ledPin = 9;
const int sensorPin = A0;
const int numSamples = 500;
int pwmValue = 240;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  while (!Serial);  // PCとの接続待ち
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'S') {  // 'S' が来たら測定開始
      analogWrite(ledPin, pwmValue);
      delay(50); // LEDの安定化

      for (int i = 0; i < numSamples; i++) {
        int value = analogRead(sensorPin);
        Serial.println(value);
        delay(10);  // 測定間隔 10ms
      }

      analogWrite(ledPin, 0);  // 測定後にLEDオフ
    }
  }
}
