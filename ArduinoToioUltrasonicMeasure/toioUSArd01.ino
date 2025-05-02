#define TRIG_PIN 9
#define ECHO_PIN 10
#define BUTTON_PIN 2

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Arduino組み込みの超音波跳ね返り秒数取得関数
  long duration = pulseIn(ECHO_PIN, HIGH);

  // 超音波センサーによる距離計算
  float distance = duration * 0.034 / 2;

  // ボタントリガー用
  int buttonState = digitalRead(BUTTON_PIN);

  // PCへのシリアル通信
  Serial.print(distance);
  Serial.print(",");
  Serial.print(duration);
  Serial.print(",");
  Serial.println(buttonState);

  delay(500);  
}
