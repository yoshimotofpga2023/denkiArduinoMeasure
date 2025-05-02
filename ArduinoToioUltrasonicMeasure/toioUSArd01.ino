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

  // Arduino�g�ݍ��݂̒����g���˕Ԃ�b���擾�֐�
  long duration = pulseIn(ECHO_PIN, HIGH);

  // �����g�Z���T�[�ɂ�鋗���v�Z
  float distance = duration * 0.034 / 2;

  // �{�^���g���K�[�p
  int buttonState = digitalRead(BUTTON_PIN);

  // PC�ւ̃V���A���ʐM
  Serial.print(distance);
  Serial.print(",");
  Serial.print(duration);
  Serial.print(",");
  Serial.println(buttonState);

  delay(500);  
}
