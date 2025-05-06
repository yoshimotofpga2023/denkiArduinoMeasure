const int pwmPin = 5;
const int switchPin = 2;
const int reedPin = 3;

int pwmValues[4] = {64, 128, 192, 255};
int pwmIndex = 0;
bool measuring = false;
unsigned long startTime;
int rotationCount = 0;

void setup() {
  pinMode(pwmPin, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(reedPin, INPUT_PULLUP);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(reedPin), countRotation, RISING);
}

void loop() {
  if (digitalRead(switchPin) == HIGH && !measuring) {
    measuring = true;
    rotationCount = 0;
    analogWrite(pwmPin, pwmValues[pwmIndex]);
    startTime = millis();
  }

  if (rotationCount >= 10 && measuring) {
    unsigned long elapsed = millis() - startTime;
    analogWrite(pwmPin, 0);

    float speed_rpm = 600000.0 / elapsed;

    Serial.print(pwmValues[pwmIndex]);
    Serial.print(",");
    Serial.println(speed_rpm, 2);

    pwmIndex++;
    measuring = false;
    delay(1000);
    if (pwmIndex >= 4) {
      while (1);
    }
  }
}

void countRotation() {
  static unsigned long lastTrigger = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTrigger > 100) {
    rotationCount++;
    lastTrigger = currentTime;
  }
}
