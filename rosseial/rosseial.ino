#include "PID.h"

// Khởi tạo PID cho mỗi bánh xe
PID pidL(-255, 255, 3.0, 0.98, 0.1);  // Motor trái
PID pidR(-255, 255, 3.0, 0.98, 0.1);  // Motor phải

volatile long encoderCountL = 0;
volatile long encoderCountR = 0;

float setpointL = 0;
float setpointR = 0;
float measuredL = 0;
float measuredR = 0;

unsigned long lastTime = 0;
int interval = 100; // ms

void demxungL() {
  encoderCountL++;
}

void demxungR() {
  encoderCountR++;
}

void setup() {
  Serial.begin(9600);

  // Encoder
  pinMode(2, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(2), demxungL, FALLING);
  pinMode(3, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(3), demxungR, FALLING);

  // Motor trái
  pinMode(4, OUTPUT);  // DIR1
  pinMode(10, OUTPUT); // DIR2
  pinMode(11, OUTPUT); // PWM

  // Motor phải
  pinMode(5, OUTPUT);  // DIR1
  pinMode(12, OUTPUT); // DIR2
  pinMode(13, OUTPUT); // PWM

  // Dừng motor ban đầu
  analogWrite(11, 0);
  analogWrite(13, 0);
}

void loop() {
  // Đọc dữ liệu tốc độ từ ROS (qua Serial)
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
      int left = data.substring(0, commaIndex).toInt();
      int right = data.substring(commaIndex + 1).toInt();
      setpointL = left;
      setpointR = right;
    }
  }

  unsigned long now = millis();
  if (now - lastTime >= interval) {
    lastTime = now;

    // Đọc số xung đo được
    measuredL = encoderCountL;
    measuredR = encoderCountR;
    encoderCountL = 0;
    encoderCountR = 0;

    // Tính PID
    float outputL = pidL.compute(setpointL, measuredL);
    float outputR = pidR.compute(setpointR, measuredR);

    // Điều khiển motor trái
    if (outputL >= 0) {
      digitalWrite(4, HIGH);  // Forward
      digitalWrite(10, LOW);
    } else {
      digitalWrite(4, LOW);   // Reverse
      digitalWrite(10, HIGH);
      outputL = -outputL;
    }
    analogWrite(11, constrain(outputL, 0, 255));

    // Điều khiển motor phải
    if (outputR >= 0) {
      digitalWrite(5, HIGH);
      digitalWrite(12, LOW);
    } else {
      digitalWrite(5, LOW);
      digitalWrite(12, HIGH);
      outputR = -outputR;
    }
    analogWrite(13, constrain(outputR, 0, 255));

    // Gửi dữ liệu về ROS
    Serial.print("L: ");
    Serial.print(measuredL);
    Serial.print(" R: ");
    Serial.print(measuredR);
    Serial.print(" PID_L: ");
    Serial.print(outputL);
    Serial.print(" PID_R: ");
    Serial.println(outputR);
  }
}
