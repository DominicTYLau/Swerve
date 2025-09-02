#include <PinChangeInterrupt.h>
#include <math.h>

// -------------------- Motor Pins --------------------
// Steering motors
#define motor_1_en 8
#define motor_1_in_A 9
#define motor_1_in_B 31

#define motor_3_en 25
#define motor_3_in_A 26
#define motor_3_in_B 27

#define motor_5_en 38
#define motor_5_in_A 39
#define motor_5_in_B 40

#define motor_7_en 2
#define motor_7_in_A 3
#define motor_7_in_B 4

// Drive motors
#define motor_2_en 28
#define motor_2_in_A 29
#define motor_2_in_B 30

#define motor_4_en 22
#define motor_4_in_A 23
#define motor_4_in_B 24

#define motor_6_en 41
#define motor_6_in_A 42
#define motor_6_in_B 43

#define motor_8_en 5
#define motor_8_in_A 6
#define motor_8_in_B 7

// -------------------- Encoder Pins --------------------
#define ENC1_A 10
#define ENC1_B 11
#define ENC3_A 50
#define ENC3_B 51
#define ENC5_A 52
#define ENC5_B 53
#define ENC7_A 12
#define ENC7_B 13

volatile long encoder1Count = 0;
volatile long encoder3Count = 0;
volatile long encoder5Count = 0;
volatile long encoder7Count = 0;

// -------------------- Encoder Constants --------------------
const float COUNTS_PER_REV = 1970.0;
const float DEG_PER_COUNT = 360.0 / COUNTS_PER_REV;

// -------------------- Robot Geometry --------------------
const float WHEELBASE = 0.12; // meters
const float TRACKWIDTH = 0.12; // meters

// wheel positions relative to robot center
const float wheelX[4] = { WHEELBASE/2,  WHEELBASE/2, -WHEELBASE/2, -WHEELBASE/2 };
const float wheelY[4] = { TRACKWIDTH/2, -TRACKWIDTH/2, -TRACKWIDTH/2,  TRACKWIDTH/2 };

// -------------------- Control --------------------
const float deadzone = 8.0;

// -------------------- Helper Functions --------------------
float countsToDegrees(long counts) {
  float deg = counts * DEG_PER_COUNT;
  deg = fmod(deg, 360.0);
  if (deg < 0) deg += 360.0;
  return deg;
}

float closestAngle(float current, float target) {
  float delta = fmod(target - current + 540.0, 360.0) - 180.0;
  return delta;
}

void setSteeringMotor(int en, int inA, int inB, float errorDeg) {
  if (errorDeg > deadzone) {
    digitalWrite(inA, HIGH); digitalWrite(inB, LOW);
    analogWrite(en, 255);
  } else if (errorDeg < -deadzone) {
    digitalWrite(inA, LOW); digitalWrite(inB, HIGH);
    analogWrite(en, 255);
  } else {
    digitalWrite(inA, LOW); digitalWrite(inB, LOW);
    analogWrite(en, 0);
  }
}

void setDriveMotor(int en, int inA, int inB, float speed) {
  int pwm = constrain(abs(speed) * 255, 0, 255);
  if (speed > 0) {
    digitalWrite(inA, HIGH); digitalWrite(inB, LOW);
  } else if (speed < 0) {
    digitalWrite(inA, LOW); digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW); digitalWrite(inB, LOW);
  }
  analogWrite(en, pwm);
}

void optimizeAngle(float &targetAngle, float &wheelSpeed, float currentAngle) {
  float delta = closestAngle(currentAngle, targetAngle);
  if (delta > 90.0) {
    targetAngle = fmod(targetAngle + 180.0, 360.0);
    wheelSpeed *= -1;
  } else if (delta < -90.0) {
    targetAngle = fmod(targetAngle - 180.0, 360.0);
    wheelSpeed *= -1;
  }
}

// -------------------- Encoder ISR --------------------
void encoder1ISR() { if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) encoder1Count++; else encoder1Count--; }
void encoder3ISR() { if (digitalRead(ENC3_A) == digitalRead(ENC3_B)) encoder3Count++; else encoder3Count--; }
void encoder5ISR() { if (digitalRead(ENC5_A) == digitalRead(ENC5_B)) encoder5Count++; else encoder5Count--; }
void encoder7ISR() { if (digitalRead(ENC7_A) == digitalRead(ENC7_B)) encoder7Count++; else encoder7Count--; }

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  int motorPins[] = {motor_1_en, motor_1_in_A, motor_1_in_B,
                     motor_2_en, motor_2_in_A, motor_2_in_B,
                     motor_3_en, motor_3_in_A, motor_3_in_B,
                     motor_4_en, motor_4_in_A, motor_4_in_B,
                     motor_5_en, motor_5_in_A, motor_5_in_B,
                     motor_6_en, motor_6_in_A, motor_6_in_B,
                     motor_7_en, motor_7_in_A, motor_7_in_B,
                     motor_8_en, motor_8_in_A, motor_8_in_B};
  for (int i=0; i<24; i++) pinMode(motorPins[i], OUTPUT);

  pinMode(ENC1_A, INPUT_PULLUP); pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC3_A, INPUT_PULLUP); pinMode(ENC3_B, INPUT_PULLUP);
  pinMode(ENC5_A, INPUT_PULLUP); pinMode(ENC5_B, INPUT_PULLUP);
  pinMode(ENC7_A, INPUT_PULLUP); pinMode(ENC7_B, INPUT_PULLUP);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENC1_A), encoder1ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENC3_A), encoder3ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENC5_A), encoder5ISR, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENC7_A), encoder7ISR, CHANGE);

  Serial.println("Swerve Drive Ready");
}

// -------------------- Loop --------------------
void loop() {
  static String input = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      int Lx, Ly, Rx;
      sscanf(input.c_str(), "%d,%d,%d", &Lx, &Ly, &Rx);
      input = "";

      float x = Lx / 127.0;
      float y = -Ly / 127.0;
      float rot = Rx / 127.0;

      float wheelSpeeds[4];
      float wheelAngles[4];

      if (fabs(x) < 0.05 && fabs(y) < 0.05 && fabs(rot) > 0.05) {
        // --- Pure rotation in place ---
        // wheels face tangent to circle: 45, 135, 225, 315
        wheelAngles[0] = 135;   wheelSpeeds[0] = fabs(rot);
        wheelAngles[1] = 45;  wheelSpeeds[1] = fabs(rot);
        wheelAngles[2] = 315;  wheelSpeeds[2] = fabs(rot);
        wheelAngles[3] = 225;  wheelSpeeds[3] = fabs(rot);


        // direction based on sign of rot
        if (rot < 0) {
          for (int i=0; i<4; i++) wheelSpeeds[i] *= -1; // CW
        }

      } else {
        // --- Normal swerve math ---
        for (int i=0; i<4; i++) {
          float vx = x + (-rot * wheelY[i]);
          float vy = y + ( rot * wheelX[i]);
          wheelSpeeds[i] = sqrt(vx*vx + vy*vy);
          wheelAngles[i] = atan2(vy, vx) * 180.0 / M_PI;
          if (wheelAngles[i] < 0) wheelAngles[i] += 360.0;
        }
        float maxWs = max(max(wheelSpeeds[0], wheelSpeeds[1]), max(wheelSpeeds[2], wheelSpeeds[3]));
        if (maxWs > 1.0) for (int i=0; i<4; i++) wheelSpeeds[i] /= maxWs;
      }

      float current[4] = {
        countsToDegrees(encoder1Count),
        countsToDegrees(encoder3Count),
        countsToDegrees(encoder5Count),
        countsToDegrees(encoder7Count)
      };

      for (int i=0; i<4; i++) optimizeAngle(wheelAngles[i], wheelSpeeds[i], current[i]);

      float err1 = closestAngle(current[0], wheelAngles[0]);
      float err3 = closestAngle(current[1], wheelAngles[1]);
      float err5 = closestAngle(current[2], wheelAngles[2]);
      float err7 = closestAngle(current[3], wheelAngles[3]);

      setSteeringMotor(motor_1_en, motor_1_in_A, motor_1_in_B, err1);
      setSteeringMotor(motor_3_en, motor_3_in_A, motor_3_in_B, err3);
      setSteeringMotor(motor_5_en, motor_5_in_A, motor_5_in_B, err5);
      setSteeringMotor(motor_7_en, motor_7_in_A, motor_7_in_B, err7);

      setDriveMotor(motor_2_en, motor_2_in_A, motor_2_in_B, wheelSpeeds[0]);
      setDriveMotor(motor_4_en, motor_4_in_A, motor_4_in_B, wheelSpeeds[1]);
      setDriveMotor(motor_6_en, motor_6_in_A, motor_6_in_B, wheelSpeeds[2]);
      setDriveMotor(motor_8_en, motor_8_in_A, motor_8_in_B, wheelSpeeds[3]);

      Serial.print("Angles: ");
      for (int i=0; i<4; i++) {
        Serial.print(wheelAngles[i]); Serial.print(", ");
      }
      Serial.println();
    } else {
      input += c;
    }
  }
}
