// ESP32-WROOM-32 DevKit v1
// Code: Receive PS4 controller input and send via UART to Arduino

#include <PS4Controller.h>

unsigned long lastTimeStamp = 0;
const int DEADZONE = 15; // adjust as needed (0–127 range per axis)

int applyDeadzone(int value) {
  if (abs(value) < DEADZONE) {
    return 0;
  }
  return value;
}

void notify() {
  // Apply deadzone to joystick values
  int lx = applyDeadzone(PS4.LStickX());
  int ly = applyDeadzone(PS4.LStickY());
  int rx = applyDeadzone(PS4.RStickX());

  // Format joystick values into CSV string
  char messageString[100];
  sprintf(messageString, "%d,%d,%d\n", lx, ly, rx);

  // Send every 50 ms
  if (millis() - lastTimeStamp > 50) {
    Serial2.print(messageString);   // TX2 (GPIO17) → Arduino Mega RX1
    lastTimeStamp = millis();
  }
}

void onConnect() {
  Serial.println("PS4 Controller Connected!");
}

void onDisConnect() {
  Serial.println("PS4 Controller Disconnected!");
}

void setup() {
  Serial.begin(115200);     // Debug output
  Serial2.begin(115200, SERIAL_8N1, 16, 17); // RX=16, TX=17 explicitly

  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  PS4.begin();  // Starts Bluetooth pairing
  Serial.println("ESP32 Ready. Waiting for PS4 controller...");
}

void loop() {
  // Empty loop, everything handled by callbacks
}