#include <Encoder.h>

#include <PID_v1.h>
#include <SPI.h>
#include <PS4BT.h>
#include <usbhub.h>


// USB Stuff

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);



//Define Variables we'll be connecting to
double steeringSetpointOne, steeringInputOne, steeringOutputOne;
double steeringSetpointTwo, steeringInputTwo, steeringOutputTwo;
double steeringSetpointThree, steeringInputThree, steeringOutputThree;
double steeringSetpointFour, steeringInputFour, steeringOutputFour;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID steeringPIDOne(&steeringInputOne, &steeringOutputOne, &steeringSetpointOne, Kp, Ki, Kd, DIRECT);
PID steeringPIDTwo(&steeringInputTwo, &steeringOutputTwo, &steeringSetpointTwo, Kp, Ki, Kd, DIRECT);
PID steeringPIDThree(&steeringInputThree, &steeringOutputThree, &steeringSetpointThree, Kp, Ki, Kd, DIRECT);
PID steeringPIDFour(&steeringInputFour, &steeringOutputFour, &steeringSetpointThree, Kp, Ki, Kd, DIRECT);



// Module 1

// Motor 1 - Drive Motor
#define motor_1_en 2
#define motor_1_in_A 10
#define motor_1_in_B 11
#define motor_1_encoder_A 34
#define motor_1_encoder_B 35

// Motor 2 - Steering Motor
#define motor_2_en 3
#define motor_2_in_A 12
#define motor_2_in_B 13
#define motor_2_encoder_A 36
#define motor_2_encoder_B 37
// Module 2

// Motor 3 - Drive Motor
#define motor_3_en 4
#define motor_3_in_A 22
#define motor_3_in_B 23
#define motor_3_encoder_A 38
#define motor_3_encoder_B 39

// Motor 4 - Steering Motor
#define motor_4_en 5
#define motor_4_in_A 24
#define motor_4_in_B 25
#define motor_4_encoder_A 40
#define motor_4_encoder_B 41
// Module 3

// Motor 5 - Drive Motor
#define motor_5_en 6
#define motor_5_in_A 26
#define motor_5_in_B 27
#define motor_5_encoder_A 42
#define motor_5_encoder_B 43

// Motor 6 - Steering Motor
#define motor_6_en 7
#define motor_6_in_A 28
#define motor_6_in_B 29
#define motor_6_encoder_A 44
#define motor_6_encoder_B 45

// Module 4

// Motor 7 - Drive Motor
#define motor_7_en 6
#define motor_7_in_A 26
#define motor_7_in_B 27
#define motor_7_encoder_A 46
#define motor_7_encoder_B 47

// Motor 8 - Steering Motor
#define motor_8_en 7
#define motor_8_in_A 28
#define motor_8_in_B 29
#define motor_8_encoder_A 48
#define motor_8_encoder_B 49

// Encoders

Encoder motor_1_encoder(motor_1_encoder_A, motor_1_encoder_B);
Encoder motor_2_encoder(motor_2_encoder_A, motor_2_encoder_B);
Encoder motor_3_encoder(motor_3_encoder_A, motor_3_encoder_B);
Encoder motor_4_encoder(motor_4_encoder_A, motor_4_encoder_B);
Encoder motor_5_encoder(motor_5_encoder_A, motor_5_encoder_B);
Encoder motor_6_encoder(motor_6_encoder_A, motor_6_encoder_B);
Encoder motor_7_encoder(motor_7_encoder_A, motor_7_encoder_B);
Encoder motor_8_encoder(motor_8_encoder_A, motor_8_encoder_B);

// Set position

long encoder_1_old_position = 0;
long encoder_2_old_position = 0;
long encoder_3_old_position = 0;
long encoder_4_old_position = 0;
long encoder_5_old_position = 0;
long encoder_6_old_position = 0;
long encoder_7_old_position = 0;
long encoder_8_old_position = 0;

const double gear_ratio = 30.0;
const double TOLERANCE = 1.0; // Acceptable range around the Setpoint


void setup() {
  // put your setup code here, to run once:

	// Set all the motor control pins to outputs
	pinMode(motor_1_en, OUTPUT);
	pinMode(motor_1_in_A, OUTPUT);
	pinMode(motor_1_in_B, OUTPUT);
  
  pinMode(motor_2_en, OUTPUT);
	pinMode(motor_2_in_A, OUTPUT);
	pinMode(motor_2_in_B, OUTPUT);

  pinMode(motor_3_en, OUTPUT);
	pinMode(motor_3_in_A, OUTPUT);
	pinMode(motor_3_in_B, OUTPUT);

  pinMode(motor_4_en, OUTPUT);
	pinMode(motor_4_in_A, OUTPUT);
	pinMode(motor_4_in_B, OUTPUT);

  pinMode(motor_5_en, OUTPUT);
	pinMode(motor_5_in_A, OUTPUT);
	pinMode(motor_5_in_B, OUTPUT);

  pinMode(motor_6_en, OUTPUT);
	pinMode(motor_6_in_A, OUTPUT);
	pinMode(motor_6_in_B, OUTPUT);

  pinMode(motor_7_en, OUTPUT);
	pinMode(motor_7_in_A, OUTPUT);
	pinMode(motor_7_in_B, OUTPUT);

  pinMode(motor_8_en, OUTPUT);
	pinMode(motor_8_in_A, OUTPUT);
	pinMode(motor_8_in_B, OUTPUT);

  // Set all encoder pins to inputs

  pinMode(motor_1_encoder_A, INPUT);
  pinMode(motor_1_encoder_B, INPUT);

  pinMode(motor_2_encoder_A, INPUT);
  pinMode(motor_2_encoder_B, INPUT);

  pinMode(motor_3_encoder_A, INPUT);
  pinMode(motor_3_encoder_B, INPUT);

  pinMode(motor_4_encoder_A, INPUT);
  pinMode(motor_4_encoder_B, INPUT);

  pinMode(motor_5_encoder_A, INPUT);
  pinMode(motor_5_encoder_B, INPUT);

  pinMode(motor_6_encoder_A, INPUT);
  pinMode(motor_6_encoder_B, INPUT);

  pinMode(motor_7_encoder_A, INPUT);
  pinMode(motor_7_encoder_B, INPUT);

  pinMode(motor_8_encoder_A, INPUT);
  pinMode(motor_8_encoder_B, INPUT);
	
	// Turn off motors - Initial state

	digitalWrite(motor_1_in_A, LOW);
	digitalWrite(motor_1_in_B, LOW);

	digitalWrite(motor_2_in_A, LOW);
	digitalWrite(motor_2_in_B, LOW);
  
  digitalWrite(motor_3_in_A, LOW);
	digitalWrite(motor_3_in_B, LOW);

  digitalWrite(motor_4_in_A, LOW);
	digitalWrite(motor_4_in_B, LOW);

  digitalWrite(motor_5_in_A, LOW);
	digitalWrite(motor_5_in_B, LOW);

  digitalWrite(motor_6_in_A, LOW);
	digitalWrite(motor_6_in_B, LOW);

  digitalWrite(motor_7_in_A, LOW);
	digitalWrite(motor_7_in_B, LOW);

  digitalWrite(motor_8_in_A, LOW);
	digitalWrite(motor_8_in_B, LOW);


  Serial.begin(9600);
}

void loop() {


  checkEncoders();
}

void controller(){
  Usb.Task();

  if (PS4.connected()) {

    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    swerveControl(PS4.getAnalogHat(LeftHatX), PS4.getAnalogHat(LeftHatY), PS4.getAnalogHat(RightHatX));
  }

}

void checkEncoders(){
  long encoder_1_new_position = motor_1_encoder.read();
  if (encoder_1_new_position != encoder_1_old_position) {
    encoder_1_old_position = encoder_1_new_position;
    Serial.println(encoder_1_old_position);
  }

  long encoder_2_new_position = motor_2_encoder.read();
  if (encoder_2_new_position != encoder_2_old_position) {
    encoder_2_old_position = encoder_2_new_position;
    Serial.println(encoder_2_old_position);
  }

  long encoder_3_new_position = motor_3_encoder.read();
  if (encoder_3_new_position != encoder_3_old_position) {
    encoder_3_old_position = encoder_3_new_position;
    Serial.println(encoder_3_old_position);
  }

  long encoder_4_new_position = motor_4_encoder.read();
  if (encoder_4_new_position != encoder_4_old_position) {
    encoder_4_old_position = encoder_4_new_position;
    Serial.println(encoder_4_old_position);
  }

  long encoder_5_new_position = motor_5_encoder.read();
  if (encoder_5_new_position != encoder_5_old_position) {
    encoder_5_old_position = encoder_5_new_position;
    Serial.println(encoder_5_old_position);
  }

  long encoder_6_new_position = motor_6_encoder.read();
  if (encoder_6_new_position != encoder_6_old_position) {
    encoder_6_old_position = encoder_6_new_position;
    Serial.println(encoder_6_old_position);
  }

  long encoder_7_new_position = motor_7_encoder.read();
  if (encoder_7_new_position != encoder_7_old_position) {
    encoder_7_old_position = encoder_7_new_position;
    Serial.println(encoder_7_old_position);
  }

  long encoder_8_new_position = motor_8_encoder.read();
  if (encoder_8_new_position != encoder_8_old_position) {
    encoder_8_old_position = encoder_8_new_position;
    Serial.println(encoder_8_old_position);
  }
}

void swerveControl(double velocityX, double velocityY, double angularVelocity){

  // Module 1
  double velocity_1_x = velocityX + angularVelocity*cos(3*PI/4); 
  double velocity_1_y = velocityY + angularVelocity*sin(3*PI/4);

  // Module 2
  double velocity_2_x = velocityX + angularVelocity*cos(5*PI/4); 
  double velocity_2_y = velocityY + angularVelocity*sin(5*PI/4);

  // Module 3
  double velocity_3_x = velocityX + angularVelocity*cos(7*PI/4); 
  double velocity_3_y = velocityY + angularVelocity*sin(7*PI/4);

  // Module 4
  double velocity_4_x = velocityX + angularVelocity*cos(PI/4); 
  double velocity_4_y = velocityY + angularVelocity*sin(PI/4);

  // Calculate velocity and angle for each module
  double velocity_1 = sqrt(velocity_1_x * velocity_1_x + velocity_1_y * velocity_1_y);
  double angle_1 = atan2(velocity_1_y, velocity_1_x);

  double velocity_2 = sqrt(velocity_2_x * velocity_2_x + velocity_2_y * velocity_2_y);
  double angle_2 = atan2(velocity_2_y, velocity_2_x);

  double velocity_3 = sqrt(velocity_3_x * velocity_3_x + velocity_3_y * velocity_3_y);
  double angle_3 = atan2(velocity_3_y, velocity_3_x);

  double velocity_4 = sqrt(velocity_4_x * velocity_4_x + velocity_4_y * velocity_4_y);
  double angle_4 = atan2(velocity_4_y, velocity_4_x);

  // find the biggest value, if greater than .... it will be supressed
  double max_velocity = max(max(velocity_1, velocity_2), max(velocity_3, velocity_4));
  if (max_velocity>255){              // velocity will be max 255
    double ratio = max_velocity/255;
    velocity_1 /= ratio;
    velocity_2 /= ratio;
    velocity_3 /= ratio;
    velocity_4 /= ratio;
  }

  // If the angle has to move more than 90 degrees... NEED TO WRITE CODE

  // Move motors --- Output
  module_1_control(velocity_1, angle_1);
  module_2_control(velocity_2, angle_2);
  module_3_control(velocity_3, angle_3);
  module_4_control(velocity_4, angle_4);
}


void module_1_control(double velocity, double angle){
  // Drive motor
  analogWrite(motor_1_en, velocity);

  // Steering motor
  
  //turn the PID on
  steeringSetpointOne = angle;
  steeringInputOne = get_module_1_angle();

  steeringPIDOne.SetMode(AUTOMATIC);


  while (abs(steeringSetpointOne - steeringInputOne) > TOLERANCE) {
    steeringInputOne = get_module_1_angle();
    steeringPIDOne.Compute();                 // Compute PID output
    analogWrite(motor_2_en, steeringOutputOne); // PID output speed for steering
  }

  // Optional: Turn off output once setpoint is reached
  analogWrite(motor_2_en, 0); // or some safe/default value
  steeringPIDOne.SetMode(MANUAL);      // Stop the PID controller
}

void module_2_control(double velocity, double angle){
  // Drive motor
  analogWrite(motor_3_en, velocity);

  // Steering motor
  
  //turn the PID on
  steeringSetpointTwo = angle;
  steeringInputTwo = get_module_2_angle();

  steeringPIDTwo.SetMode(AUTOMATIC);

  while (abs(steeringSetpointTwo - steeringInputTwo) > TOLERANCE) {
    steeringInputTwo = get_module_2_angle();
    steeringPIDTwo.Compute();                 // Compute PID output
    analogWrite(motor_4_en, steeringOutputTwo); // PID output speed for steering
  }

  // Optional: Turn off output once setpoint is reached
  analogWrite(motor_4_en, 0); // or some safe/default value
  steeringPIDTwo.SetMode(MANUAL);      // Stop the PID controller
}

void module_3_control(double velocity, double angle){
  // Drive motor
  analogWrite(motor_5_en, velocity);

  // Steering motor
  
  //turn the PID on
  steeringSetpointThree = angle;
  steeringInputThree = get_module_3_angle();

  steeringPIDThree.SetMode(AUTOMATIC);

  while (abs(steeringSetpointThree - steeringInputThree) > TOLERANCE) {
    steeringInputThree = get_module_3_angle();
    steeringPIDThree.Compute();                 // Compute PID output
    analogWrite(motor_6_en, steeringOutputThree); // PID output speed for steering
  }

  // Optional: Turn off output once setpoint is reached
  analogWrite(motor_6_en, 0); // or some safe/default value
  steeringPIDThree.SetMode(MANUAL);      // Stop the PID controller
}

void module_4_control(double velocity, double angle){
  // Drive motor
  analogWrite(motor_7_en, velocity);

  // Steering motor
  
  //turn the PID on
  double steeringSetpointFour = angle;
  double steeringInputFour = get_module_4_angle();

  steeringPIDFour.SetMode(AUTOMATIC);

  while (abs(steeringSetpointFour - steeringInputFour) > TOLERANCE) {
    steeringInputFour = get_module_4_angle();
    steeringPIDFour.Compute();                 // Compute PID output
    analogWrite(motor_8_en, steeringOutputFour); // PID output speed for steering
  }

  // Optional: Turn off output once setpoint is reached
  analogWrite(motor_8_en, 0); // or some safe/default value
  steeringPIDFour.SetMode(MANUAL);      // Stop the PID controller
}

// Get rpm
void get_module_1_rpm(){
  
}

// Get wheel angle
double get_module_1_angle(){
  return encoder_2_old_position*gear_ratio;
}


// Get rpm
void get_module_2_rpm(){

}

// Get wheel angle
double get_module_2_angle(){
  return encoder_4_old_position*gear_ratio;
}

// Get rpm
void get_module_3_rpm(){

}

// Get wheel angle
double get_module_3_angle(){
  return encoder_6_old_position*gear_ratio;
}

// Get rpm
void get_module_4_rpm(){

}

// Get wheel angle
double get_module_4_angle(){
  return encoder_8_old_position*gear_ratio;
}