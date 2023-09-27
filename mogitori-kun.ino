#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

// Function prototype
void moveMecanum(int, int, int);
void driveMotor(int, int, int);
float degreeToRadian(int);

USB Usb;

BTD Btd(&Usb); 
/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
//PS4BT PS4(&Btd);


// WARNNING: Specify here the motor to be used for mecanum
/*
  Front
1       2
        
3       4

5       6
*/
// WARNNING: Connect A to the red conductor
// WARNNING: Connect B to the black conductor
int motor1_dir = 1;
int motor1_pwm = 2;
int motor2_dir = 3;
int motor2_pwm = 4;
int motor3_dir = 5;
int motor3_pwm = 6;
int motor4_dir = 7;
int motor4_pwm = 8;
int motor5_dir = 9;
int motor5_pwm = 10;
int motor6_dir = 11;
int motor6_pwm = 12;

void setup() {
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  // motor 1
  pinMode(motor1_dir, OUTPUT);
  pinMode(motor1_pwm, OUTPUT);
  // motor 2
  pinMode(motor2_dir, OUTPUT);
  pinMode(motor2_pwm, OUTPUT);
  // motor 3
  pinMode(motor3_dir, OUTPUT);
  pinMode(motor4_pwm, OUTPUT);
  // motor 4
  pinMode(motor4_dir, OUTPUT);
  pinMode(motor4_pwm, OUTPUT);
  // motor 5
  pinMode(motor5_dir, OUTPUT);
  pinMode(motor5_pwm, OUTPUT);
  // motor 6
  pinMode(motor6_dir, OUTPUT);
  pinMode(motor6_pwm, OUTPUT);
}



void loop() {
  Usb.Task();

  if (PS4.connected()) {
    // Receive controller input
    int Vx = map(PS4.getAnalogHat(LeftHatX), 0, 255, -255, 255);
    int Vy = map(PS4.getAnalogHat(LeftHatY), 0, 255, -255, 255);
    int Vr = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);

    // Control the mecanum using the received coordinate values
    moveMecanum(Vx, Vy, Vr);
  }
}



// Control mecanum from coordinate values
void moveMecanum(int Vx, int Vy, int Vr) {
  int wheel1, wheel2, wheel3, wheel4, wheel5, wheel6;

  // Whether to rotate or not (to avoid moving while rotating)
  if (abs(Vr) > 30) {
    wheel1 = Vr;
    wheel2 = Vr;
    wheel3 = Vr;
    wheel4 = Vr;
    wheel5 = Vr;
    wheel6 = Vr;

    // Clip speed values to a range of -255 to 255 (prevents overflow)
    wheel1 = constrain(wheel1, -255, 255);
    wheel2 = constrain(wheel2, -255, 255);
    wheel3 = constrain(wheel3, -255, 255);
    wheel4 = constrain(wheel4, -255, 255);
    wheel5 = constrain(wheel5, -255, 255);
    wheel6 = constrain(wheel6, -255, 255);

    driveMotor(motor1_dir, motor1_pwm, wheel1);
    driveMotor(motor2_dir, motor2_pwm, wheel2);
    driveMotor(motor3_dir, motor3_pwm, wheel3);
    driveMotor(motor4_dir, motor4_pwm, wheel4);
    driveMotor(motor5_dir, motor5_pwm, wheel5);
    driveMotor(motor6_dir, motor6_pwm, wheel6);
  } else {
    // Calculate for each mecanum wheel
    wheel1 = Vy - Vx;  // left front
    wheel2 = -Vy - Vx; // right front
    wheel5 = Vy + Vx;  // back left
    wheel6 = -Vy + Vx; // back right

    // WARNNING: Need to modify here depending on how tires are attached.
    /*
    example: 
      wheel3 = -Vy;
      wheel4 = Vy;
    */
    if (abs(Vx) > abs(Vy)) {
      wheel3 = -Vy;  // center left(omni)
      wheel4 = -Vy;  // center right(omni)
    } else {
      wheel3 = 0;  // center left(omni)
      wheel4 = 0;  // center right(omni)
    }

    // Clip speed values to a range of -255 to 255 (prevents overflow)
    wheel1 = constrain(wheel1, -255, 255);
    wheel2 = constrain(wheel2, -255, 255);
    wheel3 = constrain(wheel3, -255, 255);
    wheel4 = constrain(wheel4, -255, 255);
    wheel5 = constrain(wheel5, -255, 255);
    wheel6 = constrain(wheel6, -255, 255);

    // Controls each motor
    driveMotor(motor1_dir, motor1_pwm, wheel1);
    driveMotor(motor2_dir, motor2_pwm, wheel2);
    driveMotor(motor3_dir, motor3_pwm, wheel3);
    driveMotor(motor4_dir, motor4_pwm, wheel4);
    driveMotor(motor5_dir, motor5_pwm, wheel5);
    driveMotor(motor6_dir, motor6_pwm, wheel6);
  }
}



// Convert from degrees to radians
float degreeToRadian(int degree) {
  return (degree * 71) / 4068;
}



// Control motors (0~255)
void driveMotor(int pin_dir, int pin_pwm, int speed) {
  if (speed > 0) {
    digitalWrite(pin_dir, LOW);
    analogWrite(pin_pwm, speed);
  } else {
    digitalWrite(pin_dir, HIGH);
    analogWrite(pin_pwm, speed);
  }
}
