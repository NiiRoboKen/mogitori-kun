#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
//PS4BT PS4(&Btd, PAIR);

// After that you can simply create the instance like so and then press the PS button on the device
PS4BT PS4(&Btd);


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
}

struct Motor {
  int dir_pin;
  int pwm_pin;
};

struct Relay {
  int pin;
  bool is_high;
};

// WARNNING: Specify here the motor to be used for mecanum
/*
  Front
1       2
        
3       4
        
5       6
*/
// WARNNING: Connect A to the red conductor
// WARNNING: Connect B to the black conductor
Motor front_left_motor = {29, 9};
Motor front_right_motor = {37, 10};
Motor center_left_motor = {28, 8};
Motor center_right_motor = {35, 6};
Motor rear_left_motor = {36, 4};
Motor rear_right_motor = {39, 5};

Motor arm_pair1 = {48, 46};
Motor arm_pair2 = {47, 44};
Motor arm_pair3 = {34, 45};

Relay front_air_cylinder = {24, true};
Relay center_air_cylinder = {26, true};
Relay rear_air_cylinder = {33, true};


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

    driveMotor(&front_left_motor, wheel1);
    driveMotor(&front_right_motor, wheel2);
    driveMotor(&center_left_motor, wheel3);
    driveMotor(&center_right_motor, wheel4);
    driveMotor(&rear_left_motor, wheel5);
    driveMotor(&rear_right_motor, wheel6);
  } else {
    // Calculate for each mecanum wheel
    wheel1 = Vy + Vx;  // left front
    wheel2 = Vy - Vx; // right front
    wheel5 = Vy + Vx;  // back left
    wheel6 = Vy - Vx; // back right

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
    driveMotor(&front_left_motor, wheel1);
    driveMotor(&front_right_motor, wheel2);
    driveMotor(&center_left_motor, wheel3);
    driveMotor(&center_right_motor, wheel4);
    driveMotor(&rear_left_motor, wheel5);
    driveMotor(&rear_right_motor, wheel6);
  }
}



// Convert from degrees to radians
float degreeToRadian(int degree) {
  return (degree * 71) / 4068;
}



// Control motors (0~255)
void driveMotor(struct Motor* motor, int speed) {
  if (speed > 0) {
    digitalWrite(motor->dir_pin, LOW);
    analogWrite(motor->pwm_pin, speed);
  } else {
    digitalWrite(motor->dir_pin, HIGH);
    analogWrite(motor->pwm_pin, speed);
  }
}


// Control air cylinders
void relayToggle(struct Relay* relay) {
  if (relay->is_high) {
    digitalWrite(relay->pin, LOW);
    relay->is_high = false;
  } else {
    digitalWrite(relay->pin, HIGH);
    relay->is_high = true;
  }
}

void init_pins() {
  // motor 1
  pinMode(front_left_motor.dir_pin, OUTPUT);
  pinMode(front_left_motor.pwm_pin, OUTPUT);
  // motor 2
  pinMode(front_right_motor.dir_pin, OUTPUT);
  pinMode(front_right_motor.pwm_pin, OUTPUT);
  // motor 3
  pinMode(center_left_motor.dir_pin, OUTPUT);
  pinMode(center_left_motor.pwm_pin, OUTPUT);
  // motor 4
  pinMode(center_right_motor.dir_pin, OUTPUT);
  pinMode(center_right_motor.pwm_pin, OUTPUT);

  // arm_motor_pair1
  pinMode(arm_pair1.dir_pin, OUTPUT);
  pinMode(arm_pair1.pwm_pin, OUTPUT);
  // arm_motor_pair2
  pinMode(arm_pair2.dir_pin, OUTPUT);
  pinMode(arm_pair2.pwm_pin, OUTPUT);
  // arm_motor_pair3
  pinMode(arm_pair3.dir_pin, OUTPUT);
  pinMode(arm_pair3.pwm_pin, OUTPUT);

  pinMode(front_air_cylinder.pin, OUTPUT);
  pinMode(center_air_cylinder.pin, OUTPUT);
  pinMode(rear_air_cylinder.pin, OUTPUT);
}

void loop() {
  Usb.Task();

  init_pins();

  while (true) {
    if (PS4.connected()) {
      // Receive controller input
      int Vx = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);
      int Vy = map(PS4.getAnalogHat(RightHatY), 0, 255, -255, 255);
      // int Vr = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);:
      int Vr  = 0;

      if (PS4.getButtonClick(CROSS)) {
        relayToggle(&front_air_cylinder);
        Serial.print(F("\r\nCROSS"));
      }
      if (PS4.getButtonClick(SQUARE)) {
        relayToggle(&center_air_cylinder);
        Serial.print(F("\r\nSQUARE"));
      }
      if (PS4.getButtonClick(CIRCLE)) {
        relayToggle(&rear_air_cylinder);
        Serial.print(F("\r\nCIRCLE"));
      }


      if (PS4.getButtonPress(UP)) {
        driveMotor(&arm_pair1, 180);
        Serial.print(F("\r\nUP"));
      } 
      if (PS4.getButtonPress(DOWN)) {
        driveMotor(&arm_pair1, -180);
        Serial.print(F("\r\nDOWN"));
      } 
      if (PS4.getButtonPress(R1)) {
        driveMotor(&arm_pair2, 180);
        Serial.print(F("\r\nR1"));
      }
      if (PS4.getAnalogButton(R2) < 180) {
        driveMotor(&arm_pair2, -180);
        Serial.print(F("\r\nR2"));
      }
      if (PS4.getButtonPress(L1)) {
        driveMotor(&arm_pair3, 180);
        Serial.print(F("\r\nL1"));
      }
      if (PS4.getAnalogButton(L2) < 180) {
        driveMotor(&arm_pair3, -180);
        Serial.print(F("\r\nL2"));
      }

      // If the input is too small, ignore it(Input range is -30 to 30)
      if (abs(Vx) < 30 && abs(Vy) < 30 && abs(Vr) < 30) {
        continue;
      }

      // Control the mecanum using the received coordinate values
      moveMecanum(Vx, Vy, Vr);
    }
  }
}

