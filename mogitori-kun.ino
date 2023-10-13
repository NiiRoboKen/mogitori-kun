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
*/
// WARNNING: Connect A to the red conductor
// WARNNING: Connect B to the black conductor
int motor1_dir = 43;
int motor1_pwm = 2;
int motor2_dir = 45;
int motor2_pwm = 3;
int motor3_dir = 6;
int motor3_pwm = 7;
int motor4_dir = 8;
int motor4_pwm = 9;

int relay12 = 0;
int relay12_is_high = false;
int relay34 = 0;
int relay34_is_high = false;
int relay56 = 0;
int relay56_is_high = false;

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

  // relay12
  pinMode(relay12, OUTPUT);
  // relay34
  pinMode(relay34, OUTPUT);
  // relay56
  pinMode(relay56, OUTPUT);
}



void loop() {
  Usb.Task();

  while (true) {
    if (PS4.connected()) {
      // Receive controller input
      int Vx = map(PS4.getAnalogHat(LeftHatX), 0, 255, -255, 255);
      int Vy = map(PS4.getAnalogHat(LeftHatY), 0, 255, -255, 255);
      int Vr = map(PS4.getAnalogHat(RightHatX), 0, 255, -255, 255);

      if (PS4.getButtonClick(CROSS)) {
        relayToggle(relay12, &relay12_is_high);
      }
      if (PS4.getButtonClick(SQUARE)) {
        relayToggle(relay34, &relay34_is_high);
      }
      if (PS4.getButtonClick(CIRCLE)) {
        relayToggle(relay56, &relay56_is_high);
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



// Control mecanum from coordinate values
void moveMecanum(int Vx, int Vy, int Vr) {
    wheel1 = Vy - Vx;  // left front
    wheel2 = -Vy - Vx; // right front
    wheel3 = Vy + Vx;  // back left
    wheel4 = -Vy + Vx; // back right

    driveMotor(motor1_dir, motor1_pwm, wheel1);
    driveMotor(motor2_dir, motor2_pwm, wheel2);
    driveMotor(motor3_dir, motor3_pwm, wheel3);
    driveMotor(motor4_dir, motor4_pwm, wheel4);
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


// Control air cylinders
void relayToggle(int pin_relay, bool* is_high) {
  if (is_high) {
    digitalWrite(pin_relay, LOW);
    *is_high = false;
  } else {
    digitalWrite(pin_relay, HIGH);
    *is_high = true;
  }
}
