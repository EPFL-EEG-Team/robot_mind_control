/**
 * @file main.ino
 * 
 * @author Emile Janho Dit Hreich (emile.janhodithreich@epfl.ch)
 *         Michael Freeman (michael.freeman@epfl.ch)
 * 
 * @brief The main arduino script to control the car through eeg, emg an
 * 
 * @version 0.1
 * @date 2022-04-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// ==============================================================================
// LIBRARIES
#include <Servo.h>
#include <SoftwareSerial.h>
#include <stdio.h>

// ==============================================================================
// MACROS AND SETUP

// ---------------------------------------
// motor

// enable pin of motor controller must have pin with pulse modulation
#define MOTOR_EN_PIN  11 
#define MOTOR_IN1  13
#define MOTOR_IN2  12
int MOTOR_SPEED = 0;

// ---------------------------------------
// EMG
enum EMG_State {
  STOP, FORWARD, BACKWARD
};

EMG_State state = STOP;

// --------------------------------------
// Bluetooth

#define TX 3
#define RX 2
#define SERIAL_BAUDRATE 115200
#define BLUETOOTH_BAUDRATE 115200

SoftwareSerial Bluetooth(TX, RX);

// --------------------------------------
// Serial data

// format: EEG_XXXXXXXX
String rawData;
String mode;
int data;

int r     = 0;
char read = 0;


// --------------------------------------
// servo

int servo_pin = 9;
int servo_straight = 90;
int servo_right    = 0;
int servo_left     = 180;

Servo servo;

// ==============================================================================



void setup() {

    Serial.begin(115200);
    Bluetooth.begin(9600);

    // motor
    pinMode(MOTOR_EN_PIN, OUTPUT);
    pinMode(MOTOR_IN1, OUTPUT);
    pinMode(MOTOR_IN2, OUTPUT);

    // servo
    servo.attach(servo_pin);


}

// turns the wheels full right
void turnRight() {
  servo.write(servo_right);
}

// turns the wheels full left
void turnLeft() {
  servo.write(servo_left);
}

void goStraight() {
  servo.write(servo_straight);
}

void changeSpeed(int speed) {
  if (speed <= 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    if (speed == 0) {
      MOTOR_SPEED = 0;
    } else {
      MOTOR_SPEED = -speed;
    }
  } else if (speed >= 200) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    MOTOR_SPEED = 200;
  } else if (speed <= -200) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    MOTOR_SPEED = 200;
  } else {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    MOTOR_SPEED = speed;
  }
  analogWrite(MOTOR_EN_PIN, MOTOR_SPEED);
}

void loop() {
  digitalWrite(MOTOR_IN1, LOW); // Direction is forward
  digitalWrite(MOTOR_IN2, HIGH);

  // check if data is available
  if (Bluetooth.available()){ 

    // retrieve data
    char a = Bluetooth.read();
    rawData.concat(a);
    r += 1;
    if (r == 13){
      r = 0;
      mode = rawData.substring(0, 3);
      data = rawData.substring(4, rawData.length()).toInt();
      rawData = "";

    }
    
    
    if (mode.equals("EEG") && state != STOP) {
      if (state == FORWARD) {
        changeSpeed(data);
      } else if (state == BACKWARD) {
        changeSpeed(-data);
      }  
    } else if (mode.equals("EMG")) {

      if (state == STOP) {
        state = FORWARD;
      } else if (state == FORWARD) {
        state = BACKWARD;
      } else if (state == BACKWARD) {
        state = STOP;
        changeSpeed(0); 
      } 
    } else if (mode.equals("IMU")) {
      
      if (data < -80) {
        turnLeft();
      } else if (data > 80) {
        turnRight();  
      } else {
        goStraight();  
      }
    }
    
  }
  

}