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
#include <PWMServo.h>
//#include <Servo_Hardware_PWM.h>
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

EMG_State state = FORWARD;

// --------------------------------------
// Bluetooth

#define TX 3
#define RX 2
#define SERIAL_BAUDRATE 2400
#define BLUETOOTH_BAUDRATE 2400

SoftwareSerial Bluetooth(TX, RX);

// --------------------------------------
// Serial data

String rawData = "";
int data;
String mode;
String prevMode;
char read                    = 0;
int r = 0;
int valid_data = 0;
// format: EEG_XXXXXXXX
char str_data_retrieved[13] = {0};

// --------------------------------------
// servo

int servo_pin = 9;
int servo_straight = 90;
int servo_right    = 0;
int servo_left     = 180;

PWMServo servo;

// ==============================================================================



void setup() {

    Serial.begin(115200);
    Serial.println("salut monde");
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
  delay(15);
}

// turns the wheels full left
void turnLeft() {
  servo.write(servo_left);
  delay(15);
}

void goStraight() {
  servo.write(servo_straight);
  delay(15);
}

void changeSpeed(int speed) {
  if (speed <= 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    if (speed == 0) {
      MOTOR_SPEED = 0;
    }else if (speed <= -200) {
      MOTOR_SPEED = 200;
    } else {
      MOTOR_SPEED = -speed;
    }
  } else if (speed >= 200) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    MOTOR_SPEED = 200;
  } else {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    MOTOR_SPEED = speed;
  }
  analogWrite(MOTOR_EN_PIN, MOTOR_SPEED);
}

void loop() {

  // check if data is available

  if (Bluetooth.available()){ 

    char a = Bluetooth.read();
    rawData.concat(a);
    r += 1;
    prevMode = String(mode);
    if (r == 13){
      r = 0;
      mode = rawData.substring(0, 3);
      data = rawData.substring(4, 13).toInt();
      rawData = String();

    }
    
    Serial.println(mode);
    //Serial.println(data);
    
    if (mode == "EEG") {
//      Serial.println("debug");
      if (state == FORWARD) {
        changeSpeed(data);
      } else if (state == BACKWARD) {
        changeSpeed(-data);
      }  
    } else if (mode.equals("EMG")) {
      if (!prevMode.equals("EMG")) {
        if (state == STOP) {
          state = FORWARD;
          digitalWrite(MOTOR_IN1, 1);
          digitalWrite(MOTOR_IN2, 0); 
        } else if (state == FORWARD) {
          state = BACKWARD;
          digitalWrite(MOTOR_IN1, 0);
          digitalWrite(MOTOR_IN2, 1);
        } else if (state == BACKWARD) {
          state = STOP;
          changeSpeed(0); 
        } 
        Serial.println(state);
      }
      
      
    } else if (mode.equals("IMU")) {
      Serial.println("IMU");
      
      if (data < -40) {
//        Serial.println("turning left");
        turnLeft();
      } else if (data > 40) {
        turnRight();  
      } else {
        goStraight();  
      }
    }else if (r == 13){
      
      while(rawData != "EEG" && rawData != "EMG" && rawData != "IMU"){
        rawData = String();
        if (Bluetooth.available()>3){
           rawData.concat(Bluetooth.read());
           rawData.concat(Bluetooth.read());
           rawData.concat(Bluetooth.read());
        }

      }
      r = 3;
    }
    
  }
}
