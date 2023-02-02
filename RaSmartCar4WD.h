#include <Arduino.h>
#include <Servo.h>
#include <RaKsRemoteControl.h>
#include <LedMatrixAiP1640.h>
#include <SR04.h>

// LED
#define PIN_LED 9

// Servo
#define PIN_SERVO A3 //servo Pin

// Line Tracking
#define PIN_TRACKING_LEFT 11
#define PIN_TRACKING_MIDDLE 7
#define PIN_TRACKING_RIGHT 8

// HC-SR04
#define PIN_TRIGGER 12  
#define PIN_ECHO 13

// Remote Controle
#define PIN_IR_RECEIVER 3

// Motors
#define PIN_MOTOR_L_CTRL 4     // define the direction control pin of B motor
#define PIN_MOTOR_L_PWM 5   //define the PWM control pin of B motor
#define PIN_MOTOR_R_CTRL 2    //define direction control pin of A motor
#define PIN_MOTOR_R_PWM 6   //define the PWM control pin of A motor

// LED Matrix
#define PIN_MATRIX_CLOCK A5
#define PIN_MATRIX_DATA A4

#define DIST_UNIT_CM 0
#define DIST_UNIT_INCH 1

#define SPEED_MAX 255
#define SPEED_STEP 10

#define BT_MODE_RUN 1
#define BT_MODE_ANTI_DROP 2
#define BT_MODE_LINE_TRACKING 3
#define BT_MODE_AVOID 4
#define BT_MODE_FOLLOWING 5

class RaSmartCar4WD
{
private:
  bool debug;
  int distanceUnit;
  int speed;
  Servo servoHead;
  RaKsRemoteControl* rcHandler;
  LedMatrixAiP1640* ledMatrix;
  SR04* distSensor;
  bool showSymbols;
  int btMode;


public:
  RaSmartCar4WD();

  // Init
  void init();

  // Debug
  void setDebug(bool dbg);

  // Servo
  void setServoAnglePWM(int iAngle);
  void setServoAngle(int iAngle);
  
  // LED
  void switchLed(bool status);
  void blinkLed(int iDelay);
  void breathLed();

  // Tracking sensor
  int getLeftTrack();
  int getMiddleTrack();
  int getRightTrack();
  void checkTrack();

  // Servo
  void calibrateServo();

  // Ultrasonic sensor
  void setDistanceUnit(int unit);
  float getDistance();
  void enableFollowMovingObjects();
  void enableAvoidObstacles();

  // IR Remote Control
  void checkRemoteControl();
  void handleRemoteControl();

  // Bluetooth
  void debugBluetooth();
  void enableBluetoothControl();

  // Wheels control
  void setSpeed(int iSpeed);
  void goForward();
  void goForward(int iSpeed);
  void goBackward();
  void goBackward(int iSpeed);
  void turnLeft();
  void turnLeft(int iSpeed);
  void turnRight();
  void turnRight(int iSpeed);
  void stop();

  // LED Matrix
  void setShowSymbols(bool iShow);
  void display(unsigned char entries[]);
  void displaySmile();
  void displayLeft();
  void displayRight();
  void displayStart();
  void displayForward();
  void displayBackward();
  void displayStop();
  void clearDisplay();

  // Line tracking
  void enableLineTracking();
};
