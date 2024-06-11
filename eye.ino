#include <Servo.h>

//define name of the servo motors
Servo upDownServo;
Servo rightLeftServo;

//define position name and value
#define left 60
#define right 120
#define middle 90
#define closed 60
#define fullOpen 160
#define halfOpen 120

#define waitTime 750
#define SMOOTHING_FACTOR 0.05
#define UPDATE_INTERVAL 10

float upDownServoPosition;
float rightLeftServoPosition;
float upDownServoTarget;
float rightLeftServoTarget;

void setup() {
  //define pin numbers of the servo motors
  upDownServo.attach(45);
  rightLeftServo.attach(47);

  //starting position of the servo motors
  delay(10);
  upDownServo.write(closed);
  rightLeftServo.write(middle);

  // Initialize positions
  upDownServoPosition = closed;
  rightLeftServoPosition = middle;
  upDownServoTarget = closed;
  rightLeftServoTarget = middle;
}

void smoothServo(Servo &servo, float &currentPosition, float targetPosition) {
  currentPosition = (targetPosition * SMOOTHING_FACTOR) + (currentPosition * (1 - SMOOTHING_FACTOR));
  servo.write(static_cast<int>(currentPosition));
}

void loop() {
  // Example sequence of movements with smoothing
  upDownServoTarget = halfOpen;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = right;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = left;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = middle;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = closed;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = fullOpen;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = closed;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = fullOpen;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = right;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = left;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = middle;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = halfOpen;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = right;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = left;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = middle;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = fullOpen;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = halfOpen;
  delay(waitTime);
  smoothMovement();

  upDownServoTarget = fullOpen;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = right;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = left;
  delay(waitTime);
  smoothMovement();

  rightLeftServoTarget = middle;
  delay(waitTime);
  smoothMovement();
}

void smoothMovement() {
  unsigned long startTime = millis();
  while (millis() - startTime < waitTime) {
    smoothServo(upDownServo, upDownServoPosition, upDownServoTarget);
    smoothServo(rightLeftServo, rightLeftServoPosition, rightLeftServoTarget);
    delay(UPDATE_INTERVAL);
  }
}
