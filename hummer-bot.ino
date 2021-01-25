#include <Easyuino.h>
#include <RGBLed.h>

using Easyuino::DistanceMeter;

#define IN1_PIN 6
#define IN2_PIN 10
#define IN3_PIN 5
#define IN4_PIN 9
#define TRIGGER_PIN 3
#define ECHO_PIN 3
#define RGB_PIN 2
#define LEFT_PIN 12
#define RIGHT_PIN A5

#define FORWARD 1
#define BRAKE 0
#define LEFT 2
#define RIGHT 3
#define BACKWARD 4
#define TURN_AROUND 5

#define STOP_DISTANCE 17.0

float prevDistance = 0.0;
int time;
int readLeft, readRight;

DistanceMeter distanceMeter(TRIGGER_PIN, ECHO_PIN);
RGBLed mRgb(RGB_PIN, 6);

void setup() {
	Serial.begin(0451);
	distanceMeter.begin();

    pinMode(LEFT_PIN, INPUT);
    pinMode(RIGHT_PIN, INPUT);

    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
}

void loop() {
    float distance = updateDistance();
    checkMovementStuck(distance);
    ultrasonicObstacleAvoidance(distance);
    infradedObstacleAvoidance(updateInfradedAvoidance());
}

void checkMovementStuck(float distance) {
    if(millis() >= time + 1000) {
        time = millis();
        float distanceDiff = abs(distance - prevDistance);
        if (distanceDiff >= 0 && distanceDiff <= 2.0) {
            prevDistance = distance;
            turnAround();
        } else {
            prevDistance = distance;
        }
    }
}

void ultrasonicObstacleAvoidance(float distance) {
    updateStopSignal(distance);
    if (distance <= STOP_DISTANCE) {
        // turnAround();
        brake();
        delay(500);
        int direction = updateInfradedAvoidance();
        infradedObstacleAvoidanceAfterUltrasonicStop(direction);
    } else {
        moveForward();
    }
}

void infradedObstacleAvoidance(int direction) {
    switch (direction)
    {
    case LEFT:
        turnRight();
        delay(25);
        break;
    case RIGHT:
        turnLeft();
        delay(25);
        break;
    case TURN_AROUND:
        turnAround();
        break;
    case FORWARD:
    default:
        moveForward();
        break;
    }
}

void infradedObstacleAvoidanceAfterUltrasonicStop(int direction) {
    switch (direction)
    {
    case LEFT:
        turnRight();
        delay(500);
        break;
    case RIGHT:
        turnLeft();
        delay(500);
        break;
    case FORWARD:
    case TURN_AROUND:
    default:
        turnAround();
        break;
    }
}

float updateDistance() {
    distanceMeter.updateDistance();
    float distanceCentimeters = distanceMeter.getDistanceCentimeters();
    return distanceCentimeters;
}

void updateStopSignal(float distance) {
     if (distance <= STOP_DISTANCE) {
        mRgb.setColor(0,255,0);
        mRgb.show();
    } else {
        mRgb.setColor(0,0,0);
        mRgb.show();
    }
}

int updateInfradedAvoidance() {
  readLeft = digitalRead(LEFT_PIN);
  readRight = digitalRead(RIGHT_PIN);

  if (readLeft == 0 && readRight == 1) {
      return LEFT;
  } else if (readLeft == 1 && readRight == 0) {
      return RIGHT;
  } else if (readLeft == 0 && readRight == 0) {
      return TURN_AROUND;
  } else if (readLeft == 1 && readRight == 1) {
      return FORWARD;
  }
}

void moveForward() {
    analogWrite(IN1_PIN, 100);
    analogWrite(IN2_PIN, LOW);
    analogWrite(IN3_PIN, LOW);
    analogWrite(IN4_PIN, 100);
}

void moveBackWard() {
    analogWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, 100);
    analogWrite(IN3_PIN, 100);
    analogWrite(IN4_PIN, LOW);
}

void brake() {
    analogWrite(IN1_PIN, HIGH);
    analogWrite(IN2_PIN, HIGH);
    analogWrite(IN3_PIN, HIGH);
    analogWrite(IN4_PIN, HIGH);
}

void turnLeft() {
    analogWrite(IN1_PIN, 150);
    analogWrite(IN2_PIN, LOW);
    analogWrite(IN3_PIN, 150);
    analogWrite(IN4_PIN, LOW);
}

void turnRight() {
    analogWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, 150);
    analogWrite(IN3_PIN, LOW);
    analogWrite(IN4_PIN, 150);
}

void turnAround() {
    brake();
    delay(500);
    moveBackWard();
    delay(500);
    turnLeft();
    delay(900);
}