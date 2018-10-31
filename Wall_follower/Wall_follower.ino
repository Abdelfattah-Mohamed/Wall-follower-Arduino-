#include <Servo.h>

Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position

byte tC = 0;    //trig of front center sensor.
byte eC = 6;    //echo of front center sensor.
byte tC1 = 1;   //trig of front right sensor.
byte eC1 = 7;   //echo of front right sensor.
byte tL = 2;    //trig of left sensor.
byte eL = 8;    //echo of left sensor.
byte tR = 12;   //trig of right sensor.
byte eR = 9;    //echo of right sensor.
byte rightMotorR = 4;   //direction of right motor.
byte rightMotorS = 10;   // speed of right motor.
byte leftMotorR = 5;    // direction of left motor.
byte leftMotorS = 11;   // speed of left motor.
byte counterOfBoxes = 13;   // blinked LED.

float distance0 = 0;  // distance of ultrasonic front center.
float distance1 = 0;  // distance of ultrasonic front right.
float distance2 = 0;  // distance of ultrasonic left.
float distance3 = 0;  // distance of ultrasonic right.

int blink_counter = 0;  // boxes have been sweeped.

int flag = 0;   // flag of end point.
void setup() {
  // put your setup code here, to run once:
  pinMode(tC, OUTPUT);    //initialize pin of trig of front center ultrasonic sensor is output.
  pinMode(tC1, OUTPUT);   //initialize pin of trig of front right ultrasonic sensor is output.
  pinMode(tL, OUTPUT);    //initialize pin of trig of left ultrasonic sensor is output.
  pinMode(tR, OUTPUT);    //initialize pin of trig of right ultrasonic sensor is output.

  pinMode(eC, INPUT);   //initialize pin of echo of front center ultrasonic sensor is input.
  pinMode(eC1, INPUT);   //initialize pin of echo of front right ultrasonic sensor is input.
  pinMode(eL, INPUT);   //initialize pin of echo of left ultrasonic sensor is input.
  pinMode(eR, INPUT);   //initialize pin of echo of left ultrasonic sensor is input.

  pinMode(rightMotorR, OUTPUT); //initialize pin of rightMotor direction is output.
  pinMode(rightMotorS, OUTPUT); //initialize pin of rightMotorR speed is output.
  pinMode(leftMotorR, OUTPUT);  //initialize pin of left motor direction is output.
  pinMode(leftMotorS, OUTPUT);  //initialize pin of lefr motor speed is output.


  myservo.attach(3); //initialize pin of servo motor.
}

void loop() {
  digitalWrite(tC, LOW);    //set trig pins to low.
  delayMicroseconds(3);
  digitalWrite(tC1, LOW);
  delayMicroseconds(3);
  digitalWrite(tL, LOW);
  delayMicroseconds(3);
  digitalWrite(tR, LOW);
  delayMicroseconds(3);

  digitalWrite(tC, HIGH);   // read distance on front center sensor.
  delayMicroseconds(10);
  digitalWrite(tC, LOW);
  distance0 = float(pulseIn(eC, HIGH)) / 58.8;

  if (distance0 > 3) {    // move the robot if there are nothing front of it.
    digitalWrite(rightMotorR, HIGH);
    analogWrite(rightMotorS, 255);
    digitalWrite(leftMotorR, HIGH);
    analogWrite(leftMotorS, 255);
  } else {
    stop_motor();   //stop motor because it found some things front of it.
    delay(10);
    check_front();  // to check front of robot is wall or box.
  }
  if (flag == 1) {
    stop_motor(); // stop motor at end point.
    return;
  }
}

void check_front() {
  digitalWrite(tC, HIGH);
  delayMicroseconds(10);
  digitalWrite(tC, LOW);
  distance0 = float(pulseIn(eC, HIGH)) / 58.8;

  digitalWrite(tC1, HIGH);
  delayMicroseconds(10);
  digitalWrite(tC1, LOW);
  distance1 = float(pulseIn(eC1, HIGH)) / 58.8;

  if (distance0 == distance2) {
    check_sides();    // to check two sides of robot is wall or box.
  } else {
    blink_led();    // blink LED.
    servo_motor();  // turn on servo motor.
  }
}

void check_sides() {
  digitalWrite(tL, HIGH);   // measure distance of left.
  delayMicroseconds(10);
  digitalWrite(tL, LOW);
  distance2 = float(pulseIn(eL, HIGH)) / 58.8;

  digitalWrite(tR, HIGH);   // measure distance of right.
  delayMicroseconds(10);
  digitalWrite(tR, LOW);
  distance3 = float(pulseIn(eR, HIGH)) / 58.8;

  if (distance2 == distance3) {   // check the end.
    /***********************************************************
                            END.
    **************************************************************/
    flag = 1;
    return;
  } else {
    if (distance2 > distance3) {  //check turn left.
      turn_left();    // turn left.
    } else {
      turn_right();   // turn right.
    }
  }
}

void blink_led() {
  blink_counter++;
  digitalWrite(counterOfBoxes, LOW);
  delay(100);
  for (int i = 0; i < blink_counter; i++) { // blink led number of moved boxes.
    digitalWrite(counterOfBoxes, HIGH);   // give light 0.5sec.
    delay(500);
    digitalWrite(counterOfBoxes, LOW);  // turn off 0.5sec.
    delay(500);
  }
}
void servo_motor() {
  for (pos = 0; pos <= 180; pos += 2) {   // move servo motor in step of 2 degree to move box.
    myservo.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 10) {  // move servo motor back in step of 10 degree to return in initialize value(degree = 0).
    myservo.write(pos);
    delay(15);
  }
}



void stop_motor() {   // stop motor to tack action.
  digitalWrite(rightMotorR, LOW);
  analogWrite(rightMotorS, 0);
  digitalWrite(leftMotorR, LOW);
  analogWrite(leftMotorS, 0);
}

void turn_right() {   // to turn robot right.
  digitalWrite(rightMotorR, LOW); // turn off right one.
  analogWrite(rightMotorS, 0);
  digitalWrite(leftMotorR, HIGH); // turn on left one.
  analogWrite(leftMotorS, 255);
  delay(600);                       //that's delay for make 90 degree turn.  // I don't know required time so i suppose that's time.
  digitalWrite(rightMotorR, LOW); // to stop robot to take action.
  analogWrite(rightMotorS, 0);
  digitalWrite(leftMotorR, LOW);
  analogWrite(leftMotorS, 0);
}

void turn_left() {    // to turn robot left.
  digitalWrite(rightMotorR, HIGH);  // turn on right one.
  analogWrite(rightMotorS, 0);
  digitalWrite(leftMotorR, LOW);    // turn off left one.
  analogWrite(leftMotorS, 255);
  delay(600);                       //that's delay for make 90 degree turn.  // I don't know required time so i suppose that's time.
  digitalWrite(rightMotorR, LOW); // to stop robot to take action.
  analogWrite(rightMotorS, 0);
  digitalWrite(leftMotorR, LOW);
  analogWrite(leftMotorS, 0);
}
