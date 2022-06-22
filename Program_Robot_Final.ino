/*
* Firmware for the ”2WD Ultrasonic Motor Robot Car Kit”
*
* Stephen A. Edwards
*
* Hardware configuration :
* A pair of DC motors driven by an L298N H bridge motor driver
* An HC−SR04 ultrasonic range sensor mounted atop a small hobby servo
*/
#include <Servo.h>
Servo servo;
// Ultrasonic Module pins
const int trigPin = 13; // 10 microsecond high pulse causes chirp , wait 50 us
const int echoPin = 12; // Width of high pulse indicates distance
// Servo motor that aims ultrasonic sensor .
const int servoPin = 11; // PWM output for hobby servo
// Motor control pins : L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 4; // Right motor Direction 1
const int in4Pin = 2; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control
int sensorPin1 = 8; //declare variable for sensor input pin
int pinState1; //declare variable for sensor state
int sensorPin2 = 9;
int pinState2;
int backSensor = 10;
int backState;
unsigned long start1;
unsigned long start2;
unsigned long start3;
unsigned long start4;
unsigned long start5;
unsigned long start6;
unsigned long start7;
unsigned long start8;

enum Motor { LEFT, RIGHT };
// Set motor speed: 255 full ahead, −255 full reverse , 0 stop
void go( enum Motor m, int speed)
{
digitalWrite (m == LEFT ? in1Pin : in3Pin , speed > 0 ? HIGH : LOW );
digitalWrite (m == LEFT ? in2Pin : in4Pin , speed <= 0 ? HIGH : LOW );
analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed );
}

// Initial motor test :
// left motor forward then back
// right motor forward then back
void testMotors ()
{
static int speed[8] = {128,255,128,0,-128,-255,-128,0};
go(RIGHT, 0);
for (unsigned char i = 0 ; i < 8 ; i++)
go(LEFT, speed[i ]), delay (200);
for (unsigned char i = 0 ; i < 8 ; i++)
go(RIGHT, speed[i ]), delay (200);
}
// Read distance from the ultrasonic sensor , return distance in mm
//
// Speed of sound in dry air , 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ−6)
// 2d = p * 10ˆ−6 s * 343 m/s = p * 0.00343 m = p * 0.343 mm/us
unsigned int readDistance ()
{
digitalWrite ( trigPin , HIGH );
delayMicroseconds (10);
digitalWrite ( trigPin , LOW );
unsigned long period = pulseIn ( echoPin, HIGH );
return period * 343 / 2000;
}

#define NUM_ANGLES 7
unsigned char sensorAngle[NUM_ANGLES] = {30, 35, 85, 90, 95, 145, 150};
unsigned int distance [NUM_ANGLES];
// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed. This takes a reading , then
// sends the servo to the next angle. Call repeatedly once every 50 ms or so.
void readNextDistance ()
{
static unsigned char angleIndex = 0;
static signed char step = 1;
distance [angleIndex ] = readDistance ();
angleIndex += step ;
if (angleIndex == NUM_ANGLES -1) step = -1;
else if (angleIndex == 0) step = 1;
servo . write ( sensorAngle[angleIndex ] );
}
// Initial configuration
//
// Configure the input and output pins
// Center the servo
// Turn off the motors
// Test the motors
// Scan the surroundings once
//
void setup () {
pinMode(sensorPin2,INPUT_PULLUP);
pinMode(sensorPin1, INPUT_PULLUP); //set the sensor pin as an input with pullup resistor
pinMode(backSensor, INPUT_PULLUP);
Serial.begin(9600); //initialize serial communication
pinMode(trigPin , OUTPUT);
pinMode(echoPin, INPUT);
digitalWrite ( trigPin , LOW);
pinMode(enAPin, OUTPUT);
pinMode(in1Pin, OUTPUT);
pinMode(in2Pin, OUTPUT);
pinMode(in3Pin, OUTPUT);
pinMode(in4Pin, OUTPUT);
pinMode(enBPin, OUTPUT);
servo . attach ( servoPin );
servo . write (90);
go(LEFT, 0);
go(RIGHT, 0);
testMotors ();
// Scan the surroundings before starting
servo . write ( sensorAngle[0] );
delay (200);
for (unsigned char i = 0 ; i < NUM_ANGLES ; i ++)
readNextDistance (), delay (200);
}
// Main loop:
//
// Get the next sensor reading
// If anything appears to be too close , back up
// Otherwise, go forward
//

void loop () {

pinState1 = digitalRead(sensorPin1);
pinState2 = digitalRead(sensorPin2);
backState = digitalRead(backSensor);
/*
  if (pinState1) {
    Serial.println("No surface detected on left!");
  } else {
    Serial.println("Surface detected on left.");
  }
  if (pinState2) {
    Serial.println("No surface detected on right!");
  } else {
    Serial.println("Surface detected on right.");
  }
  if(backState) {
    Serial.println("NO SURFACE ON THE BACK!");
  } else {
    Serial.println("Back surface detected.");
  }
  if(tooClose) {
    Serial.println("Wall spotted ahead.");
  } else {
    Serial.println("No walls ahead.");
  }
*/
readNextDistance ();
// See if something is too close at any angle
unsigned char tooClose = 0;
for (unsigned char i = 0 ; i < NUM_ANGLES ; i++)
if ( distance [ i ] < 300)
tooClose = 1;

if(backState == 1) {
  go(LEFT,110);
  go(RIGHT,90);
} else {
if(tooClose == 1) {
    go(LEFT,-30);
    go(RIGHT,-100);
} else {
if(pinState1 == 1 && pinState2 == 1) {
  start3 = millis();
  while(millis()-start3 <= 800) {
    if(backState == 1) {
      go(LEFT,110);
      go(RIGHT,90);
    } if(backState == 0) {
    go(LEFT,-110);
    go(RIGHT,-100);
    }
  }
  int choice = random(0,2);
  if(choice == 0) {
    start4 = millis();
    while(millis()-start4 <= 500) {
      if(backState == 1) {
        go(LEFT,110);
        go(RIGHT,90);
      } if(backState == 0) {
      go(LEFT,-100);
      go(RIGHT,-30);
      }
    }
  } if (choice == 1) {
    start5 = millis();
    while(millis()-start5 <= 500) {
      if(backState == 1) {
        go(LEFT,110);
        go(RIGHT,90);
      } if(backState == 0) {
      go(LEFT,-30);
      go(RIGHT,-100);
      }
    }
  }
} if(pinState1 == 1 && pinState2 == 0) {
  start1 = millis();
  while(millis()-start1 <= 400) {
    if(backState == 1) {
      go(LEFT,110);
      go(RIGHT,90);
    } if(backState == 0) {
    go(LEFT,-110);
    go(RIGHT,-90);
    }
  }
  start6 = millis();
  while(millis()-start6 <= 900) {
    if(backState == 1) {
      go(LEFT,110);
      go(RIGHT,90);
    } if(backState == 0) {
    go(LEFT,-110);
    go(RIGHT,-30);
    }
  }
} if(pinState1 == 0 && pinState2 == 1) {
  start2 = millis();
  while(millis()-start2<=400) {
    if(backState == 1) {
      go(LEFT,110);
      go(RIGHT,90);
    } if(backState == 0) {
    go(LEFT,-110);
    go(RIGHT,-90);
    }
  }
  start7 = millis();
  while(millis()-start7<=900) {
    if(backState == 1) {
      go(LEFT,110);
      go(RIGHT,90);
    } if(backState == 0) {
    go(LEFT,-30);
    go(RIGHT,-90);
    }
  }
} if(pinState1 == 0 && pinState2 == 0) {
  go(LEFT,110);
  go(RIGHT,90);
}
}
}
delay(250);
}
