#include <AFMotor.h>
#include "QuickMedianLib.h"

//Defines pins for the connected sensor
const int trigPinX = A0;
const int echoPinX = A1;
const int trigPinY = A2;
const int echoPinY = A3;

//Array for storing shelf information
char shelfInfo[4]; 
char * pShelfInfo = shelfInfo;

//The ditances measured by the sensors
float distance1;
float distance2;

//Class ultra sonic sensor with 1D Kalman Filter
class UltraSonic {
  public:
  int trigPin; 
  int echoPin;
  float measurement;
  long duration;
  float x;
  float p;
  float r;
  float q;
  float k;
  float bias;

  UltraSonic(int trig, int echo, float prior, float adjust) {
    trigPin = trig; 
    echoPin = echo;
    x = prior;
    p = 1000.0;
    q = 1.0;
    r = 100.0;
    bias = adjust;
  }
  
  float Measure() {
     //Reset the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    measurement = duration*0.0343/2;

    p = p+q;
    k = p/(p+r);
    x = x + k*(measurement - x);
    p = (1-k)*p;
    return (x+bias);
  }
};

// Create class instances for sensors and motors 
UltraSonic sensorX(trigPinX, echoPinX, 10, 0.5);
UltraSonic sensorY(trigPinY, echoPinY, 5, -2);
AF_DCMotor motorX(1);
AF_DCMotor motorY(2);
AF_DCMotor motorZ(3);

//P-controller for the motors
void controller(float desired, UltraSonic &sensor, int K, AF_DCMotor &motor, int lowVel, float tol) {
  boolean reached = false;
  while(!reached) {
    float distance = sensor.Measure();
    //Serial.print("Distance: ");
    //Serial.println(distance);

    if( (desired-tol < distance) && (distance < desired+tol)) {
      motor.run(RELEASE);
      delay(10);
      distance = sensor.Measure();
      if( (desired-tol < distance) && (distance < desired+tol)) {
        reached = !reached;
      }
      
    } else {
        float error = distance - desired;
        int controlSignal = error*K;
        motor.setSpeed(max(min(abs(controlSignal),255),lowVel));

      if(controlSignal > 0) { 
        motor.run(FORWARD);
      } else {
        motor.run(BACKWARD);
      }
    }
    }
  }

//Extend fork
void extend(AF_DCMotor &motorZ) {
  motorZ.run(BACKWARD);
  delay(2500);
  motorZ.run(RELEASE);
}

//Retract fork
void retract(AF_DCMotor &motorZ) {
  motorZ.run(FORWARD);
  delay(2500);
  motorZ.run(RELEASE);
}

//Convert desired column to x-coordinate
float getX(char col) {
  float x = 0.0;
  if(col == 'A') {
    x = 28; 
  } else if(col == 'B') {
    x = 20.0;
  } else {
    x = 11.5;
  }
  return x;
};

//Convert desired row to y-coordinate
float getY(char row) {
  float y = 0.0;
  if(row == '1') {
    y = 2.0; 
  } else {
    y = 6.5;
  }
  return y;
};

String readInputString() {
  String inputString;
   while (Serial.available() == 0) {
    // Wait for User to Input Data
  }
  inputString = Serial.readString();
  return inputString;
}

void askUser(char *pShelfInfo) {
  Serial.println("Choose pick up shelf:");
  String pickUpString = readInputString();
  Serial.println(pickUpString);
  pShelfInfo[0] = pickUpString[0];
  pShelfInfo[1] = pickUpString[1];

  Serial.println("Choose drop off shelf:");
  String dropOffString = readInputString();
  Serial.println(dropOffString);
  pShelfInfo[2] = dropOffString[0];
  pShelfInfo[3] = dropOffString[1];
}

void setup() {
  // put your setup code here, to run once:
  motorX.setSpeed(255);
  motorX.run(RELEASE);
  motorY.setSpeed(255);
  motorY.run(RELEASE);
  motorZ.setSpeed(255);
  motorZ.run(RELEASE);
  pinMode(trigPinX, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinX, INPUT); // Sets the echoPin as an Input
  pinMode(trigPinY, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinY, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  //Wait for the Kalman filters to converge 
  Serial.println("Filters converging...");
  for(int i=0; i<1000; i++) {
    distance1 = sensorX.Measure();
    delay(10);
    distance2 = sensorY.Measure();
    delay(10);
  }
  Serial.println("Filters ready.");
  delay(1000);
}

void loop() {
Serial.println("Robot ready for a new task!");
//Get desired pick up and drop off point from user
askUser(pShelfInfo);

//Transform shelf names to coordinates in cm
float desiredX = getX(shelfInfo[0]);
float desiredY = getY(shelfInfo[1]);
float desiredX2 = getX(shelfInfo[2]);
float desiredY2 = getY(shelfInfo[3]);

//Pick up
controller(desiredX, sensorX, 30, motorX, 150, 0.1);
Serial.println("X position reached.");
delay(1000);
controller(desiredY, sensorY, -30, motorY, 100, 0.1);
Serial.println("Y position reached.");
delay(1000);
extend(motorZ);
delay(1000);
controller(desiredY+2.5, sensorY, -30, motorY, 100, 0.1);
delay(1000);
retract(motorZ);
Serial.println("Done.");

delay(2000);

//Leave
controller(desiredX2, sensorX, 30, motorX, 150, 0.1);
Serial.println("X position reached.");
delay(1000);
controller(desiredY2+1, sensorY, -30, motorY, 100, 0.1);
Serial.println("Y position reached.");
delay(1000);
extend(motorZ);
delay(1000);
controller(desiredY2, sensorY, -30, motorY, 100, 0.1);
delay(1000);
retract(motorZ);
Serial.println("Done.");
}
