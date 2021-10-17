#include <ESC.h>
#include "MedianFilter.h"
#include <NewPing.h>
#include <Encoder.h>

//For MaxSonar
#define USPin1 A0  //front
#define USPin2 A2    //back

//For Ultrasonics
#define TRIGGER_PIN1  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     2  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // Front
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // Back

Encoder encoderMain(18, 19);
//There are the intterupt enabled pins
//Arduino Mega  2, 3, 18, 19, 20, 21

int oESC;

int i = 0;

bool hardwareTest = false;

int speedToTravelLeft = 60; //First
int speedToTravelRight = 60;

int dataLog[300];


//PWM Out, Min, Max, Arming
ESC myESC1 (8, 1000, 2000, 1500);
ESC myESC2 (9, 1000, 2000, 1500);

float getUltrasonicDistance(bool isFront, bool isSonar = false);

int motorSpeed = 1500;
int stopSpeed = 1500;
double stopDistance = 30;
int speedRange = 130;  //+- from 1500
int patrollingSpeed = 80;
int speedSafety = 50;

bool dir_forward = true;
unsigned int lastInput = 0;
MedianFilter frontDist;
MedianFilter backDist;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.setTimeout(10000);
  Serial.print("Arming Motors");
  myESC1.arm();
  myESC2.arm();
  delay(8000);
  Serial.println(" - Done");

  pinMode(5, INPUT); 

  if (hardwareTest) {
    Serial.print("Motor 1 @ 1400");
    myESC1.speed(1400);
    delay(1500);
    Serial.println(" - Stopped");
    myESC1.speed(1500);
    delay(2500);
    Serial.print("Motor 1 @ 1600");
    myESC1.speed(1600);
    delay(1500);
    Serial.println(" - Stopped");
    myESC1.speed(1500);
    delay(2500);

    Serial.print("Motor 2 @ 1400");
    myESC2.speed(1400);
    delay(1500);
    Serial.println(" - Stopped");
    myESC2.speed(1500);
    delay(2500);
    Serial.print("Motor 2 @ 1600");
    myESC2.speed(1600);
    delay(1500);
    Serial.println(" - Stopped");
    myESC2.speed(1500);

    Serial.println("Motor Check Complete!");
  }

}

void loop() {
  
  //      Serial.println(encoderMain.read());
  //      delay(20);

  //Stop to left
  for (oESC = 1500; oESC <= 1500 + speedToTravelLeft; oESC += 1) { // goes from 1000 microseconds to 2000 microseconds
    myESC1.speed(oESC);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(oESC);
    delay(20);                                            // waits 10ms for the ESC to reach speed
  }

  float sonar1_reading = sonar1.ping_cm();
  float last_reading1 = 9999;
  while (sonar1_reading > 90 || sonar1_reading == 0 || last_reading1 > 90 || last_reading1 == 0) {
//    Serial.println(sonar1_reading);
    last_reading1 = sonar1_reading;
    sonar1_reading = sonar1.ping_cm();
    delay(50);
  }
  Serial.println(sonar1_reading);


  //Left to stop
  for (oESC = 1500 + speedToTravelLeft; oESC >= 1500; oESC -= 1) { // goes from 1000 microseconds to 2000 microseconds
    myESC1.speed(oESC);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(oESC);
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(500);
  dataLog[i] = encoderMain.read();
  i++;
  if (i % 10 == 1 || i % 10 == 0) {
    //Data dump
    myESC1.stop();
    delay(1000);
    myESC1.arm();
    while(digitalRead(5) == 0){};
    Serial.println("Uploading now");
    for (int i = 0; i < 299; i = i + 1) {
      Serial.println(dataLog[i]);
    }
    delay(3000);
  }

  //Stop to right
  for (oESC = 1500; oESC >= 1500 - speedToTravelRight; oESC -= 1) { // goes from 1000 microseconds to 2000 microseconds
    myESC1.speed(oESC);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(oESC);
    delay(20);                                            // waits 10ms for the ESC to reach speed
  }

  float sonar2_reading = sonar2.ping_cm();
  float last_reading2 = 9999;
  while (sonar2_reading > 60 || sonar2_reading == 0 || last_reading2 > 60 || last_reading2 == 0) {
//    Serial.println(sonar2_reading);
    last_reading2 = sonar2_reading;
    sonar2_reading = sonar2.ping_cm();
    delay(50);
  }
  Serial.println(sonar2_reading);


  //Right to stop
  for (oESC = 1500 - speedToTravelRight; oESC <= 1500; oESC += 1) { // goes from 1000 microseconds to 2000 microseconds
    myESC1.speed(oESC);                                    // tell ESC to go to the oESC speed value
    myESC2.speed(oESC);
    delay(10);                                            // waits 10ms for the ESC to reach speed
  }

  delay(500);
  dataLog[i] = encoderMain.read();
  i++;

}
