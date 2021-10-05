#include <ESC.h>
#include "MedianFilter.h"
#include <NewPing.h>

//For MaxSonar
#define USPin1 A0  //front
#define USPin2 A2    //back

//For Ultrasonics
#define TRIGGER_PIN1  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  9  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     10  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // Front
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // Back

ESC myESC1 (8, 1000, 2000, 2000);
ESC myESC2 (9, 1000, 2000, 2000);

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
   myESC1.arm();
   myESC2.arm();
   myESC1.speed(1500);
   myESC2.speed(1500);
   delay(5000);

   Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  frontDist.push(getUltrasonicDistance(true) + 8 ); //front == true// front Dist tends to read high
  backDist.push( getUltrasonicDistance(false) + 6); //back dist tends to read low

  //UNCOMMENT THIS IF USING 2 CAN ULTRASONICS
  //frontDist.push(getUltrasonicDistance(true, true)); //front == true// front Dist tends to read high
  //backDist.push( getUltrasonicDistance(false, true)); //back dist tends to read low

  //debug maxSonar
  Serial.print(getUltrasonicDistance(true));
  Serial.println(" ");
  Serial.println(getUltrasonicDistance(false));
  //debug ultrasonics (2 can ver)
//  Serial.print(getUltrasonicDistance(true, true));
//  Serial.println(" ");
//  Serial.println(getUltrasonicDistance(false, true));
  

  
    if (frontDist.read() <= stopDistance && backDist.read() <= stopDistance ) {
      motorSpeed = stopSpeed;
    } 
    
    else {
      if ( frontDist.read() <= stopDistance ) {
        //if too close
        //Serial.print("BACKWARD");
        dir_forward = false;
      } 
       else if ( backDist.read() <= stopDistance) {
        //Serial.print("FORWARD");
        dir_forward = true;
      } 
      
      if (dir_forward) {
        motorSpeed = stopSpeed + patrollingSpeed;
      } 
      
      else if (!dir_forward) {
        motorSpeed = stopSpeed - patrollingSpeed;
      }
    }
    lastInput = millis();

    myESC1.speed(motorSpeed);
    myESC2.speed(motorSpeed);
}

/*
  Get the Ultrasnonic reading and perform conversion from analog to centimeter
  Reading from USPin1 if isFront is true meaning the robot is going forward (away from the charging station)
  Reading from USPin2 if isFront is false -> going backward (toward the charging station)
*/
float getUltrasonicDistance(bool isFront, bool isSonar = false) // returns distance in centimeters
{
  int distance = 0;
  if (not isSonar)
  {
  if (isFront) distance = analogRead(USPin1);
  else distance = analogRead(USPin2);
  return (distance / 1024.0) * 512 * 2.54;
  }

  else //for Sonar
  {
    if (isFront) return sonar1.ping_cm();
    else return sonar2.ping_cm();
    }
}
