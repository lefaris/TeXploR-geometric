// TExploR internal mass travels to angle specified
// Input: degrees to travel to ex. 45
// Output: degree input is converted to arc length and internal mass travels to that point

#include <AccelStepper.h>
#define motorInterfaceType 1 // Define motor interface type
#define PI 3.1415926535897932384626433832795

int degree = 90; // User input angle
int arc_length = ((degree*PI)/180)*172*5; // Calculate the arc length to traverse to
const int dirPin = 2; // Define direction pin
const int stepPin = 3; // Define step pin

AccelStepper myStepper(motorInterfaceType, stepPin, dirPin); // Creates an instance



void setup() {
  myStepper.setMaxSpeed(200); // Set max. speed
  //myStepper.setAcceleration(50); // Set acceleration factor
  myStepper.setSpeed(200); // Initialize speed
  //myStepper.moveTo(arc_length); // Initialize target pos. 800 is ~90 degrees, 1600 is ~180 degrees
  // If performing tip-over motion, comment above target position 
  // and uncomment below target position
  // myStepper.moveTo(arc_length);
  delay(6000); // Allow time to start recording data
}

void loop() {
  myStepper.setMaxSpeed(200); // Set max. speed
  // Uncomment 3 lines below for rolling locomotion
  //if (myStepper.distanceToGo() == 0) { // Change direction once the motor reaches target position
  //  myStepper.moveTo(-myStepper.currentPosition());
  //}
  myStepper.moveTo(-arc_length);
  while (myStepper.distanceToGo() != 0) {
    myStepper.moveTo(-arc_length);
    myStepper.setSpeed(200);
    myStepper.runSpeedToPosition();
  }
  //myStepper.run(); // Move the motor 180 degrees
  //myStepper.runSpeed(); // Uncomment for constant movement and don't use myStepper.run()
}
