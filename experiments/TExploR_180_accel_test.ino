// New code written for testing TExploR acceleration for 180 degree half arc in ARL

#include <AccelStepper.h>
#define motorInterfaceType 1 // Define motor interface type

const int dirPin = 2; // Define direction pin
const int stepPin = 3; // Define step pin
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin); // Creates an instance

void setup() {
  myStepper.setMaxSpeed(1600); // Set max. speed
  myStepper.setAcceleration(800); // Set acceleration factor
  myStepper.setSpeed(800); // Initialize speed
  myStepper.moveTo(1200); // Initialize target pos. 800 is ~90 degrees, 1600 is ~180 degrees
  // If performing tip-over motion, comment above target position 
  // and uncomment below target position
  //myStepper.moveTo(400);
}

void loop() {
  // Uncomment 3 lines below for rolling locomotion
  if (myStepper.distanceToGo() == 0) { // Change direction once the motor reaches target position
    myStepper.moveTo(-myStepper.currentPosition());
  }
  myStepper.run(); // Move the motor 180 degrees
  //myStepper.runSpeed(); // Uncomment for constant movement and don't use myStepper.run()
}
