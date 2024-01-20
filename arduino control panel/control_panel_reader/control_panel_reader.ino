/*
 *  Project     Chassis-5516-2024
 *  @author     Sam
 *  @link       https://github.com/CCSC-Robotics-club/FRC2024-Chassis
 *  @license    MIT
 *
 * a simple arduino program to read a customized control panel for robot
 * it simulates Xbox inputs 
 */

#include <XInput.h>

#define refreshRate 1000
#define keyAmount 4

int keyPins[keyAmount];
int keyOutputs[keyAmount];

void setup() {
  keyBindings();
  for (int key = 0; key < keyAmount; key++)  
    pinMode(keyPins[key], INPUT);
}

void keyBindings() {
  keyPins[0] = 2;
  keyOutputs[0] = BUTTON_A;

  keyPins[1] = 3;
  keyOutputs[1] = BUTTON_B;

  keyPins[2] = 4;
  keyOutputs[0] = BUTTON_X;

  keyPins[3] = 5;
  keyOutputs[0] = BUTTON_Y;
}

void loop() {
  for (int key = 0; key < keyAmount; key++) {
    if (digitalRead(keyPins[key]) == 1) XInput.press(keyOutputs[key]);
    else XInput.release(keyOutputs[key]);
  }
  delay(1000.0f / refreshRate);
}
