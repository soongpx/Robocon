//Locomotion
#define wheel1 2 //upper left
#define wheel2 3 //upper right
#define wheel3 4 //lower left
#define wheel4 5 //lower right
#define dir_1 0 //upper left direction
#define dir_2 1 //upper right direction
#define dir_3 2 //lower left direction
#define dir_4 3 //lower right direction
#define motor_speed 120

//PS4
#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
//USB Set UP
USB Usb;
USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

// input
int input_signal = 0;

enum 
{
  NoPress,
  Up,
  Down,
  Left,
  Right,
  Square,
  Circle,
  Cross,
  Triangle
} button;

enum 
{
  input,
  process,
  output
} progress;

enum
{
  initialisation,
  start,
  rectanglemove
} state;

//Locomotion
void Forward(int pwm)
{
    PORTA |= (1 << dir_1);
    PORTA &= ~(1 << dir_2);
    PORTA |= (1 << dir_3);
    PORTA &= ~(1 << dir_4);
      
    analogWrite(wheel1, pwm);
    analogWrite(wheel2, pwm);
    analogWrite(wheel3, pwm);
    analogWrite(wheel4, pwm);
}

void Backward(int  pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA |= (1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA |= (1 << dir_4);

  analogWrite(wheel1, pwm);
  analogWrite(wheel2, pwm);
  analogWrite(wheel3, pwm);
  analogWrite(wheel4, pwm);
}

void Leftward(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA |= (1 << dir_3);
  PORTA |= (1 << dir_4);

  analogWrite(wheel1, pwm);
  analogWrite(wheel2, pwm);
  analogWrite(wheel3, pwm);
  analogWrite(wheel4, pwm);

}

void Rightward(int pwm)
{
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);

  analogWrite(wheel1, pwm);
  analogWrite(wheel2, pwm);
  analogWrite(wheel3, pwm);
  analogWrite(wheel4, pwm);
}

void MotorStopping()
{
  analogWrite(wheel1, 0);
  analogWrite(wheel2, 0);
  analogWrite(wheel3, 0);
  analogWrite(wheel4, 0);
}

//input process
void dpadmovement()
{
  if (PS4.getButtonPress(UP)) { 
    input_signal += 1;
    button = Up;
    Serial.println("Forward");
  }
  if (PS4.getButtonPress(RIGHT)) {
    input_signal += 2;

    Serial.println("Right");
  }
  if (PS4.getButtonPress(DOWN)) {
    input_signal += 4;
    Serial.println("Backward");
  }
  if (PS4.getButtonPress(LEFT)) { 
    input_signal += 8;
    Serial.println("Left");
  }
  if (PS4.getButtonPress(L1)){ 
  }
  if (PS4.getButtonPress(R1)){ 
  }
  if (PS4.getButtonPress(SQUARE)){
  }
  if (PS4.getButtonPress(CIRCLE)){ 
  }
  if (PS4.getButtonPress(TRIANGLE)){ 
  }
  if (PS4.getButtonPress(CROSS)){ 
  } 
  
}

void PS4Setup(){
  Serial.begin(115200);

  #if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
  #endif
    if (Usb.Init() == -1)
    {
      Serial.print(F("\r\nOSC did not start"));
      while (1); // Halt
    }
    Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

void setup() {
  PS4Setup();
}

void loop() {
  switch (state){
    case start:
      switch(progress){
        case input:
          Usb.Task();
          if (PS4.connected()) {
            dpadmovement();
          }
        case process:
          switch (input_signal){
            case 0:
              MotorStopping();
              break;
            case 1:
              Forward(motor_speed);
              break;
            case 2:
              Backward(motor_speed);
              break;
            case 4:
              Leftward(motor_speed);
              break;
            case 8:
              Rightward(motor_speed);
              break;
            default:
              MotorStopping();
          }
      }
      case rectangle:
        
  }

}
