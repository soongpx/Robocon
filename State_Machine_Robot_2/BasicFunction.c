#include "stdint.h"
#include "ProgramConstant.h"
#include "arduino.h"

//Locomotion
uint8_t Basic_Function_Counter = 0;
extern uint8_t OperatingState;

void MoveFront(int pwm)
{
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_3);
  SetPWM(pwm);
  Basic_Function_Counter += 1;
  if(OperatingState = 0xFF)  Basic_Function_Counter = 0;
  SerialPrintStringln("Hahaha");
}

void MoveBack(int  pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_3);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_4);
  SetPWM(pwm);
  Basic_Function_Counter += 1; 
}

void Leftward(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_3);
  PORTA |= (1 << dir_4);
  SetPWM(pwm);
  Basic_Function_Counter += 1; 
}

void MoveRight(int pwm)
{
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_2);
  SetPWM(pwm);
  Basic_Function_Counter += 1; 
}


void SetPWM(int PWM_Value)
{
  analogWrite(wheel1, PWM_Value);
  analogWrite(wheel2, PWM_Value);
  analogWrite(wheel3, PWM_Value);
  analogWrite(wheel4, PWM_Value);  
}
