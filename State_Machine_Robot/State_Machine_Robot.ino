
//PS4
#include <PS4BT.h>
#include <usbhub.h>
#include <TimedAction.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include "ProgramConstant.h"
#include "Variable.h"
//USB Set UP



void DebugMessageTaskCode()
{
  if(DebugMessage != "")
  {
    Serial.print(DebugMessage);
    DebugMessage = "";          // Clear away the message so that serial terminal wont full of repeated useless message
  }
}


void PS4_Repeat_Init_Code()
{
  if (Usb.Init() == -1)
  {
    USB_Detected = false;
    DebugMessage = F("USB host init fail");
    PS4_Repeat_Init_Task.enable();          // It is ok to keep enable, it is just a flag only, not starting another thread
  }
  else
  {
    USB_Detected = true;
    DebugMessage =F("\r\nPS4 Bluetooth Library Started");
    PS4_Repeat_Init_Task.disable();         // Disable PS4_Repeat_Init_Task, it means PS4_Repeat_Init_Task.check() will basically do nothing
  }
}
void InputTaskCode()
{
  UP_Pressed        = PS4.getButtonPress(UP)?        true:UP_Pressed;
  RIGHT_Pressed     = PS4.getButtonPress(RIGHT)?     true:RIGHT_Pressed;
  DOWN_Pressed      = PS4.getButtonPress(DOWN)?      true:DOWN_Pressed;
  LEFT_Pressed      = PS4.getButtonPress(LEFT)?      true:LEFT_Pressed;
  L1_Pressed        = PS4.getButtonPress(L1)?        true:L1_Pressed;
  SQUARE_Pressed    = PS4.getButtonPress(SQUARE)?    true:SQUARE_Pressed;
  CIRCLE_Pressed    = PS4.getButtonPress(CIRCLE)?    true:CIRCLE_Pressed;
  TRIANGLE_Pressed  = PS4.getButtonPress(TRIANGLE)?  true:TRIANGLE_Pressed;
  CROSS_Pressed     = PS4.getButtonPress(CROSS)?     true:CROSS_Pressed;
  START_Pressed   =   PS4.getButtonPress(OPTIONS)?   true:START_Pressed;
  // Scan IMU Reading from serial
  // Scan ON/OFF Switch or Sensor Reading
}

void ClearButtonStatus()
{
  UP_Pressed        = false;
  RIGHT_Pressed     = false;
  DOWN_Pressed      = false;
  LEFT_Pressed      = false;
  L1_Pressed        = false;
  SQUARE_Pressed    = false;
  CIRCLE_Pressed    = false;
  TRIANGLE_Pressed  = false;
  CROSS_Pressed     = false;
  START_Pressed   =   false;  
}
void ProcessTaskCode()
{
  switch(OperatingState)
  {
    case USB_Detect_Hold:
        if (USB_Detected = true) OperatingState = WaitStart; // Set First State to wait user press Start button
        else DebugMessage = F("Detecting PS4 from USB Host ");
      break;
    case WaitStart:
      if(START_Pressed)
        OperatingState = CheckActionToDo ;
        ClearButtonStatus(); // May be "START_Pressed = false" is more relevan
      break;
        
    case CheckActionToDo:
      // Please Take note that the following code process Button in priority manner
      if(CIRCLE_Pressed)
      {
        OperatingState = CircleMove;
        Serial.println("User Select to Move In Circle");
      }
      else if (SQUARE_Pressed)
      {
        OperatingState = RectangleMove;
        Serial.println("User Select to Move In Rectangle");
      }
      else if (UP_Pressed)
      {
        OperatingState = Move_Forward;
        Serial.println("User Select to Move Forward");
      }
      else if (DOWN_Pressed)
      {
        OperatingState = Move_Backward;
        Serial.println("User Select to Move Backward");
      }
      else if (LEFT_Pressed)
      {
        OperatingState = Move_Left;
        Serial.println("User Select to Move Left");
      }
      else if (RIGHT_Pressed)
      {
        OperatingState = Move_Right;
        Serial.println("User Select to Move Right");
      }
      else 
      {
        ClearButtonStatus(); // In Action Checking Cycle, disregard some functional button
        Serial.println("User Select Non Action Button, Do nothing");
      }
      break;
      
    case CircleMove:
      if(CROSS_Pressed) 
      {
        // Exit Circle Move Operation 
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case RectangleMove:
      if(CROSS_Pressed) 
      {
        // Exit Rectangle Move Operation 
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Forward:
      if(CROSS_Pressed) 
      {
        // Exit Forward Move Operation 
        OperatingState = CheckActionToDo;
      }
      ForwardSpeed = 100;   // Can actually perform ramping by using counter
      BackwardSpeed =-1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Backward:
      if(CROSS_Pressed) 
      {
        // Exit Backward Move Operation 
        OperatingState = CheckActionToDo;
      }
      BackwardSpeed = 100;   // Can actually perform ramping by using counter
      ForwardSpeed =-1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Left:
      if(CROSS_Pressed) 
      {
        // Exit Left Move Operation 
        OperatingState = CheckActionToDo;
      }      
      LeftSpeed = 100;   // Can actually perform ramping by using counter
      RightSpeed =-1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Right:
      if(CROSS_Pressed) 
      {
        // Exit Right Move Operation 
        OperatingState = CheckActionToDo;
      }
      RightSpeed = 100;   // Can actually perform ramping by using counter
      LeftSpeed =-1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    default:                    // For in case something go wrong, force user press Start Button again
      OperatingState = WaitStart;
      Serial.println("Warning : Serious Error Occur, Entering Unexpected state");
      ClearButtonStatus();      // Check Carefully if this is necessary
  }
}

void OutputTaskCode()
{
  if(LeftSpeed > 0) Leftward(LeftSpeed);
  else if(RightSpeed > 0) MoveRight(RightSpeed); 
  else if(ForwardSpeed > 0) MoveFront(ForwardSpeed);
  else if( BackwardSpeed > 0) MoveBack(BackwardSpeed);
}

void SetPWM(int PWM_Value)
{
  analogWrite(wheel1, PWM_Value);
  analogWrite(wheel2, PWM_Value);
  analogWrite(wheel3, PWM_Value);
  analogWrite(wheel4, PWM_Value);  
}

//Locomotion
void MoveFront(int pwm)
{
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_3);
  SetPWM(pwm);
}

void MoveBack(int  pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_3);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_4);
  SetPWM(pwm);
}

void Leftward(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_3);
  PORTA |= (1 << dir_4);
  SetPWM(pwm);
}

void MoveRight(int pwm)
{
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_2);
  SetPWM(pwm);
}

void MotorStopping()
{
  SetPWM(0);
}

void setup() 
{
  Serial.begin(115200);

  OperatingState = USB_Detect_Hold; // Set First State to wait user press Start button
                                    // When Debugging, developer can point to any value of interest according to test case
  Sync_Basic_Task();
  PS4_Repeat_Init_Code(); // Attempt Scan for USB Host Repeatedly
  InputTask.enable();     // Enable Input Task to be monitored
  ProcessTask.enable();   // Enable Process Task to be monitored
  OutputTask.enable();    // Enable Output Task to be monitored
  DebugMessageTask.enable();  
  
}

void loop() 
{
  PS4_Repeat_Init_Task.check();   // Keep Retry to check USB host connection for 1 second until connected (Refer to Variable.h)
  if (USB_Detected)
  {
    InputTask.check();              // Input Task fire threads every 10ms (Refer to Variable.h)
    ProcessTask.check();            // Process Task fire threads every 10ms (Refer to Variable.h)
    OutputTask.check();             // Output Task fire threads every 10ms (Refer to Variable.h)
  }
  else
  {
    Sync_Basic_Task();              // No USB detected, dont run anything, but keep syncing Input, Process and Output Task Interval.
  }
  DebugMessageTask.check();       // Debug Message Print fire threads every 1ms (Refer to Variable.h)
}

void Sync_Basic_Task()
{
// The following code is purposely to control when each task is being execute  
// Without the following code, program can still run, but programmer have no idea which Task will be execute first.  
    InputTask.reset();   // Input Task Run from 10ms from this point onwards
    delay(3);            // Process Task Shall Execute 3ms after Input Task
    ProcessTask.reset(); // Process Task Run from 10ms from this point onwards
    delay(3);            // Output Task Shall Execute 3ms after Process Task
    OutputTask.reset();  // Output Task Run from 10ms from this point onwards
// The above code is purposely to control when each task is being execute    
}
