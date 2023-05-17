//PS4
#include <PS2X_lib.h>  
#include <TimedAction.h>
#include <AccelStepper.h>
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
  error = ps2x.config_gamepad(52,51,53,50, true, true);   //GamePad(clock, command, attention, data, Pressures?, Rumble?) 
 //13,11,10,12 (Uno)
 //52,51,53,50 (Mega)
  if(error == 0){
    USB_Detected = true;
    DebugMessage = F("Found Controller, configured successful");
    PS4_Repeat_Init_Task.disable();          // It is ok to keep enable, it is just a flag only, not starting another thread 
  }
   
  else if(error == 1){
    USB_Detected = false;
    DebugMessage = F("\nNo controller found, check wiring");
    // If error persists without having wiring problem, close the power source and on again
    PS4_Repeat_Init_Task.enable();          //  It is ok to keep enable, it is just a flag only, not starting another thread
  }
   
  else if(error == 2){
    USB_Detected = false;
    DebugMessage = F("\nController found but not accepting commands.");
    PS4_Repeat_Init_Task.enable();          //  It is ok to keep enable, it is just a flag only, not starting another thread
  }
   
  else if(error == 3){
    USB_Detected = false;
    DebugMessage = F("\nController refusing to enter Pressures mode");
    PS4_Repeat_Init_Task.enable();          //  It is ok to keep enable, it is just a flag only, not starting another thread
  }
}

void InputTaskCode()
{
  ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed
  UP_Pressed        = ps2x.Button(PSB_PAD_UP)?       true:UP_Pressed;
  RIGHT_Pressed     = ps2x.Button(PSB_PAD_RIGHT)?    true:RIGHT_Pressed;
  DOWN_Pressed      = ps2x.Button(PSB_PAD_DOWN)?     true:DOWN_Pressed;
  LEFT_Pressed      = ps2x.Button(PSB_PAD_LEFT)?     true:LEFT_Pressed;
  L1_Pressed        = ps2x.Button(PSB_L1)?           true:L1_Pressed;
  R1_Pressed        = ps2x.Button(PSB_R1)?           true:R1_Pressed;
  L2_Pressed        = ps2x.Button(PSB_L2)?           true:L2_Pressed;
  R2_Pressed        = ps2x.Button(PSB_R2)?           true:R2_Pressed;
  SQUARE_Pressed    = ps2x.Button(PSB_PINK)?         true:SQUARE_Pressed;
  CIRCLE_Pressed    = ps2x.Button(PSB_RED)?          true:CIRCLE_Pressed;
  TRIANGLE_Pressed  = ps2x.Button(PSB_GREEN)?        true:TRIANGLE_Pressed;
  CROSS_Pressed     = ps2x.Button(PSB_BLUE)?         true:CROSS_Pressed;
  START_Pressed     = ps2x.Button(PSB_START)?        true:START_Pressed;
  // Scan IMU Reading from serial
  calYawAngle();
  Input = yaw_angle;
  // Scan ON/OFF Switch or Sensor Reading
}

void ClearButtonStatus()
{
  UP_Pressed        = false;
  RIGHT_Pressed     = false;
  DOWN_Pressed      = false;
  LEFT_Pressed      = false;
  L1_Pressed        = false;
  R1_Pressed        = false;
  SQUARE_Pressed    = false;
  CIRCLE_Pressed    = false;
  TRIANGLE_Pressed  = false;
  CROSS_Pressed     = false;
  START_Pressed     = false;  
}
void ProcessTaskCode()
{
  switch(OperatingState)
  {
    case USB_Detect_Hold:
        if (USB_Detected = true){
        OperatingState = WaitStart; // Set First State to wait user press Start button
        }
        else DebugMessage = F("Detecting PS4 from USB Host ");
      break;
    case WaitStart:
      if(CROSS_Pressed){
        OperatingState = CheckActionToDo ;
      }
      ClearButtonStatus(); // May be "START_Pressed = false" is more relevant
      //Start Button may be 1 when PS4 is not connected to BlueRetro
      //Suggestion is to change Start Button to another button
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
      else if (TRIANGLE_Pressed)
      {
        OperatingState = TriangleMove;
        Serial.println("User Select to Move In Triangle");
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
      else if (L1_Pressed)
      {
        OperatingState = Rotate_Left;
        Serial.println("User Select to Rotate Left");
      }
      else if (R1_Pressed)
      {
        OperatingState = Rotate_Right;
        Serial.println("User Select to Rotate Right");
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

    case TriangleMove:
      resetYawAngle();
      Input = 0;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Forward:
      if(CROSS_Pressed) 
      {
        // Exit Forward Move Operation
        ramping_counter = 0; 
        OperatingState = CheckActionToDo;
      }
      ramping_counter++;
      if (ramping_counter == 10){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        ForwardSpeed++;  
        ramping_counter = 0;
      }    
      if (ForwardSpeed > motor_speed) ForwardSpeed = motor_speed; // Perform ramping by using counter (PX)
      BackwardSpeed =-1;
      LeftSpeed = -1;
      RightSpeed = -1;
      if (Input > 0)
        FrontOutput++;
      else if (Input < 0)
        FrontOutput--;
      else
        FrontOutput = FrontOutput;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Backward:
      if(CROSS_Pressed) 
      {
        // Exit Backward Move Operation 
        ramping_counter = 0;
        OperatingState = CheckActionToDo;
      }
      ramping_counter++;
      if (ramping_counter == 10){    //check ramping_counter (max speed x ramping counter x function time (10ms)
        BackwardSpeed++;  
        ramping_counter = 0;
      }  
      if (BackwardSpeed > motor_speed) BackwardSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      LeftSpeed = -1;
      RightSpeed = -1;
      if (Input > 0)
        BackOutput++;
      else if (Input < 0)
        BackOutput--;
      else
        BackOutput = BackOutput;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Left:
      if(CROSS_Pressed) 
      {
        // Exit Left Move Operation 
        ramping_counter = 0;
        OperatingState = CheckActionToDo;
      }      
      ramping_counter++;
      if (ramping_counter == 10){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        LeftSpeed++;  
        ramping_counter = 0;
      }    
      if (LeftSpeed > motor_speed) LeftSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      BackwardSpeed = -1;
      RightSpeed =-1;
      if (Input > 0)
        LeftOutput++;
      else if (Input < 0)
        LeftOutput--;
      else
        LeftOutput = LeftOutput;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Right:
      if(CROSS_Pressed) 
      {
        // Exit Right Move Operation 
        ramping_counter = 0;
        OperatingState = CheckActionToDo;
      }
      ramping_counter++;
      if (ramping_counter == 10){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        RightSpeed++;  
        ramping_counter = 0;
      }     
      if (RightSpeed > motor_speed) RightSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      BackwardSpeed = -1;
      LeftSpeed =-1;
      if (Input > 0)
        RightOutput++;
      else if (Input < 0)
        RightOutput--;
      else
        RightOutput = RightOutput;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Rotate_Left:
      if(CROSS_Pressed) 
      {
        // Exit Left Move Operation 
        ramping_counter = 0;
        OperatingState = CheckActionToDo;
      }      
      ramping_counter++;
      if (ramping_counter == 10){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        RotateLeftSpeed++;  
        ramping_counter = 0;
      }    
      if (RotateLeftSpeed > motor_speed) RotateLeftSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      BackwardSpeed = -1;
      LeftSpeed = -1;
      RightSpeed =-1;
      RotateRightSpeed = -1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Rotate_Right:
      if(CROSS_Pressed) 
      {
        // Exit Right Move Operation 
        ramping_counter = 0;
        OperatingState = CheckActionToDo;
      }
      ramping_counter++;
      if (ramping_counter == 10){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        RotateRightSpeed++;  
        ramping_counter = 0;
      }    
      if (RotateRightSpeed > motor_speed) RotateRightSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      BackwardSpeed = -1;
      LeftSpeed =-1;
      RightSpeed = -1;
      RotateLeftSpeed = -1;
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
  if(LeftSpeed > 0)               MoveLeft(LeftSpeed);
  else if(RightSpeed > 0)         MoveRight(RightSpeed); 
  else if(ForwardSpeed > 0)       MoveFront(ForwardSpeed);
  else if(BackwardSpeed > 0)      MoveBack(BackwardSpeed);
  else if(RotateLeftSpeed > 0)    RotateLeft(RotateLeftSpeed);
  else if(RotateRightSpeed > 0)   RotateRight(RotateRightSpeed); 
  else                            MotorStopping();
}

//Locomotion
int cap200PWMValue(int pwm){
  if (pwm > 200){
    return 200;
  } else{
    return pwm;
  }
}

void MoveFront(int pwm)
{
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_4);
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_3);
  
  analogWrite(wheel1, cap200PWMValue(pwm - FrontOutput));
  analogWrite(wheel2, cap200PWMValue(pwm + FrontOutput));
  analogWrite(wheel3, cap200PWMValue(pwm - FrontOutput));
  analogWrite(wheel4, cap200PWMValue(pwm + FrontOutput));
}

void MoveBack(int  pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);
   
  analogWrite(wheel1, cap200PWMValue(pwm + BackOutput));
  analogWrite(wheel2, cap200PWMValue(pwm - BackOutput));
  analogWrite(wheel3, cap200PWMValue(pwm + BackOutput));
  analogWrite(wheel4, cap200PWMValue(pwm - BackOutput));
}

void MoveLeft(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_3);
  
  analogWrite(wheel1, cap200PWMValue(pwm + LeftOutput));
  analogWrite(wheel2, cap200PWMValue(pwm + LeftOutput));
  analogWrite(wheel3, cap200PWMValue(pwm - LeftOutput));
  analogWrite(wheel4, cap200PWMValue(pwm - LeftOutput));
}

void MoveRight(int pwm)
{
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_4);
  
  analogWrite(wheel1, cap200PWMValue(pwm - RightOutput));
  analogWrite(wheel2, cap200PWMValue(pwm - RightOutput));
  analogWrite(wheel3, cap200PWMValue(pwm + RightOutput));
  analogWrite(wheel4, cap200PWMValue(pwm + RightOutput));
}

void RotateLeft(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_3);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_4);
  
  analogWrite(wheel1, cap200PWMValue(pwm));
  analogWrite(wheel2, cap200PWMValue(pwm));
  analogWrite(wheel3, cap200PWMValue(pwm));
  analogWrite(wheel4, cap200PWMValue(pwm));
}

void RotateRight(int pwm)
{
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_3);
  
  analogWrite(wheel1, cap200PWMValue(pwm));
  analogWrite(wheel2, cap200PWMValue(pwm));
  analogWrite(wheel3, cap200PWMValue(pwm));
  analogWrite(wheel4, cap200PWMValue(pwm));
}

void MotorStopping()
{
    analogWrite(wheel1, 0);
    analogWrite(wheel2, 0);
    analogWrite(wheel3, 0);
    analogWrite(wheel4, 0);
}

//IMU (added by PX)
void serialEvent1()
{
  while (Serial1.available())
  {
    Re_buf[counter] = (unsigned char)Serial1.read();
    // The fist byte is not the packet header,
    // skip to wait till the packet header to arrive
    if (counter == 0 && Re_buf[0] != 0x55) return;
    counter++;
    if (counter == 11) // All packet received, bool sign = true
    {
      counter = 0;
      sign = 1;
    }
  }
}

void calYawAngle() {
  if (sign) {
    sign = 0;
    if (Re_buf[0] == 0x55)
    {
      uint16_t sum = 0;

      for (int x = 0; x < 10; x++)
      {
        sum += Re_buf[x];   
      }

      while (sum > 256)
      {
        sum -= 256;
      }
      
      if ( sum == Re_buf[10] && Re_buf[1] == 83 ) {                       //0x53
        yaw_angle = (short(Re_buf [7] << 8 | Re_buf [6]) / 32768.0 * 180);
        Serial.print("Current Angle:");
        Serial.println(yaw_angle);
        suss++;
        //Serial.print("Failed: ");
        //Serial.println(fail / suss * 100);
        return;
      }
      else if (sum != Re_buf[10])
      {
        suss++;
        fail++;
        //Serial.print("Failed:  ");
        //Serial.println(fail / suss * 100);
        return;
      }
    }
  }
}

void resetYawAngle() {
  MotorStopping();
  Serial1.write(0xFF);
  Serial1.write(0xAA);
  Serial1.write(0x52);
}

void IMU_Setup_Code()
{
  //IMU
  Serial1.begin(115200);
  Serial1.write(0xFF);
  Serial1.write(0xAA);
  Serial1.write(0x52); //0xFF 0xAA 0x52 is command for initializing angle in z Direction =0

  DDRA = B00001111;
  PORTA = B00000000;
  MotorStopping();
}



void setup() 
{
  Serial.begin(115200);

  OperatingState = USB_Detect_Hold; // Set First State to wait user press Start button
  USB_Detected = false;
                                    // When Debugging, developer can point to any value of interest according to test case
  Sync_Basic_Task();
  PS4_Repeat_Init_Code(); // Attempt Scan for USB Host Repeatedly
  IMU_Setup_Code();       // Enable IMU(z-axis)
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
