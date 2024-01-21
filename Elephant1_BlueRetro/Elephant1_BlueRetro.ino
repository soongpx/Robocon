//PS4
#include <SPI.h>
#include <TimedAction.h>
//PID
#include <PID_v1.h>
#include "ProgramConstant.h"
#include "Variable.h"
//USB Set UP 
#include <AccelStepper.h>

//PID
PID myPID(&Input, &Output, &setpoint, Kp, Ki, Kd, DIRECT);

void DebugMessageTaskCode()
{
  if(DebugMessage != "")
  {
    Serial.println(DebugMessage);
    DebugMessage = "";          // Clear away the message so that serial terminal wont full of repeated useless message
  }
}


void PS4_Repeat_Init_Code()
{  
  SPI.begin();
  SPI.beginTransaction(SPISettings(100000, LSBFIRST, SPI_MODE3)); //5 Microseconds per bit
  pinMode(SPI_MISO, INPUT_PULLUP);
  pinMode(SlaveAck, INPUT_PULLUP);
  pinMode(SPI_CLK, OUTPUT); //configure ports
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SlaveSelect, OUTPUT);
  digitalWrite(SlaveSelect, HIGH);
  PS2_SPI_SEND(Enter_Config, 5);
  PS2_SPI_SEND(Turn_ON_Analog_Mode, 9);
  //  PS2_SPI_SEND(Maps_Motor,5);       // PS4 dont support this?
  //  PS2_SPI_SEND(Turn_ON_Pressure,9); // This is useless for ps4
  PS2_SPI_SEND(Exit_Config, 9);
  PS2_SPI_SEND(ReadAllData, BufferSize);
  if (SPI_Packet[2] != 0x5A){
    USB_Detected = false;
    DebugMessage = F("\nNo controller found, check wiring");
    // If error persists without having wiring problem, close the power source and on again
    PS4_Repeat_Init_Task.enable();          //  It is ok to keep enable, it is just a flag only, not starting another thread
  } else{
    USB_Detected = true;
    DebugMessage = F("Found Controller, configured successful");
    PS4_Repeat_Init_Task.disable();          // It is ok to keep disable
  }
}
void InputTaskCode()
{
  PS2_SPI_SEND(ReadAllData, BufferSize);    
  if(SPI_Packet[3] ^ 0xFF) 
    {
      if (~SPI_Packet[3] & DPAD_UP) UP_Pressed = 1;
      if (~SPI_Packet[3] & DPAD_DOWN) DOWN_Pressed = 1;
      if (~SPI_Packet[3] & DPAD_LEFT) LEFT_Pressed = 1;
      if (~SPI_Packet[3] & DPAD_RIGHT) RIGHT_Pressed = 1;
    }
  if(SPI_Packet[4] ^ 0xFF) 
  {
    if (~SPI_Packet[4] & L1_PRESSED) L1_Pressed = 1;
    if (~SPI_Packet[4] & L2_PRESSED) L2_Pressed = 1;
    if (~SPI_Packet[4] & R1_PRESSED) R1_Pressed = 1;
    if (~SPI_Packet[4] & R2_PRESSED) R2_Pressed = 1;
    if (~SPI_Packet[4] & GREEN_TRIANGLE_PRESSED) TRIANGLE_Pressed = 1;
    if (~SPI_Packet[4] & RED_CIRCLE_PRESSED) CIRCLE_Pressed = 1;
    if (~SPI_Packet[4] & BLUE_CROSS_PRESSED) CROSS_Pressed = 1;
    if (~SPI_Packet[4] & PINK_SQUARE_PRESSED) SQUARE_Pressed = 1;
  }
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
  L2_Pressed        = false;
  R2_Pressed        = false;
}
void ProcessTaskCode()
{
  switch (OperatingState)
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
        check = 0;
        OperatingState = CircleMove;
        Serial.println("User Select to Move In Circle");
      }
      else if (SQUARE_Pressed)
      {
        check = 0;
        OperatingState = RectangleMove;
        Serial.println("User Select to Move In Rectangle");
      }
      else if (CROSS_Pressed)
      {
        check = 0;
        OperatingState = CrossMove;
        Serial.println("User Select to Move In Cross");
      }
      else if (UP_Pressed)
      {
        check = 0;
        OperatingState = Move_Forward;
        Serial.println("User Select to Move Forward");
      }
      else if (DOWN_Pressed)
      {
        check = 0;
        OperatingState = Move_Backward;
        Serial.println("User Select to Move Backward");
      }
      else if (LEFT_Pressed)
      {
        check = 0;
        OperatingState = Move_Left;
        Serial.println("User Select to Move Left");
      }
      else if (RIGHT_Pressed)
      {
        check = 0;
        OperatingState = Move_Right;
        Serial.println("User Select to Move Right");
      }
      else if (L1_Pressed)
      {
        check = 0;
        OperatingState = Rotate_Left;
        Serial.println("User Select to Rotate Left");
      }
      else if (R1_Pressed)
      {
        check = 0;
        OperatingState = Rotate_Right;
        Serial.println("User Select to Rotate Right");
      }
      else 
      {
        check++;
        if (check == 1) {
          if (pneu == 1) pneu = 0; 
          if (pneu == 0) pneu = 1; 
          if (ser == 1) ser = 0;
          if (ser == 0) ser = 1;
        }
        ClearButtonStatus(); // In Action Checking Cycle, disregard some functional button
        Serial.println("User Select Non Action Button, Do nothing");
      }
      break;
      
    case CircleMove:
      conveyor_move = 1;
      OperatingState = CheckActionToDo ;
      ClearButtonStatus();
      break;
         
    case RectangleMove:
      if (pneu == 1){
        keep = 1;
      } else{
        keep = 0;
      }
        OperatingState = CheckActionToDo ;
      ClearButtonStatus();
      break;

    case CrossMove:
      if (ser == 1){
        servoAngle = 179;
      } else {
        servoAngle = 0;
      }
        OperatingState = CheckActionToDo ;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Forward:
      lift_up = 1;
      lift_down = 0;
        OperatingState = CheckActionToDo ;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Backward:
      lift_down = 1;
      lift_up = 0;
        OperatingState = CheckActionToDo ;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    default:                    // For in case something go wrong, force user press Start Button again
      conveyor_move = 0;
      keep = 0;
      lift_up = 0;
      lift_down = 0;
      OperatingState = WaitStart;
      Serial.println("Warning : Serious Error Occur, Entering Unexpected state");
      ClearButtonStatus();      // Check Carefully if this is necessary
  }
}

void OutputTaskCode()
{
  if (conveyor_move)                          MoveConveyor();
  else                                        StopConveyor();

  if (keep)                                   Keep();
  else                                        NoKeep(); 
  
  if (servoAngle == 0 || servoAngle == 179){
    Servo1.write(servoAngle);
    Servo2.write(servoAngle);
  }

  if (lift_up)                                LiftUp();
  else if (lift_down)                         LiftDown();
  else                                        NoLift();
}

//Mechanism
void LiftUp(){
  Serial.println("Lift Up");
  digitalWrite(lift_dir, 1);
  analogWrite(lift_pwm, 255);
}

void LiftDown(){
  Serial.println("Lift Down");
  digitalWrite(lift_dir, 0);
  analogWrite(lift_pwm, 255);
}

void NoLift(){
  digitalWrite(lift_dir, 0);
  analogWrite(lift_pwm, 0);
}

void MoveConveyor(){
  Serial.println("Move Conveyor");
  digitalWrite(conveyor_dir, 1);
  analogWrite(conveyor_pwm, 255);
}

void StopConveyor(){
  analogWrite(conveyor_pwm, 0);
}

void Keep(){
  Serial.println("Pneumatic");
  digitalWrite(pneumatic, 1);
}

void NoKeep(){
  digitalWrite(pneumatic, 0);
}

//PS4
void PS2_SPI_SEND(byte* Data, uint8_t DataSize)
{
  uint8_t i = 0;
  digitalWrite(SlaveSelect, LOW);   // Set Attention Line Low at Start of Packet
  while (i < DataSize)            // Block for 450 Microseconds ( (40 + 10) x 9)
  {
    SPI_Packet[i] = SPI.transfer(Data[i]);
    delayMicroseconds(16);          //Delay 10 Microseconds (Seems Compulsory)
    i = i + 1;
  } // Please take note PS2 data retrieval takes 450 microseconds , may intefere Stepper motor control timing.
  digitalWrite(SlaveSelect, HIGH);  // Set Attention Line High after End of Packet
}

void Servo_Setup_Code()
{
  //Servo
  Servo1.attach(4);
  Servo2.attach(5);
}



void setup() 
{
  Serial.begin(115200);

  OperatingState = USB_Detect_Hold; // Set First State to wait user press Start button
  USB_Detected = false;
                                    // When Debugging, developer can point to any value of interest according to test case
  Servo_Setup_Code();   // Enable Servo
  Sync_Basic_Task();
  DebugMessageTask.reset();  
  InputTask.reset();     // Set InputTask Time stamp to current millis
  delay(3);
  ProcessTask.reset();   // Process Task Execute after 3ms InputTask is execute
  delay(3);
  OutputTask.reset();    // Output Task Execute after 3ms Process Task is execute
  delay(3);
  InputTask.enable();     // Enable Input Task to be monitored
  ProcessTask.enable();   // Enable Process Task to be monitored
  OutputTask.enable();    // Enable Output Task to be monitored
  DebugMessageTask.enable();  
  PS4_Repeat_Init_Code(); // Attempt Scan for USB Host Repeatedly
  
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

/*Errors need to be solved
1.) Unknown Controller will be found error
    - Need to close arduino power source and connect back
    - Try connect BlueRetro to a power source first only switch on Arduino
2.) Start Button Initialise
    - Start button will be 1 during the setup phase (Tested using raw code)
3.) Joystick Value 
    - Joystick value will be shown sometimes even though nothing is pressed (Do checking when joystick is moved)
4.) L2 and R2 
    - Pressing L2 and R2 may lead to calling other buttons flag to be 1 (Check unusual buttons) (Not recommend to use L2 and R2 buttons)
*/
