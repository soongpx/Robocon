//PS4
#include <SPI.h>
#include <TimedAction.h>
//PID
#include <PID_v1.h>
#include "ProgramConstant.h"
#include "Variable.h"
//USB Set UP

//PID
PID myPID(&Input, &Output, &setpoint, Kp, Ki, Kd, DIRECT);


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
  L2_Pressed        = false;
  R2_Pressed        = false;
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
      else if (L2_Pressed)
      {
        OperatingState = Left_2;
        Serial.println("User Select to L2");
      }
      else if (R2_Pressed)
      {
        OperatingState = Right_2;
        Serial.println("User Select to R2");
      }
      else 
      {
        ramping_counter = 0; 
        ForwardSpeed = -1;
        BackwardSpeed = -1;
        LeftSpeed = -1;
        RightSpeed = -1;
        RotateRightSpeed = -1;
        RotateLeftSpeed = -1;
        ClearButtonStatus(); // In Action Checking Cycle, disregard some functional button
        Serial.println("User Select Non Action Button, Do nothing");
      }
      break;
      
    case CircleMove:
      Pull = 1;
      if (CROSS_Pressed){
        Pull = 0;
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case RectangleMove:
      Push = 1;
      if (TRIANGLE_Pressed){
        Release = 1;
      } else if (CIRCLE_Pressed){
        Suck = 1;
      } else{
        Release = 0;
        Suck = 0;
      }
      if (CROSS_Pressed){
        Push = 0;
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;

    case TriangleMove:
      if (SQUARE_Pressed){
        Up = 1;
      } else if (CIRCLE_Pressed){
        Down = 1;
      } else{
        Up = 0;
        Down = 0;

      }
      if (CROSS_Pressed){
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Forward:
      myPID.Compute();
      Output = 0;         // PID output is not used
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        ForwardSpeed++;  
        ramping_counter = 0;
      }    
      if (ForwardSpeed > motor_speed) ForwardSpeed = motor_speed; // Perform ramping by using counter (PX)
      BackwardSpeed =-1;
      LeftSpeed = -1;
      RightSpeed = -1;
      RotateLeftSpeed = -1;
      RotateRightSpeed = -1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      OperatingState = CheckActionToDo;
      break;
      
    case Move_Backward:
      myPID.Compute();
      Output = 0;         // PID output is not used
      ramping_counter++;
      if (ramping_counter == 1){    //check ramping_counter (max speed x ramping counter x function time (10ms)
        BackwardSpeed++;  
        ramping_counter = 0;
      }  
      if (BackwardSpeed > motor_speed) BackwardSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      LeftSpeed = -1;
      RightSpeed = -1;
      RotateLeftSpeed = -1;
      RotateRightSpeed = -1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Left:
      myPID.Compute();
      Output = 0;         // PID output is not used
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        LeftSpeed++;  
        ramping_counter = 0;
      }    
      if (LeftSpeed > motor_speed) LeftSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      BackwardSpeed = -1;
      RightSpeed =-1;
      RotateLeftSpeed = -1;
      RotateRightSpeed = -1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Right:
      myPID.Compute();
      Output = 0;         // PID output is not used
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        RightSpeed++;  
        ramping_counter = 0;
      }     
      if (RightSpeed > motor_speed) RightSpeed = motor_speed; // Perform ramping by using counter (PX)
      ForwardSpeed =-1;
      BackwardSpeed = -1;
      LeftSpeed =-1;
      RotateLeftSpeed = -1;
      RotateRightSpeed = -1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Rotate_Left:
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
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
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
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

    case Left_2:
      if (TRIANGLE_Pressed){
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;

    case Right_2:
      OperatingState = CheckActionToDo;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    default:                    // For in case something go wrong, force user press Start Button again
      Pull = 0;
      Push = 0;
      Suck = 0;
      Up = 0;
      Down = 0;
      Release = 0;
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
  else if (Pull)                  pull();
  else if (Push)                  push();
  else if (Up)                    tilt_up();
  else if (Down)                  tilt_down();
  else                            MotorStopping();
  if (Release)               release();
  else if (Suck)                  suck();
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
  Serial.println(pwm);
  
  analogWrite(wheel1, cap200PWMValue(pwm - Output));
  analogWrite(wheel2, cap200PWMValue(pwm + Output));
  analogWrite(wheel3, cap200PWMValue(pwm - Output));
  analogWrite(wheel4, cap200PWMValue(pwm + Output));
}

void MoveBack(int  pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  PORTA &= ~(1 << dir_4);
  Serial.println(pwm);
   
  analogWrite(wheel1, cap200PWMValue(pwm + Output));
  analogWrite(wheel2, cap200PWMValue(pwm - Output));
  analogWrite(wheel3, cap200PWMValue(pwm + Output));
  analogWrite(wheel4, cap200PWMValue(pwm - Output));
}

void MoveLeft(int pwm)
{
  PORTA &= ~(1 << dir_1);
  PORTA &= ~(1 << dir_4);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_2);
  PORTA |= (1 << dir_3);
  
  analogWrite(wheel1, cap200PWMValue(pwm + Output));
  analogWrite(wheel2, cap200PWMValue(pwm + Output));
  analogWrite(wheel3, cap200PWMValue(pwm - Output));
  analogWrite(wheel4, cap200PWMValue(pwm - Output));
}

void MoveRight(int pwm)
{
  PORTA &= ~(1 << dir_2);
  PORTA &= ~(1 << dir_3);
  delayMicroseconds(100); // Make sure all direction set to 0, only then turn on correct direction
  PORTA |= (1 << dir_1);
  PORTA |= (1 << dir_4);
  
  analogWrite(wheel1, cap200PWMValue(pwm - Output));
  analogWrite(wheel2, cap200PWMValue(pwm - Output));
  analogWrite(wheel3, cap200PWMValue(pwm + Output));
  analogWrite(wheel4, cap200PWMValue(pwm + Output));
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
  analogWrite(tilting_pwm_pin, 0);
  analogWrite(loading_pwm_pin, 0);
}

//Mechanism
void tilt_up(){
  digitalWrite(tilting_dir_pin, 1);
  analogWrite(tilting_pwm_pin,255);  
}

void tilt_down(){
  digitalWrite(tilting_dir_pin, 0);
//  PORTA &= ~(1 << tilting_dir_pin);
  analogWrite(tilting_pwm_pin,255);
}

void pull(){
  PORTA |= (1 << loading_dir_pin);
  analogWrite(loading_pwm_pin,255);
}

void push(){
  PORTA &= ~(1 << loading_dir_pin);
  analogWrite(loading_pwm_pin,255);
}

void suck(){
  PORTA |= (1 << magnet_dir_pin);
}

void release(){
  PORTA &= ~(1 << magnet_dir_pin);
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

void IMU_PID_Stepper_Setup_Code()
{
  //IMU
  Serial1.begin(115200);
  Serial1.write(0xFF);
  Serial1.write(0xAA);
  Serial1.write(0x52); //0xFF 0xAA 0x52 is command for initializing angle in z Direction =0
  
  //PID
  Input = yaw_angle;
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampletime);
  myPID.SetOutputLimits(-25500, 25500);
  DDRA = B11111111;
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
  IMU_PID_Stepper_Setup_Code();   // Enable IMU(z-axis) & PID
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
