//PS4
#include <PS2X_lib.h>  
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
  CROSS_Pressed     = false;int UP         = 0;
  RIGHT      = 0;
  DOWN       = 0;
  LEFT       = 0;
  L1         = 0;
  R1         = 0;
  L2         = 0;
  R2         = 0;
  SQUARE     = 0;
  CIRCLE     = 0;
  TRIANGLE   = 0;
  CROSS      = 0;
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
      if(TRIANGLE_Pressed) 
      {
//        if (stepper1.distanceToGo() == 0) // if Reach absolute distance desired, what to do?
//        if (stepper2.distanceToGo() == 0 // if Reach absolute distance desired, what to do?
//        if (stepper1.isRunning())        // if Stepper Motor 1 is running, what to do?
//        if (stepper2.isRunning())        // if Stepper Motor 2 is running, what to do?
        stepper1.moveTo(-distance1);
        stepper2.moveTo(distance1);
        Stepper1Move = 1;
        Stepper2Move = 1;
        prev_tri = 1;
      } else if (SQUARE_Pressed)
      {
        if (!prev_tri){
          stepper1.moveTo(-distance2);
          stepper2.moveTo(distance2);
          Stepper1Move = 1;
          Stepper2Move = 1;
        } else{
          stepper1.moveTo(-distance1);
          stepper2.moveTo(distance1);
          Stepper1Move = 1;
          Stepper2Move = 1;
          prev_tri = 0;
        }
      } else if (CIRCLE_Pressed){    
        Stepper1Move = 0;
        Stepper2Move = 0;
        prev_tri = 0;
      }

      if (RIGHT_Pressed){
        if (prev_up){
          tilt_up = 1;
          prev_up = 0;
        } else if (prev_down){
          tilt_down = 1;
          prev_down = 0;
        } else{                
          stepper1.moveTo(-distance1);
          stepper2.moveTo(distance1);
          Stepper1Move = 1;
          Stepper2Move = 1;
          keep = 1;
          prev_right = 1;
        }
      } else if (UP_Pressed){
        if (prev_right){
          stepper1.moveTo(-distance1);
          stepper2.moveTo(distance1);
          Stepper1Move = 1;
          Stepper2Move = 1;
          keep = 1;
          prev_right = 0;
        } else if (prev_down){
          tilt_down = 1;
          prev_down = 0;
        } else{
          Stepper1Move = 0;
          Stepper2Move = 0;
          tilt_up = 1;
          prev_up = 1;
        }
      }else if(DOWN_Pressed){
        if (prev_up){
          tilt_up = 1;
          prev_up = 0;
        } else if (prev_right){
          stepper1.moveTo(-distance1);
          stepper2.moveTo(distance1);
          Stepper1Move = 1;
          Stepper2Move = 1;
          keep = 1;
          prev_down = 0;
        } else{
          Stepper1Move = 0;
          Stepper2Move = 0;
          tilt_down = 1;
          prev_down = 1;
        }
      }else if(CIRCLE_Pressed){
        Stepper1Move = 0;
        Stepper2Move = 0;
        tilt_up = 0;
        tilt_down = 0;
        keep = 0;
      }
      else {
        tilt_up = 0;
        tilt_down = 0;
        keep = 0;
        prev_down = 0;
        prev_up = 0;
        prev_right = 0;
      }

      if(CROSS_Pressed) 
      {
        if (!prev_tri){
          keep = 0;
          tilt_up = 0;
          tilt_down = 0;
          Stepper1Move = 0;
          Stepper2Move = 0;
          // Exit Circle Move Operation
          OperatingState = CheckActionToDo;
        } else{
          stepper1.moveTo(-distance1);
          stepper2.moveTo(distance1);
          Stepper1Move = 1;
          Stepper2Move = 1;
          prev_tri = 0;
        }
      }
      ClearButtonStatus();
      break;
      
    case RectangleMove:
      if (UP_Pressed){
        tilt_up = 1;
        tilt_down = 0;
        ClearButtonStatus();      
      } else if (DOWN_Pressed){
        tilt_up = 0;
        tilt_down = 1;
        ClearButtonStatus();      
      } else{
        tilt_up = 0;
        tilt_down = 0;
      }

      if (CIRCLE_Pressed){
        rotate = 0;
        ClearButtonStatus(); 
      } else if (TRIANGLE_Pressed){
        rotate = 1;   
      }

      if (LEFT_Pressed){
        shoot = 1;
        ClearButtonStatus();    
      } else if (RIGHT_Pressed){
        shoot = 0;
        ClearButtonStatus();    
      }

      if(CROSS_Pressed) 
      {
        // Exit Rectangle Move Operation 
        tilt_up = 0;
        tilt_down = 0;
        rotate = 0;
        shoot = 0;
        ClearButtonStatus();      // Check Carefully if this is necessary
        OperatingState = CheckActionToDo;
      }
      break;

    case TriangleMove:
      resetYawAngle();
      OperatingState = CheckActionToDo;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Forward:
      if(CROSS_Pressed) 
      {
        // Exit Forward Move Operation
        ramping_counter = 0; 
        ForwardSpeed = -1;
        OperatingState = CheckActionToDo;
      }
      myPID.Compute();
      Output = 0;         // PID output is not used
      ramping_counter++;
      if (ramping_counter == 1){     //check time to max speed (max speed x ramping counter x function time (10ms))
        ForwardSpeed++;  
        ramping_counter = 0;
      }    
      if (ForwardSpeed > motor_speed) ForwardSpeed = motor_speed; // Perform ramping by using counter (PX)
      BackwardSpeed =-1;
      LeftSpeed = -1;
      RightSpeed = -1;
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Backward:
      if(CROSS_Pressed) 
      {
        // Exit Backward Move Operation 
        ramping_counter = 0;
        BackwardSpeed = -1;
        OperatingState = CheckActionToDo;
      }
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
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Left:
      if(CROSS_Pressed) 
      {
        // Exit Left Move Operation 
        ramping_counter = 0;
        LeftSpeed = -1;
        OperatingState = CheckActionToDo;
      }      
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
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Move_Right:
      if(CROSS_Pressed) 
      {
        // Exit Right Move Operation 
        ramping_counter = 0;
        RightSpeed = -1;
        OperatingState = CheckActionToDo;
      }
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
      ClearButtonStatus();      // Check Carefully if this is necessary
      break;
      
    case Rotate_Left:
      if(CROSS_Pressed) 
      {
        // Exit Left Move Operation 
        ramping_counter = 0;
        RotateLeftSpeed = -1;
        OperatingState = CheckActionToDo;
      }      
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        RotateLeftSpeed++;  
        ramping_counter = 0;
      }    
      if (RotateLeftSpeed > motor_speed) RotateLeftSpeed = rotate_speed; // Perform ramping by using counter (PX)
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
        RotateRightSpeed = -1;
        OperatingState = CheckActionToDo;
      }
      ramping_counter++;
      if (ramping_counter == 1){     //check ramping_counter (max speed x ramping counter x function time (10ms)
        RotateRightSpeed++;  
        ramping_counter = 0;
      }    
      if (RotateRightSpeed > motor_speed) RotateRightSpeed = rotate_speed; // Perform ramping by using counter (PX)
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
  if(LeftSpeed > 0)                 MoveLeft(LeftSpeed);
  else if(RightSpeed > 0)           MoveRight(RightSpeed); 
  else if(ForwardSpeed > 0)         MoveFront(ForwardSpeed);
  else if(BackwardSpeed > 0)        MoveBack(BackwardSpeed);
  else if(RotateLeftSpeed > 0)      RotateLeft(RotateLeftSpeed);
  else if(RotateRightSpeed > 0)     RotateRight(RotateRightSpeed); 
  else                              MotorStopping();

  
  if(tilt_up)                       Tilt_Up();  
  else if(tilt_down)                Tilt_Down();  
  else                              Stop_Tilting(); 

  if(shoot)                         Shoot();
  else                              Retract(); 

  if(rotate)                        Start_Flywheel_Front();
  else if(keep)                     Start_Flywheel_Back();
  else                              Stop_Flywheel();


}

void StepperControlCode()
{
  if(Stepper1Move || Stepper2Move)  Lift();    
  else                              Stop_Stepper();
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
}

//Mechanism
void Lift(){
  if (Stepper1Move && Stepper2Move){
    Serial.println("Lift");
    stepper1.run();
    stepper2.run();
  } else if (Stepper1Move){
    stepper1.run();
    stepper2.stop();
  } else if (Stepper2Move){
    stepper2.run();
    stepper1.stop();
  } else{
    stepper1.stop();
    stepper2.stop();
  }
}

void Tilt_Up(){
  Serial.println("Tilt Up");
  digitalWrite(tilt_dir, 1);
  analogWrite(tilt_pwm, 255);
}

void Tilt_Down(){
  Serial.println("Tilt Down");
  digitalWrite(tilt_dir, 0);
  analogWrite(44, 255);
}

void Start_Flywheel_Front(){
  Serial.println("Start Flywheel");
  digitalWrite(flywheel2_dir, 0);
  delayMicroseconds(100);
  digitalWrite(flywheel1_dir, 1);
  analogWrite(13, 255);
  analogWrite(flywheel2_pwm, 255);
}

void Start_Flywheel_Back(){
  Serial.println("Back");
  digitalWrite(flywheel1_dir, 0);
  delayMicroseconds(100);
  digitalWrite(flywheel2_dir, 1);
  analogWrite(flywheel1_pwm, 100);
  analogWrite(flywheel2_pwm, 100);
}

void Stop_Flywheel(){
  analogWrite(flywheel1_pwm, 0);
  analogWrite(flywheel2_pwm, 0);
}

void Stop_Tilting()
{
  analogWrite(tilt_pwm, 0);
}

void Stop_Stepper(){
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper1.stop();
  stepper2.stop();
}

void Shoot(){
  digitalWrite(pneumatic, 0);
}

void Retract(){
  digitalWrite(pneumatic, 1);
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
  Serial1.write("\xFF\xAA\x52")  ; //0xFF 0xAA 0x52 is command for initializing angle in z Direction =0
//  Serial1.write(0xFF);
//  Serial1.write(0xAA);
//  Serial1.write(0x52); 
  
  //PID
  Input = yaw_angle;
  setpoint = 0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampletime);
  myPID.SetOutputLimits(-25500, 25500);
  DDRA = B11111111;
  PORTA = B00000000;
  pinMode(tilt_pwm, OUTPUT);
  pinMode(tilt_dir, OUTPUT);
  pinMode(flywheel1_dir, OUTPUT);
  pinMode(flywheel2_dir, OUTPUT);
  MotorStopping();

  //Stepper
//  stepper1.setMaxSpeed(600);    
//  stepper1.setAcceleration(180);
//  stepper2.setMaxSpeed(300);   
//  stepper2.setAcceleration(60);
  // Since StepperAlternator implemented, try use stepper motor max speed and acceleration with same value
  stepper1.setMaxSpeed(2000);      // if motor driver microstep is 1, motor 200 step / rev, then motor speed is 2 rev/s
  stepper1.setAcceleration(1000);  // 2 seconds to reach max speed
  stepper2.setMaxSpeed(2000);      // if motor driver microstep is 1, motor 200 step / rev, then motor speed is 2 rev/s
  stepper2.setAcceleration(1000);  // 2 seconds to reach max speed
}



void setup() 
{
  Serial.begin(115200);

  OperatingState = USB_Detect_Hold; // Set First State to wait user press Start button
  USB_Detected = false;
                                    // When Debugging, developer can point to any value of interest according to test case
  IMU_PID_Stepper_Setup_Code();   // Enable IMU(z-axis) & PID
  Sync_Basic_Task();
  PS4_Repeat_Init_Code(); // Attempt Scan for USB Host Repeatedly
  StepperControlTask.reset();
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
  StepperControlTask.enable();
  
}

void loop() 
{
  PS4_Repeat_Init_Task.check();   // Keep Retry to check USB host connection for 1 second until connected (Refer to Variable.h)
  if (USB_Detected)
  {
    InputTask.check();              // Input Task fire threads every 10ms (Refer to Variable.h)
    ProcessTask.check();            // Process Task fire threads every 10ms (Refer to Variable.h)
    OutputTask.check();             // Output Task fire threads every 10ms (Refer to Variable.h)
    StepperControlTask.check();     // Stepper Control Need to be updated faster
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
    StepperControlTask.reset();     // Stepper Control Need to be updated faster
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
