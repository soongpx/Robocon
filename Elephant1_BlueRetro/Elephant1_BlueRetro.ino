//PS4
#include <PS2X_lib.h>
#include <TimedAction.h>
#include "ProgramConstant.h"
#include "Variable.h"

void DebugMessageTaskCode() {
  if (DebugMessage != "") {
    Serial.print(DebugMessage);
    DebugMessage = "";  // Clear away the message so that serial terminal wont full of repeated useless message
  }
}

void PS4_Repeat_Init_Code() {
  error = ps2x.config_gamepad(52, 51, 53, 50, true, true);  //GamePad(clock, command, attention, data, Pressures?, Rumble?)
                                                            //13,11,10,12 (Uno)
                                                            //52,51,53,50 (Mega)
  if (error == 0) {
    USB_Detected = true;
    DebugMessage = F("Found Controller, configured successful");
    PS4_Repeat_Init_Task.disable();  // It is ok to keep enable, it is just a flag only, not starting another thread
  }

  else if (error == 1) {
    USB_Detected = false;
    DebugMessage = F("\nNo controller found, check wiring");
    // If error persists without having wiring problem, close the power source and on again
    PS4_Repeat_Init_Task.enable();  //  It is ok to keep enable, it is just a flag only, not starting another thread
  }

  else if (error == 2) {
    USB_Detected = false;
    DebugMessage = F("\nController found but not accepting commands.");
    PS4_Repeat_Init_Task.enable();  //  It is ok to keep enable, it is just a flag only, not starting another thread
  }

  else if (error == 3) {
    USB_Detected = false;
    DebugMessage = F("\nController refusing to enter Pressures mode");
    PS4_Repeat_Init_Task.enable();  //  It is ok to keep enable, it is just a flag only, not starting another thread
  }
}
void InputTaskCode() {
  ps2x.read_gamepad(false, vibrate);  //read controller and set large motor to spin at 'vibrate' speed
  UP_Pressed = ps2x.Button(PSB_PAD_UP) ? true : UP_Pressed;
  RIGHT_Pressed = ps2x.Button(PSB_PAD_RIGHT) ? true : RIGHT_Pressed;
  DOWN_Pressed = ps2x.Button(PSB_PAD_DOWN) ? true : DOWN_Pressed;
  LEFT_Pressed = ps2x.Button(PSB_PAD_LEFT) ? true : LEFT_Pressed;
  L1_Pressed = ps2x.Button(PSB_L1) ? true : L1_Pressed;
  R1_Pressed = ps2x.Button(PSB_R1) ? true : R1_Pressed;
  L2_Pressed = ps2x.Button(PSB_L2) ? true : L2_Pressed;
  R2_Pressed = ps2x.Button(PSB_R2) ? true : R2_Pressed;
  SQUARE_Pressed = ps2x.Button(PSB_PINK) ? true : SQUARE_Pressed;
  CIRCLE_Pressed = ps2x.Button(PSB_RED) ? true : CIRCLE_Pressed;
  TRIANGLE_Pressed = ps2x.Button(PSB_GREEN) ? true : TRIANGLE_Pressed;
  CROSS_Pressed = ps2x.Button(PSB_BLUE) ? true : CROSS_Pressed;
  START_Pressed = ps2x.Button(PSB_START) ? true : START_Pressed;
}

void ClearButtonStatus() {
  UP_Pressed = false;
  RIGHT_Pressed = false;
  DOWN_Pressed = false;
  LEFT_Pressed = false;
  L1_Pressed = false;
  R1_Pressed = false;
  SQUARE_Pressed = false;
  CIRCLE_Pressed = false;
  TRIANGLE_Pressed = false;
  CROSS_Pressed = false;
  START_Pressed = false;
}
void ProcessTaskCode() {
  switch (OperatingState) {
    case USB_Detect_Hold:
      if (USB_Detected = true) {
        OperatingState = WaitStart;  // Set First State to wait user press Start button
      } else DebugMessage = F("Detecting PS4 from USB Host ");
      break;
    case WaitStart:
      if (CROSS_Pressed) {
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();  // May be "START_Pressed = false" is more relevant
      //Start Button may be 1 when PS4 is not connected to BlueRetro
      //Suggestion is to change Start Button to another button
      break;

    case CheckActionToDo:
      // Please Take note that the following code process Button in priority manner
      if (CIRCLE_Pressed) {
        OperatingState = CircleMove;
        Serial.println("User Select to Move In Circle");
      } else if (SQUARE_Pressed) {
        OperatingState = RectangleMove;
        Serial.println("User Select to Move In Rectangle");
      } else if (CROSS_Pressed) {
        OperatingState = CrossMove;
        Serial.println("User Select to Move In Rectangle");
      } else if (TRIANGLE_Pressed) {
        OperatingState = TriangleMove;
        Serial.println("User Select to Move In Triangle");
      } else if (UP_Pressed) {
        OperatingState = Move_Forward;
        Serial.println("User Select to Move Forward");
      } else if (DOWN_Pressed) {
        OperatingState = Move_Backward;
        Serial.println("User Select to Move Backward");
      } else if (LEFT_Pressed) {
        OperatingState = Move_Left;
        Serial.println("User Select to Move Left");
      } else if (RIGHT_Pressed) {
        OperatingState = Move_Right;
        Serial.println("User Select to Move Right");
      } else if (L1_Pressed) {
        OperatingState = Rotate_Left;
        Serial.println("User Select to Rotate Left");
      } else if (R1_Pressed) {
        OperatingState = Rotate_Right;
        Serial.println("User Select to Rotate Right");
      } else {
        ClearButtonStatus();  // In Action Checking Cycle, disregard some functional button
        Serial.println("User Select Non Action Button, Do nothing");
      }
      break;

    case CircleMove:
      LowerSpeed = mechanism_speed;
      LiftSpeed = 0;
      TransferSpeed = 0;
      TransferBackSpeed = 0;
      Pull_Enable = 0;
      Push_Enable = 0;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case RectangleMove:
      LowerSpeed = 0;
      LiftSpeed = 0;
      TransferSpeed = 0;
      TransferBackSpeed = 0;
      Pull_Enable = 0;
      Push_Enable = 1;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case CrossMove:
      LowerSpeed = 0;
      LiftSpeed = mechanism_speed;
      TransferSpeed = 0;
      TransferBackSpeed = 0;
      Pull_Enable = 0;
      Push_Enable = 0;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case TriangleMove:
      LowerSpeed = 0;
      LiftSpeed = 0;
      TransferSpeed = 0;
      TransferBackSpeed = 0;
      Pull_Enable = 1;
      Push_Enable = 0;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case Move_Forward:
      LowerSpeed = 0;
      LiftSpeed = 0;
      TransferSpeed = mechanism_speed;
      TransferBackSpeed = 0;
      Pull_Enable = 0;
      Push_Enable = 0;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case Move_Backward:
      LowerSpeed = 0;
      LiftSpeed = 0;
      TransferSpeed = 0;
      TransferBackSpeed = mechanism_speed;
      Pull_Enable = 0;
      Push_Enable = 0;
      OperatingState = CheckActionToDo;
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case Move_Left:
      if (CROSS_Pressed) {
        // Exit Left Move Operation
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case Move_Right:
      if (CROSS_Pressed) {
        // Exit Right Move Operation
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case Rotate_Left:
      if (CROSS_Pressed) {
        // Exit Left Move Operation
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    case Rotate_Right:
      if (CROSS_Pressed) {
        // Exit Right Move Operation
        OperatingState = CheckActionToDo;
      }
      ClearButtonStatus();  // Check Carefully if this is necessary
      break;

    default:  // For in case something go wrong, force user press Start Button again
      OperatingState = WaitStart;
      Serial.println("Warning : Serious Error Occur, Entering Unexpected state");
      ClearButtonStatus();  // Check Carefully if this is necessary
  }
}

void OutputTaskCode() {
  if (LiftSpeed > 0)              lift(LiftSpeed);
  else if (LowerSpeed > 0)        lower(LowerSpeed);
  else if (Pull_Enable)           pull();
  else if (Push_Enable)           push();
  else if (TransferSpeed > 0)     transfer(TransferSpeed);
  else if (TransferBackSpeed > 0) transfer_reverse(TransferBackSpeed);
  else MotorStopping();
}

//Mechanism
void lift(int motor_speed) {
  PORTA |= (1 << lifting_dir_pin);
  analogWrite(lifting_pwm_pin, motor_speed);
}

void lower(int motor_speed) {
  PORTA &= ~(1 << lifting_dir_pin);
  analogWrite(lifting_pwm_pin, motor_speed);
}

void pull() {
  PORTA |= (1 << loading_dir_pin);
}

void push() {
  PORTA &= ~(1 << loading_dir_pin);
}

void transfer(int motor_speed) {
  PORTA &= ~(1 << conveyor_dir_pin);
  analogWrite(conveyor_pwm_pin, motor_speed);
}

void transfer_reverse(int motor_speed) {
  PORTA |= (1 << conveyor_dir_pin);
  analogWrite(conveyor_pwm_pin, motor_speed);
}

void MotorStopping() {
  analogWrite(conveyor_pwm_pin, 0);
  analogWrite(lifting_pwm_pin, 0);
}


void setup() {
  Serial.begin(115200);

  OperatingState = USB_Detect_Hold;  // Set First State to wait user press Start button
  USB_Detected = false;
  // When Debugging, developer can point to any value of interest according to test case
  Sync_Basic_Task();
  PS4_Repeat_Init_Code();  // Attempt Scan for USB Host Repeatedly
  InputTask.enable();      // Enable Input Task to be monitored
  ProcessTask.enable();    // Enable Process Task to be monitored
  OutputTask.enable();     // Enable Output Task to be monitored
  DebugMessageTask.enable();
}

void loop() {
  PS4_Repeat_Init_Task.check();  // Keep Retry to check USB host connection for 1 second until connected (Refer to Variable.h)
  if (USB_Detected) {
    InputTask.check();    // Input Task fire threads every 10ms (Refer to Variable.h)
    ProcessTask.check();  // Process Task fire threads every 10ms (Refer to Variable.h)
    OutputTask.check();   // Output Task fire threads every 10ms (Refer to Variable.h)
  } else {
    Sync_Basic_Task();  // No USB detected, dont run anything, but keep syncing Input, Process and Output Task Interval.
  }
  DebugMessageTask.check();  // Debug Message Print fire threads every 1ms (Refer to Variable.h)
}

void Sync_Basic_Task() {
  // The following code is purposely to control when each task is being execute
  // Without the following code, program can still run, but programmer have no idea which Task will be execute first.
  InputTask.reset();    // Input Task Run from 10ms from this point onwards
  delay(3);             // Process Task Shall Execute 3ms after Input Task
  ProcessTask.reset();  // Process Task Run from 10ms from this point onwards
  delay(3);             // Output Task Shall Execute 3ms after Process Task
  OutputTask.reset();   // Output Task Run from 10ms from this point onwards
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
