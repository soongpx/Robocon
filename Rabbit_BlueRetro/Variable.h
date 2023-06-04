#include <AccelStepper.h>

PS2X ps2x;
AccelStepper stepper1(1, step1Pin, dir1Pin);
AccelStepper stepper2(1, step2Pin, dir2Pin);
 

//++++++++++++++++++++++++++++++++++++++ TimedAction Threads +++++++++++++++++++++++++++++++++++++++//
void InputTaskCode();
void ProcessTaskCode();
void OutputTaskCode();
void DebugMessageTaskCode();
void PS4_Repeat_Init_Code();
void StepperControlCode();
TimedAction InputTask = TimedAction(10, InputTaskCode); // fire threads every 10ms
TimedAction ProcessTask = TimedAction(10, ProcessTaskCode);
TimedAction OutputTask = TimedAction(10, OutputTaskCode);
TimedAction StepperControlTask = TimedAction(1, StepperControlCode);
TimedAction PS4_Repeat_Init_Task = TimedAction(1000, PS4_Repeat_Init_Code);
TimedAction DebugMessageTask = TimedAction(1, DebugMessageTaskCode);
String DebugMessage = "";

int UP         = 0;
int RIGHT      = 0;
int DOWN       = 0;
int LEFT       = 0;
int L1         = 0;
int R1         = 0;
int L2         = 0;
int R2         = 0;
int SQUARE     = 0;
int CIRCLE     = 0;
int TRIANGLE   = 0;
int CROSS      = 0;

bool USB_Detected      = false;
bool UP_Pressed        = false;
bool RIGHT_Pressed     = false;
bool DOWN_Pressed      = false;
bool LEFT_Pressed      = false;
bool L1_Pressed        = false;
bool R1_Pressed        = false;
bool L2_Pressed        = false;
bool R2_Pressed        = false;
bool SQUARE_Pressed    = false;
bool CIRCLE_Pressed    = false;
bool TRIANGLE_Pressed  = false;
bool CROSS_Pressed     = false;

bool prev_up = 0;
bool prev_down = 0;
bool prev_right = 0;
bool prev_tri = 0;
//++++++++++++++++++++++++++++++++++++++ Program State +++++++++++++++++++++++++++++++++++++++//
uint8_t OperatingState;
uint8_t LocoState;
uint8_t MechanismState;


//++++++++++++++++++++++++++++++++ Output Bool Switch & Value ++++++++++++++++++++++++++++++++++//
//PS4
int error = 0; 
byte type = 0;
byte vibrate = 0;

//Locomotion
int motor_speed           = 180;
int rotate_speed          = 100;
int16_t ForwardSpeed      =-1;
int16_t BackwardSpeed     =-1;
int16_t LeftSpeed         =-1;
int16_t RightSpeed        =-1;
int16_t RotateLeftSpeed   =-1;
int16_t RotateRightSpeed  =-1;
int ramping_counter       = 0;
bool Stop_Loco            = 0;

//Mechanism
bool tilt_up = 0;
bool tilt_down = 0;
uint8_t StepperAlternator = 0;
bool Stepper1Move = 0;
bool Stepper2Move = 0;
bool rotate = 0;
bool retract = 0;
bool shoot = 0;
bool keep = 0;

//IMU
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;

//PID
double Kp = 3.05, Ki = 0, Kd = 0.5;
double Input, Output,setpoint;
double yaw_angle;
int suss,fail;

//Stepper
bool cycle_finish = 0; 
//int distance1 = 100000;  // You Should Never try to use vague variable type inside MCU coding !!!!!!!!!!!!!!!
//int distance2 = -100000; // You Should Never try to use vague variable type inside MCU coding !!!!!!!!!!!!!!!
//void    moveTo(long absolute) // moveTo function prototype
int32_t distance1 = -31072; // This value is tested with arduino serial.println and get this funny value
int32_t distance2 = 31072;  // This value is tested with arduino serial.println and get this funny value
int pos;
