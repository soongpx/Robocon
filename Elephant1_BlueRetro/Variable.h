#include <Servo.h>

Servo Servo1;
Servo Servo2;
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
TimedAction PS4_Repeat_Init_Task = TimedAction(1000, PS4_Repeat_Init_Code);
TimedAction DebugMessageTask = TimedAction(1, DebugMessageTaskCode);
String DebugMessage = "";

#define PS4_SHARE                 0x01
#define PS4_UNKNOWN1              0x02
#define PS4_UNKNOWN2              0x04
#define PS4_OPTION                0x08
#define DPAD_UP                   0x10
#define DPAD_RIGHT                0x20
#define DPAD_DOWN                 0x40
#define DPAD_LEFT                 0x80
#define L2_PRESSED                0x01
#define R2_PRESSED                0x02
#define L1_PRESSED                0x04
#define R1_PRESSED                0x08
#define GREEN_TRIANGLE_PRESSED    0x10
#define RED_CIRCLE_PRESSED        0x20
#define BLUE_CROSS_PRESSED        0x40
#define PINK_SQUARE_PRESSED       0x80

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

int check = 0;
//++++++++++++++++++++++++++++++++++++++ Program State +++++++++++++++++++++++++++++++++++++++//
uint8_t OperatingState;
uint8_t LocoState;
uint8_t MechanismState;


//++++++++++++++++++++++++++++++++ Output Bool Switch & Value ++++++++++++++++++++++++++++++++++//
//PS4
uint8_t SPI_Packet[BufferSize] = {0};
static byte ReadAllData[] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static byte Enter_Config[] = {0x01, 0x43, 0x00, 0x01, 0x00};
static byte Turn_ON_Analog_Mode[] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
static byte Maps_Motor[] = {0x01, 0x4D, 0x00, 0x00, 0x01};
static byte Exit_Config[] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

//Mechanism
bool pneu = 0;
bool ser = 0;
bool keep = 0;
bool lift_up = 0;
bool lift_down = 0;
bool conveyor_move = 0;
int servoAngle = 0;

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
