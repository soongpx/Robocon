PS2X ps2x;

//++++++++++++++++++++++++++++++++++++++ TimedAction Threads +++++++++++++++++++++++++++++++++++++++//
void InputTaskCode();
void ProcessTaskCode();
void OutputTaskCode();
void DebugMessageTaskCode();
void PS4_Repeat_Init_Code();

TimedAction InputTask = TimedAction(10, InputTaskCode); // fire threads every 10ms
TimedAction ProcessTask = TimedAction(10, ProcessTaskCode);
TimedAction OutputTask = TimedAction(10, OutputTaskCode);
TimedAction PS4_Repeat_Init_Task = TimedAction(1000, PS4_Repeat_Init_Code);
TimedAction DebugMessageTask = TimedAction(1, DebugMessageTaskCode);
String DebugMessage = "";

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
bool START_Pressed     = false; 

//++++++++++++++++++++++++++++++++++++++ Program State +++++++++++++++++++++++++++++++++++++++//
uint8_t OperatingState;


//++++++++++++++++++++++++++++++++ Output Bool Switch & Value ++++++++++++++++++++++++++++++++++//
//PS4
int error = 0; 
byte type = 0;
byte vibrate = 0;

//Locomotion
int16_t ForwardSpeed      =-1;
int16_t BackwardSpeed     =-1;
int16_t LeftSpeed         =-1;
int16_t RightSpeed        =-1;
int16_t RotateLeftSpeed   =-1;
int16_t RotateRightSpeed  =-1;
int ramping_counter       = 0;

//IMU
unsigned char Re_buf[11], counter = 0;
unsigned char sign = 0;
int suss,fail;
double yaw_angle = 0, Input = 0;
int FrontOutput = 0, BackOutput = 0, LeftOutput = 0, RightOutput = 0;