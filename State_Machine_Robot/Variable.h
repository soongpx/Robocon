USB Usb;
USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the PS4BT class in two ways */
// This will start an inquiry and then pair with the PS4 controller - you only have to do this once
// You will need to hold down the PS and Share button at the same time, the PS4 controller will then start to blink rapidly indicating that it is in pairing mode
PS4BT PS4(&Btd, PAIR);

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

bool USB_Detected = false;
bool UP_Pressed = false;
bool RIGHT_Pressed = false;
bool DOWN_Pressed = false;
bool LEFT_Pressed = false;
bool L1_Pressed = false;
bool R1_Pressed = false;
bool SQUARE_Pressed = false;
bool CIRCLE_Pressed = false;
bool TRIANGLE_Pressed = false;
bool CROSS_Pressed = false;
bool START_Pressed   =   false; 

//++++++++++++++++++++++++++++++++++++++ Program State +++++++++++++++++++++++++++++++++++++++//
uint8_t OperatingState;


//++++++++++++++++++++++++++++++++ Output Bool Switch & Value ++++++++++++++++++++++++++++++++++//
int16_t ForwardSpeed =-1;
int16_t BackwardSpeed =-1;
int16_t LeftSpeed =-1;
int16_t RightSpeed =-1;
