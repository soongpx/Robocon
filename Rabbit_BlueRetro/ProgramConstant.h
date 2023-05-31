//Locomotion
#define wheel1 2 //upper left
#define wheel2 3 //upper right
#define wheel3 4 //upper left
#define wheel4 5 //upper right
#define dir_1 0 //upper left direction
#define dir_2 1 //upper right direction
#define dir_3 2 //upper left direction
#define dir_4 3 //upper right direction

//Mechanism
#define tilt_pwm 44
#define tilt_dir 30
#define flywheel1_pwm 13
#define flywheel1_dir 29
#define flywheel2_pwm 12
#define flywheel2_dir 28
#define pneumatic 27
#define step1Pin 7 //Stepper 1 step pin
#define dir1Pin 6 //Stepper 1 direction pin
#define step2Pin 10 //Stepper 2 step pin
#define dir2Pin 11 //Stepper 2 direction pin
#define sampletime 0 //sample time for PID in ms

//PS4
//right now, the library does NOT support hot-pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
#define PS2X_DEBUG
#define PS2X_COM_DEBUG


enum
{
  USB_Detect_Hold,
  WaitStart,
  CheckActionToDo,
  CircleMove,
  RectangleMove,
  TriangleMove,
  Move_Forward,
  Move_Backward,
  Move_Left,
  Move_Right,
  Rotate_Left,
  Rotate_Right
} state;
