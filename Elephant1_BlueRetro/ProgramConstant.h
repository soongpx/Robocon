//Mechanism
#define lifting_pwm_pin 3
#define lifting_dir_pin 1
#define lifting_speed 255 

#define loading_dir_pin 2

#define conveyor_pwm_pin 2
#define conveyor_dir_pin 0

#define mechanism_speed 120

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
  CrossMove,
  TriangleMove,
  Move_Forward,
  Move_Backward,
  Move_Left,
  Move_Right,
  Rotate_Left,
  Rotate_Right
} state;
