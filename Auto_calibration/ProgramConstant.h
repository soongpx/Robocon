//Locomotion
#define wheel1 2 //upper left
#define wheel2 3 //upper right
#define wheel3 4 //lower left
#define wheel4 5 //lower right
#define dir_1 0 //upper left direction
#define dir_2 1 //upper right direction
#define dir_3 2 //lower left direction
#define dir_4 3 //lower right direction
#define motor_speed 120

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
