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


enum
{
  USB_Detect_Hold,
  WaitStart,
  CheckActionToDo,
  CircleMove,
  RectangleMove,
  Move_Forward,
  Move_Backward,
  Move_Left,
  Move_Right
} state;
