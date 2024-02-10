// Differential Drive Version 1.2 - Kaegan Phillips (2/6/24)
// ROS Compatible
#include <Arduino.h>
#include <std_msgs/String.h>
#include <ros.h>

/*-----Left Motors-----*/
#define LF_DIR 13 // Left-Front
#define LM_DIR 12 // Left-Middle
#define LB_DIR 11 // Left-Back
#define L_PWM  10  // Left-PWM
/*-----Right Motors-----*/
#define RF_DIR 9 // Right-Front
#define RM_DIR 8 // Right-Middle
#define RB_DIR 7 // Right-Back
#define R_PWM  6  // Right-PWM

// speed control constants
const float speedConst = 100.0; // normal speed
const int   boostMult  = 2;     // (2 * speedConst) : total cannot exceed 255

/*-----Side class declaration-----*/
class Side 
{
private:
  int F_DIR, M_DIR, B_DIR, PWM; // pin assignments
  char direction = 'S'; // current direction
  float speed = 0.00;    // current speed
public:
  // Side constructor
  Side(int F, int M, int B, int P) : F_DIR(F), M_DIR(M), B_DIR(B), PWM(P)
  {
    // set pins as outputs
    pinMode(F_DIR, OUTPUT);
    pinMode(M_DIR, OUTPUT);
    pinMode(B_DIR, OUTPUT);
    pinMode(PWM,   OUTPUT);
  }
  // movement functions
  void forward();
  void reverse();
  void boost();
  void stop();
  // mutator functions
  void setDirection(char dir);
};
// function to rotate Side forward
void Side::forward()
{
  // if forward, check for boost and return
  if (direction == 'F'){
    while (speed > speedConst){
      analogWrite(PWM, speed);
      speed = speed - 0.01;
    }
    return;
  }
  // if reverse, slow to a stop
  if (direction == 'R'){
    while (speed > 0){
      analogWrite(PWM, speed);
      speed = speed - 0.01;
    }
    // set direction to stop once stopped
    direction = 'S';
  }
  // set HIGH for forward rotation
  digitalWrite(F_DIR, HIGH);
  digitalWrite(M_DIR, HIGH);
  digitalWrite(B_DIR, HIGH);
  // increase speed
  if (direction == 'S'){
    while (speed < speedConst){
      analogWrite(PWM, speed);
      speed = speed + 0.01;
    }
  }
}
// function to rotate Side backward
void Side::reverse()
{
  // if reverse, check for boost and return
  if (direction == 'R'){
    while (speed > speedConst){
      analogWrite(PWM, speed);
      speed = speed - 0.01;
    }
    return;
  }
  // if forward, slow to a stop
  if (direction == 'F'){
    while (speed > 0){
      analogWrite(PWM, speed);
      speed = speed - 0.01;
    }
    // set direction to stop once stopped
    direction = 'S';
  }
  // set LOW for reverse rotation
  digitalWrite(F_DIR, LOW);
  digitalWrite(M_DIR, LOW);
  digitalWrite(B_DIR, LOW);
  // increase speed
  if (direction == 'S'){
    while (speed < speedConst){
      analogWrite(PWM, speed);
      speed = speed + 0.01;
    }
  }
}
// function to boost Side
void Side::boost()
{
  // do not boost if stopped
  if (direction == 'S'){
    return;
  }
  // increase speed
  while (speed < (speedConst * boostMult))
  {
    analogWrite(PWM, speed);
    speed = speed + 0.01;
  }
}
// function to stop Side
void Side::stop()
{
  // decrease speed
  while (speed > 0)
  {
    analogWrite(PWM, speed);
    speed = speed - 0.01;
  }
}
// function to set direction variable
void Side::setDirection(char dir)
{
  // S:stop, F:forward, R:reverse
  direction = dir;
}

/*-----Rover class declaration-----*/
class Rover
{
private:
  Side leftSide;  // left motor control
  Side rightSide; // right motor control
public:
  // Rover constructor to pass motor pins
  Rover() : leftSide(LF_DIR, LM_DIR, LB_DIR, L_PWM),
            rightSide(RF_DIR, RM_DIR, RB_DIR, R_PWM) 
  {
    // initialize each side as stopped
    leftSide.setDirection('S');
    rightSide.setDirection('S');
  }
  // Rover movement functions
  void forward();
  void reverse();
  void left();
  void right();
  void boost();
  void stop();
};
// function to move Rover forward
void Rover::forward()
{
  leftSide.forward();
  leftSide.setDirection('F');
  rightSide.forward();
  rightSide.setDirection('F');
}
// function to move Rover backward
void Rover::reverse()
{
  leftSide.reverse();
  leftSide.setDirection('R');
  rightSide.reverse();
  rightSide.setDirection('R');

}
// function to turn Rover left
void Rover::left()
{
  leftSide.forward();
  leftSide.setDirection('F');
  rightSide.reverse();
  rightSide.setDirection('R');
}
// function to turn Rover right
void Rover::right()
{
  leftSide.reverse();
  leftSide.setDirection('R');
  rightSide.forward();
  rightSide.setDirection('F');
}
// function to boost Rover
void Rover::boost()
{
  leftSide.boost();
  rightSide.boost();
}
// function to stop Rover
void Rover::stop()
{
  leftSide.stop();
  leftSide.setDirection('S');
  rightSide.stop();
  rightSide.setDirection('S');
}

/*-----ROS Setup-----*/
int input = 0;
void messageCb( const std_msgs::String& msg)
{
  input = msg.data;
}
ros::NodeHandle nh;
ros::Subscriber<std_msgs::String> sub("motor_controller_publisher", &messageCb );
// setup function to initialize communications
void setup() 
{
  // initialize serial communucation at 115200 baud
  Serial.begin(115200);
  Serial.println("Rover Control: Start");

  nh.initNode();
  nh.subscribe(sub);
}
// Rover instantiation
Rover rover;
// loop function to take input
void loop() 
{
  // input validation
  if (input == '1' || input == '2' || input == '3' || input == '4' || input == '5' || input == '6')
  {
    // if 1 go forward
    if (input == '1')
    {
      rover.forward();
    }
    // if 2 go backward
    if (input == '2')
    {
      rover.reverse();
    }
    // if 3 turn left
    if (input == '3')
    {
      rover.turnLeft();
    }
    // if 4 turn right
    if (input == '4')
    {
      rover.turnRight();
    }
    // if 5 boost
    if (input == '5')
    {
      rover.boost();
    }
    // if 6 stop
    if (input == '6')
    {
      rover.stop();
    }
  }
}