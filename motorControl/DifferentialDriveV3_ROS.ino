// Differential Drive Version 1.3 - Kaegan Phillips (2/28/24)
// ROS Compatible
#include <Arduino.h>
#include <std_msgs/String.h>
#include <ros.h>

/*-----Left Motors-----*/
#define LF_DIR 13 // Left-Front
#define LM_DIR 12 // Left-Middle
#define LB_DIR 11 // Left-Back
/*-----Right Motors-----*/
#define RF_DIR 9 // Right-Front
#define RM_DIR 8 // Right-Middle
#define RB_DIR 7 // Right-Back
// Pulse-Width Modulation
#define PWM  6

// speed control constants
const int speedConst = 100.0;             // normal speed
const int boostConst = (2 * speedConst);  // boost total cannot exceed 255

// global variables
char direction = 'S'; // F:forward, B:backward, R:right, L:left, S:stop
int  speed = 0;

// function to move Rover forward
void forward()
{
  // if forward, clear boost and return
  if (direction == 'F'){
    while (speed > speedConst){
      analogWrite(PWM, speed);
      speed--;
    }
    return;
  } 
  else // slow to a stop
  {
    while (speed > 0){
      analogWrite(PWM, speed);
      speed--;
    }
  }
  // set all HIGH for forward rotation
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LM_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RM_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);
  // increase speed
  while (speed < speedConst){
    analogWrite(PWM, speed);
    speed++;
  }
  direction = 'F';
}
// function to move Rover backward
void reverse()
{
  // if backward, clear boost and return
  if (direction == 'B'){
    while (speed > speedConst){
      analogWrite(PWM, speed);
      speed--;
    }
    return;
  } 
  else // slow to a stop
  {
    while (speed > 0){
      analogWrite(PWM, speed);
      speed--;
    }
  }
  // set all LOW for backward rotation
  digitalWrite(LF_DIR, LOW);
  digitalWrite(LM_DIR, LOW);
  digitalWrite(LB_DIR, LOW);
  digitalWrite(RF_DIR, LOW);
  digitalWrite(RM_DIR, LOW);
  digitalWrite(RB_DIR, LOW);
  // increase speed
  while (speed < speedConst){
    analogWrite(PWM, speed);
    speed++;
  }
  direction = 'B';
}
// function to turn Rover left
void left()
{
  // if left, clear boost and return
  if (direction == 'L'){
    while (speed > speedConst){
      analogWrite(PWM, speed);
      speed--;
    }
    return;
  } 
  else // slow to a stop
  {
    while (speed > 0){
      analogWrite(PWM, speed);
      speed--;
    }
  }
  // set left HIGH
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LM_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  // and right LOW
  digitalWrite(RF_DIR, LOW);
  digitalWrite(RM_DIR, LOW);
  digitalWrite(RB_DIR, LOW);
  // increase speed
  while (speed < speedConst){
    analogWrite(PWM, speed);
    speed++;
  }
  direction = 'L';
}
// function to turn Rover right
void right()
{
  // if right, clear boost and return
  if (direction == 'R'){
    while (speed > speedConst){
      analogWrite(PWM, speed);
      speed--;
    }
    return;
  } 
  else // slow to a stop
  {
    while (speed > 0){
      analogWrite(PWM, speed);
      speed--;
    }
  }
  // set left LOW
  digitalWrite(LF_DIR, LOW);
  digitalWrite(LM_DIR, LOW);
  digitalWrite(LB_DIR, LOW);
  // and right HIGH
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RM_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);
  // increase speed
  while (speed < speedConst){
    analogWrite(PWM, speed);
    speed++;
  }
  direction = 'R';
}
// function to boost Rover
void boost()
{
  // do not boost if stopped
  if (direction == 'S') return;
  // increase speed
  while (speed < boostConst)
  {
    analogWrite(PWM, speed);
    speed++;
  }
}
// function to stop Rover
void stop()
{
  // decrease speed
  while (speed > 0)
  {
    analogWrite(PWM, speed);
    speed--;
  }
  direction = 'S';
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
// loop function to take input
void loop() 
{
  // input validation
  if (input == '1' || input == '2' || input == '3' || input == '4' || input == '5' || input == '6')
  {
    // if 1 go forward
    if (input == '1')
    {
      forward();
    }
    // if 2 go backward
    if (input == '2')
    {
      reverse();
    }
    // if 3 turn left
    if (input == '3')
    {
      left();
    }
    // if 4 turn right
    if (input == '4')
    {
      right();
    }
    // if 5 boost
    if (input == '5')
    {
      boost();
    }
    // if 6 stop
    if (input == '6')
    {
      stop();
    }
  }
}