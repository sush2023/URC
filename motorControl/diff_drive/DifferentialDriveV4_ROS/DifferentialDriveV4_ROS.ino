// Differential Drive Motor Control : ROS Compatible (2/29/24)
#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include<string.h>

/*-----Left Motors-----*/
#define LF_DIR 12 // Left-Front
#define LM_DIR 11 // Left-Middle
#define LB_DIR 10 // Left-Back
/*-----Right Motors-----*/
#define RF_DIR 8 // Right-Front
#define RM_DIR 7 // Right-Middle
#define RB_DIR 6 // Right-Back
// Pulse-Width Modulation
#define M_PWM  9

// speed control constants
const int speedConst = 100;               // normal speed
const int turnConst  = 20; // turning speed (75%)
const int boostConst = (2 * speedConst)+30;    // boosted speed (200%) [CANNOT EXCEED 255]

// global variables
char direction = 'S'; // F:forward, B:backward, R:right, L:left, S:stop
int  speed = 0;

// function to move Rover forward
void forward()
{
  // if forward, clear boost and return
  if (direction == 'F'){
    decrementSpeed();
    return;
  } 
  else stop();
  // set all HIGH for forward rotation
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LM_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RM_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);
  // increase speed
  incrementSpeed();
  // set direction
  direction = 'F';
}
// function to move Rover backward
void reverse()
{
  // if backward, clear boost and return
  if (direction == 'B'){
    decrementSpeed();
    return;
  } 
  else stop();
  // set all LOW for backward rotation
  digitalWrite(LF_DIR, LOW);
  digitalWrite(LM_DIR, LOW);
  digitalWrite(LB_DIR, LOW);
  digitalWrite(RF_DIR, LOW);
  digitalWrite(RM_DIR, LOW);
  digitalWrite(RB_DIR, LOW);
  // increase speed
  incrementSpeed();
  // set direction
  direction = 'B';
}
// function to turn Rover left
void left()
{
  // if left, clear boost and return
  if (direction == 'L'){
    decrementSpeed();
    return;
  } 
  else stop();
  // set left HIGH
  digitalWrite(LF_DIR, HIGH);
  digitalWrite(LM_DIR, HIGH);
  digitalWrite(LB_DIR, HIGH);
  // and right LOW
  digitalWrite(RF_DIR, LOW);
  digitalWrite(RM_DIR, LOW);
  digitalWrite(RB_DIR, LOW);
  // increase speed
  incrementSpeed();
  // set direction
  direction = 'L';
}
// function to turn Rover right
void right()
{
  // if right, clear boost and return
  if (direction == 'R'){
    decrementSpeed();
    return;
  }
  else stop();
  // set left LOW
  digitalWrite(LF_DIR, LOW);
  digitalWrite(LM_DIR, LOW);
  digitalWrite(LB_DIR, LOW);
  // and right HIGH
  digitalWrite(RF_DIR, HIGH);
  digitalWrite(RM_DIR, HIGH);
  digitalWrite(RB_DIR, HIGH);
  // increase speed
  incrementSpeed();
  // set direction
  direction = 'R';
}
// function to boost Rover
void boost()
{
  // do not boost if stopped
  if (direction == 'S' || direction == 'R' || direction == 'L') return;
  // increase speed
  while (speed < boostConst)
  {
    analogWrite(M_PWM, speed);
    speed++;
    delay(10);
  }
}
// function to stop Rover
void stop()
{
  // decrease speed
  while (speed > 0)
  {
    analogWrite(M_PWM, speed);
    speed--;
    delay(10);
  }
  // set direction
  direction = 'S';
}
// function to increment speed
void incrementSpeed()
{
  int desiredSpeed = (direction == 'F' || direction == 'B') ? speedConst : turnConst;
  while (speed < desiredSpeed){
    analogWrite(M_PWM, speed);
    speed++;
    delay(10);
  }
  return;
}
// function to decrement speed
void decrementSpeed()
{
  int desiredSpeed = (direction == 'F' || direction == 'B') ? speedConst : turnConst;
  while (speed > desiredSpeed){
    analogWrite(M_PWM, speed);
    speed--;
    delay(10);
  }
  return;
}

/*-----ROS Setup-----*/
uint8_t input = 0;
ros::NodeHandle nh;

void messageCb( const std_msgs::UInt8& toggle_msg){
//  Serial.println("Message is: " );
//  Serial.println(toggle_msg.data);
  input = toggle_msg.data;
}
ros::Subscriber<std_msgs::UInt8> sub("motor_controller_publisher", &messageCb );

std_msgs::String msg;
ros::Publisher topic_pub("MotorRosPublisher", &msg);

void setup() 
{
  // initialize serial communucation at 115200 baud
  Serial.begin(115200);
  Serial.println("Rover Control: Start");

  nh.initNode();
  //nh.getHardware()->setBaud(115200);
  nh.subscribe(sub);
  nh.advertise(topic_pub);
}
// loop function to take input
void loop() 
{
  String output_string = "NO";
  char o_string[20];
  // input validation
  if (input == 1 || input == 2 || input == 3 || input == 4 || input == 5 || input == 6)
  {
    // if 1 go forward
    if (input == 1)
    {
      output_string = "Moving forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward();
    }
    // if 2 go backward
    else if (input == 2)
    {
      output_string = "Moving reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse();
    }
    // if 3 turn left
    else if (input == 3)
    {
      output_string = "Turn Left";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      left();
    }
    // if 4 turn right
    else if (input == 4)
    {
      output_string = "Turn Right";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      right();
    }
    // if 5 boost
    else if (input == 5)
    {
      output_string = "Boost!";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      boost();
    }
    // if 6 stop
    else if (input == 6)
    {
      output_string = "STOP!";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      stop();
    }
    topic_pub.publish(&msg);
  }
  nh.spinOnce();
}
