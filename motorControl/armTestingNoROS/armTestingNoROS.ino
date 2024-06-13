// Arm Testing Control - Kaegan Phillips (2/22/24)
// ROS Compatible
#include <Arduino.h>
#include <std_msgs/String.h>
#include <ros.h>

/*-----Arm Motors-----*/
#define M1_DIR 13
#define M2_DIR 12
#define M3_DIR 11
#define M4_DIR 10
#define M5_DIR 9
#define M6_DIR 8
#define M_PWM  6  // PWM for speed

// speed control constants
const float speedConst = 50.0; // normal speed

/*-----Motor class declaration-----*/
class Motor 
{
private:
  int M_DIR, PWM;        // pin assignments
  char direction = 'S';  // current direction
  float speed = 0.00;    // current speed
public:
  // Side constructor
  Side(int M, int P) : M_DIR(M), PWM(P)
  {
    // set pins as outputs
    pinMode(M_DIR, OUTPUT);
    pinMode(PWM,   OUTPUT);
  }
  // movement functions
  void forward();
  void reverse();
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

// motor instantiations
Motor motor1(M1_DIR, M_PWM);
Motor motor2(M2_DIR, M_PWM);
Motor motor3(M3_DIR, M_PWM);
Motor motor4(M4_DIR, M_PWM);
Motor motor5(M5_DIR, M_PWM);
Motor motor6(M6_DIR, M_PWM);

// loop function to take input from serial interface
void loop() 
{
  // input validation
  if (input >= 0 && input <= 12)
  {
    // if 0, motor1 forward
    if (input == 0)
    {
      motor1.forward();
      motor1.setDirection('F');
    }
    // if 1, motor1 reverse
    if (input == 1)
    {
      motor1.reverse();
      motor1.setDirection('R');
    }

    // if 2, motor2 forward
    if (input == 2)
    {
      motor2.forward();
      motor2.setDirection('F');
    }
    // if 3, motor2 reverse
    if (input == 3)
    {
      motor2.reverse();
      motor2.setDirection('R');
    }

    // if 4, motor1 forward
    if (input == 4)
    {
      motor3.forward();
      motor3.setDirection('F');
    }
    // if 5, motor1 reverse
    if (input == 5)
    {
      motor3.reverse();
      motor3.setDirection('R');
    }

    // if 6, motor1 forward
    if (input == 6)
    {
      motor4.forward();
      motor4.setDirection('F');
    }
    // if 7, motor1 reverse
    if (input == 7)
    {
      motor4.reverse();
      motor4.setDirection('R');
    }

    // if 8, motor1 forward
    if (input == 8)
    {
      motor5.forward();
      motor5.setDirection('F');
    }
    // if 9, motor1 reverse
    if (input == 9)
    {
      motor5.reverse();
      motor5.setDirection('R');
    }

    // if 10, motor1 forward
    if (input == 10)
    {
      motor6.forward();
      motor6.setDirection('F');
    }
    // if 11, motor1 reverse
    if (input == 11)
    {
      motor6.reverse();
      motor6.setDirection('R');
    }

    // if 12, all stop
    if (input == 12)
    {
      motor1.stop();
      motor1.setDirection('S');
      motor2.stop();
      motor2.setDirection('S');
      motor3.stop();
      motor3.setDirection('S');
      motor4.stop();
      motor4.setDirection('S');
      motor5.stop();
      motor5.setDirection('S');
      motor6.stop();
      motor6.setDirection('S');
    }
  }
}