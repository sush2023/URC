// Arm Control - Kaegan Phillips (2/29/24)
#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include<string.h>

/*-----Arm Motors-----*/
#define M1_DIR 6
#define M2_DIR 7
#define M3_DIR 8
#define M4_DIR 10
#define M5_DIR 11
#define M6_DIR 12
#define M_PWM  9  // PWM for speed

// speed control constant
const int speedConst = 20.0; // normal speed
// global variables
int speed = 0;

// function to rotate motor forward
void forward(int motor)
{
  // stop motor
  stop();
  // set HIGH for forward rotation
  digitalWrite(motor, HIGH);
  // increase speed
  incrementSpeed();
}
// function to rotate motor backward
void reverse(int motor)
{
  // stop motor
  stop();
  // set LOW for backward rotation
  digitalWrite(motor, LOW);
  // increase speed
  incrementSpeed();
}
// function to stop all motors
void stop()
{
  // decrease speed
  while (speed > 0)
  {
    analogWrite(M_PWM, speed);
    speed--;
    delay(10);
  }
}
// function to increment speed
void incrementSpeed()
{
  while (speed < speedConst){
    analogWrite(M_PWM, speed);
    speed++;
    delay(10);
  }
  return;
}

/*-----ROS Setup-----*/
uint8_t input = 12;
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
  if (input >= 0 && input <= 12)
  {
    // if 0, motor1 forward
    if (input == 0){
      output_string = "M1 Forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward(M1_DIR);
    }
    // if 1, motor1 reverse
    if (input == 1){
      output_string = "M1 Reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse(M1_DIR);
    }
    // if 2, motor2 forward
    if (input == 2){
      output_string = "M2 Forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward(M2_DIR);
    }
    // if 3, motor2 reverse
    if (input == 3){
      output_string = "M2 Reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse(M2_DIR);
    }
    // if 4, motor3 forward
    if (input == 4){
      output_string = "M3 Forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward(M3_DIR);
    }
    // if 5, motor3 reverse
    if (input == 5){
      output_string = "M3 Reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse(M3_DIR);
    }
    // if 6, motor4 forward
    if (input == 6){
      output_string = "M4 Forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward(M4_DIR);
    }
    // if 7, motor4 reverse
    if (input == 7){
      output_string = "M4 Reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse(M4_DIR);
    }
    // if 8, motor5 forward
    if (input == 8){
      output_string = "M5 Forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward(M5_DIR);
    }
    // if 9, motor5 reverse
    if (input == 9){
      output_string = "M5 Reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse(M5_DIR);
    }
    // if 10, motor6 forward
    if (input == 10){
      output_string = "M6 Forward";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      forward(M6_DIR);
    }
    // if 11, motor6 reverse
    if (input == 11){
      output_string = "M6 Reverse";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      reverse(M6_DIR);
    }
    // if 12, all stop
    if (input == 12){
      output_string = "All Stop";
      output_string.toCharArray(o_string, output_string.length()+1);
      msg.data = o_string;
      stop();
    }
  }
}