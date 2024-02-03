// Differential Drive Version 1.1 - Kaegan Phillips (1/30/24)

#include <ros.h>
#include <std_msgs/String.h>
#include <Arduino.h>

/*----Left Motors Pins----*/

#define LF_DIR 13 // Left-Front

#define LM_DIR 12 // Left-Middle

#define LB_DIR 11 // Left-Back

#define L_PWM  10  // Left-PWM

/*----Right Motors Pins----*/

#define RF_DIR 9 // Right-Front

#define RM_DIR 8 // Right-Middle

#define RB_DIR 7 // Right-Back

#define R_PWM  6  // Right-PWM

//void writeMsgToLog(const std_msgs::String::ConstPtr& msg) {
//    ROS_INFO("Motor State:  %s", msg->data.c_str());
//
//}


// Side class declaration


class Side

{

private:

  int F_DIR, M_DIR, B_DIR, PWM;

public:

  // Side constructor

  Side(int F, int M, int B, int P) : F_DIR(F), M_DIR(M), B_DIR(B), PWM(P)

  {

    // set outputs

    pinMode(F_DIR, OUTPUT);

    pinMode(M_DIR, OUTPUT);

    pinMode(B_DIR, OUTPUT);

    pinMode(PWM,   OUTPUT);

  }

  // Side movement functions

  void forward();

  void reverse();

};

// function to rotate Side forward

void Side::forward()

{

  // HIGH for forward rotation

  digitalWrite(F_DIR, HIGH);

  digitalWrite(M_DIR, HIGH);

  digitalWrite(B_DIR, HIGH);

  analogWrite(PWM, 50);

}

// function to rotate Side backward

void Side::reverse()

{

  // LOW for reverse rotation

  digitalWrite(F_DIR, LOW);

  digitalWrite(M_DIR, LOW);

  digitalWrite(B_DIR, LOW);

  analogWrite(PWM, 50);

}



// Rover class declaration

class Rover

{

  private:

    // Side object instantiations

    Side leftSide;

    Side rightSide;

  public:

    // Rover constructor to pass motor pins

    Rover() : leftSide(LF_DIR, LM_DIR, LB_DIR, L_PWM),

              rightSide(RF_DIR, RM_DIR, RB_DIR, R_PWM)

    {}

    // Rover public movement functions

    void forward();

    void reverse();

    void turnLeft();

    void turnRight();

    void boost();

    void stop();

};

// function to move Rover forward

void Rover::forward()

{

  leftSide.forward();

  rightSide.forward();

}

// function to move Rover backward

void Rover::reverse()

{

  leftSide.reverse();

  rightSide.reverse();

}

// function to turn Rover left

void Rover::turnLeft()

{

  leftSide.forward();

  rightSide.reverse();

}

// function to turn Rover right

void Rover::turnRight()

{

  leftSide.reverse();

  rightSide.forward();

}

// function to stop Rover

void Rover::boost()

{

  leftSide.forward();

  rightSide.forward();

}

// function to stop Rover

void Rover::stop()

{

  leftSide.forward();

  rightSide.forward();

}

int input = 0;

void messageCb( const std_msgs::String& msg){
  input = msg.data;
}


ros::NodeHandle nh;
//ros::Subscriber state_sub = nh.subscribe("motor_contoller_publisher", 1000, writeMsgToLog);
ros::Subscriber<std_msgs::String> sub("motor_controller_publisher", &messageCb );



void setup() 

{

  // initialize serial communucation at 9600 baud

  Serial.begin(9600);

  Serial.println("Rover Control: Start");

  nh.initNode();
  nh.subscribe(sub);
  

}



// Rover instantiation

Rover rover;



// loop function to take input from serial interface


void loop() 

{
  nh.spinOnce();
  // initialize serial interface
    // read input

    

    // input validation

  if (input == '1' || input == '2' || input == '3' || input == '4' || input == '5')

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
