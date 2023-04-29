#include <ros.h>
#include <geometry_msgs/Twist.h>

int l_en = 30;
int r_en = 31;
int rpwm = 4;
int lpwm = 3;


//ros node handle
ros::NodeHandle nh;
geometry_msgs::Twist msg;




void forward(){
  digitalWrite(r_en, HIGH);
  digitalWrite(l_en, HIGH); 
  analogWrite(rpwm, 255); 
  analogWrite(lpwm, 0); 
  
}

void backward(){
  digitalWrite(r_en, HIGH); 
  digitalWrite(l_en, HIGH); 
  analogWrite(rpwm, 0);
  analogWrite(lpwm, 255);
}

void no_movement(){
  digitalWrite(r_en, LOW);
  digitalWrite(l_en, LOW);
}

void messageCallback(const geometry_msgs::Twist& cmd_vel){
  linear_x = cmd_vel.linear.x;
  rotation = cmd_vel.angular.z;

  if (linear_x > 0){
    forward();
  }
  else if (linear_x < 0){
    backward(); 
  }

  if (rotation > 0){
    forward();
  }
  else if (rotation < 0){
    backward();
  }

  if (linear_x == 0){
    no_movment();
  }
}

ros::Subscriber <geometry_msgs::Twist> sub("/car/cmd_vel", &messageCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(9600); 
  // put your setup code here, to run once:
  pinMode(l_en, OUTPUT); 
  pinMode(r_en, OUTPUT); 
  pinMode(rpwm, OUTPUT); 
  pinMode(lpwm, OUTPUT);  
  digitalWrite(l_en, LOW); 
  digitalWrite(r_en, LOW);
  digitalWrite(rpwm, LOW);
  digitalWrite(lpwm, LOW);   
  
}

void loop() {
  // put your main code here, to run repeatedly:
//  Serial.println("Forward"); 
//  
//  delay(2000);
//
//  Serial.println("Stop");
//  
//  delay(2000); 
//  
//  Serial.println("Backward"); 
//  
//  delay(2000);
  while (!nh.connected()){
    {
      nh.spinOnce();
    }
      nh.spinOnce();
      delay(1);
  }
  
}
