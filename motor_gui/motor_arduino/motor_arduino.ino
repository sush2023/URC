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
  void forward(float speed);
  void reverse(float speed);
};
// function to rotate Side forward
void Side::forward(float speed)
{
  // HIGH for forward rotation
  digitalWrite(F_DIR, HIGH);
  digitalWrite(M_DIR, HIGH);
  digitalWrite(B_DIR, HIGH);
  analogWrite(PWM, speed);
}
// function to rotate Side backward
void Side::reverse(float speed)
{
  // LOW for reverse rotation
  digitalWrite(F_DIR, LOW);
  digitalWrite(M_DIR, LOW);
  digitalWrite(B_DIR, LOW);
  analogWrite(PWM, speed);
}

// Rover class declaration
class Rover
{
  private:
    // Side object instantiations
    Side leftSide;
    Side rightSide;
    // Rover speed
    float speed = 50.0;
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
  leftSide.forward(speed);
  rightSide.forward(speed);
}
// function to move Rover backward
void Rover::reverse()
{
  leftSide.reverse(speed);
  rightSide.reverse(speed);
}
// function to turn Rover left
void Rover::turnLeft()
{
  leftSide.forward(speed);
  rightSide.reverse(speed);
}
// function to turn Rover right
void Rover::turnRight()
{
  leftSide.reverse(speed);
  rightSide.forward(speed);
}
// function to boost Rover
void Rover::boost()
{
  // incremented boost
  while (speed < 200)
  {
    analogWrite(L_PWM, speed);
    analogWrite(R_PWM, speed);
    speed += 0.01;
  }
}
// function to stop Rover
void Rover::stop()
{
  // incremented stop
  while (speed > 0)
  {
    analogWrite(L_PWM, speed);
    analogWrite(R_PWM, speed);
    speed -= 0.01;
  }
}

void setup() 
{
  // initialize serial communucation at 115200 baud
  Serial.begin(115200);
  Serial.println("Rover Control: Start");
}

// Rover instantiation
Rover rover;

// loop function to take input from serial interface
void loop() 
{
  // input validation
  if (Serial.available() > 0)

  { 
    int input = Serial.readString().toInt();
    Serial.println(input);
    if (input == 1 || input == 2 || input == 3 || input == 4 || input == 5 || input == 6)
    {
      // if 1 go forward
      if (input == 1)
      {
        Serial.println("Forward");
        rover.forward();
      }
      // if 2 go backward
      if (input == 2)
      {
        Serial.println("Backwards");
        rover.reverse();
      }
      // if 3 turn left
      if (input == 3)
      {
        Serial.println("Left");
        rover.turnLeft();
      }
      // if 4 turn right
      if (input == 4)
      {
        Serial.println("Right");
        rover.turnRight();
      }
      // if 5 boost
      if (input == 5)
      {
        Serial.println("Speed Up");
        rover.boost();
      }
      // if 6 stop
      if (input == 6)
      {
        Serial.print("Stop");
        rover.stop();
      }
    }
  }
}