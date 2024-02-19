// Differential Drive Version 1.1 - Kaegan Phillips (1/30/24)



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

void setup() 
{
  Serial.begin(115200);

  Serial.println("Rover Control: Start");

}



// Rover instantiation

Rover rover;



// loop function to take input from serial interface

void loop() 

{

  // initialize serial interface

  if (Serial.available() > 0)

  { 

    // read input

    String input = Serial.readString();
    Serial.println(input);

    // input validation

    if (input == "foward" || input == "backward" || input ==  "left" || input == "right" || input == "boost" || input == "stop")
    {
      if (input == "foward")
      {
        
        rover.forward();
      }
      if (input == "backward")
      {
        rover.reverse();
      }
      if (input == "left")
      {
        rover.turnLeft();
      }
      if (input == "right")
      {
        rover.turnRight();
      }
      if (input == "boost")
      {
        rover.boost();
      }
      if (input == "stop")
      {
        rover.stop();
      }

    }

  }

}
