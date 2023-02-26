
//8 is IN1
//9 is IN2
int in1 = 8;
int in2 = 9;

void setup() {
  Serial.begin(9600); 
  // put your setup code here, to run once:
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT);   
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Forward"); 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW); 
  delay(2000);

  Serial.print("Backward"); 
  digitalWrite(in1, LOW); 
  digitalWrite(in2, HIGH); 
  delay(2000);
  
}
