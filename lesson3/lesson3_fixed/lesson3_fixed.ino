#include <Servo.h>
#define SERVO_PIN 11
Servo head;


// IR Sensors
int sensor1 = A0;      // Left most sensor
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;      // Right most sensor
int sensor5 = A4;      // Right most sensor

 

// Motor Variables
#define ENA 5 
#define ENB 6  
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 10


// int ENB =2;
 //int ENA =3;
#define LOW_SPEED 65
#define MID_SPEED 85
#define HIGH_SPEED 115
#define DELAY_TIME 100
 
 
 
void setup()
{
  int error=0;
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENA, OUTPUT);
  analogWrite(ENB, 0); //Left Motor Speed
  analogWrite(ENA, 0); //Right Motor Speed
  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  
  //init servo
  head.attach(SERVO_PIN);
  head.write(110);

  Serial.println("Started !!");
  /*
  forward();
  analogWrite(ENB, 90); //Left Motor Speed
  analogWrite(ENA, 90); //Right Motor Speed
  delay(500);
  reverse();
  analogWrite(ENB, 90); //Left Motor Speed
  analogWrite(ENA, 90); //Right Motor Speed
  delay(500);
  left();
  analogWrite(ENB, 0); //Left Motor Speed
  analogWrite(ENA, 90); //Right Motor Speed
  delay(500);
  right();
  analogWrite(ENB, 90); //Left Motor Speed
  analogWrite(ENA, 0); //Right Motor Speed
  delay(500);
    stop_bot(); 
    */
}
void loop()
{
 tracking();
}

 
void tracking()
{
   String senstr="";
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);
  int sensorvalue=32;
  sensorvalue +=s0*16+s1*8+s2*4+s3*2+s4;
  senstr= String(sensorvalue,BIN);
  senstr=senstr.substring(1,6);
  
  Serial.print(senstr);
  Serial.print("\t");
  if (senstr=="10000")
  {  
    Serial.println("Sharp Left2");
    sharpLeftTurn();
    analogWrite(ENB, LOW_SPEED); //Left Motor Speed
    analogWrite(ENA, MID_SPEED); //Right Motor Speed
    delay(DELAY_TIME);
    stop_bot();
  }
  if ( senstr=="01000" || senstr=="11000")
   {
     Serial.println("Sharp Left1");
      sharpLeftTurn();
      analogWrite(ENB, LOW_SPEED); //Left Motor Speed
      analogWrite(ENA, LOW_SPEED); //Right Motor Speed
      delay(DELAY_TIME);
      stop_bot();     
   }
   
  if ( senstr=="11100" || senstr=="10100" )
  {
     Serial.println("Left");
      left();
      analogWrite(ENB, 0); //Left Motor Speed
      analogWrite(ENA, HIGH_SPEED); //Right Motor Speed
      delay(DELAY_TIME);
      stop_bot(); 
  }
  if ( senstr=="01100" ||  senstr=="11110"  || senstr=="10010"  || senstr=="10110"  || senstr=="11010")
  {
     Serial.println("Slit Left");
      forward();
      analogWrite(ENB, MID_SPEED); //Left Motor Speed
      analogWrite(ENA, HIGH_SPEED); //Right Motor Speed
  }
 if (senstr=="01110" || senstr=="01010" || senstr=="00100"  || senstr=="10001"  || senstr=="10101"  || senstr=="10011" || senstr=="11101" || senstr=="10111" || senstr=="11011"  || senstr=="11001")
  {
     Serial.println("Forward");
      forward();
      analogWrite(ENB, HIGH_SPEED); //Left Motor Speed
      analogWrite(ENA, HIGH_SPEED); //Right Motor Speed
  }
 if ( senstr=="00110" || senstr=="01111" || senstr=="01001" || senstr=="01011" || senstr=="01101")
  {
        Serial.println("Slit Right");
      forward();
      analogWrite(ENB, HIGH_SPEED); //Left Motor Speed
      analogWrite(ENA, MID_SPEED); //Right Motor Speed
  }
 if (senstr=="00111" || senstr=="00101" )
  {    Serial.println("Right");
      right();
      analogWrite(ENB, HIGH_SPEED); //Left Motor Speed
      analogWrite(ENA, 0); //Right Motor Speed
      delay(DELAY_TIME);
      stop_bot(); 
  }
 if (senstr=="00010" || senstr=="00011")
 {
   Serial.println("Sharp Right1");
      sharpRightTurn();
      analogWrite(ENB, LOW_SPEED); //Left Motor Speed
      analogWrite(ENA, LOW_SPEED); //Right Motor Speed
      delay(DELAY_TIME);
      stop_bot();     
 }
 if (senstr=="00001" ||  senstr=="00000" || senstr=="11111")
 {
   Serial.println("Sharp Right2");
      sharpRightTurn();
      analogWrite(ENB, MID_SPEED); //Left Motor Speed
      analogWrite(ENA, LOW_SPEED); //Right Motor Speed
      delay(DELAY_TIME);
      stop_bot();     
 }
 
 
 
   
}

 

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0); //Left Motor Speed
  analogWrite(ENA, 0); //Right Motor Speed
}
