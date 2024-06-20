#include <Servo.h>
#define SERVO_PIN 11
#define LPT 2 // scan loop coumter
#define BUZZ_PIN 13

#define IN1 7  // Right motor(K1/K2) direction Pin 7
#define IN2 8  // Right motor(K1/K2) direction Pin 8
#define IN3 9  // Left motor(K3/K4) direction Pin 9
#define IN4 10 // Left motor(K3/K4) direction Pin 10
#define ENA 5  // D5 to ENA PWM speed pin for Right motor(K1/K2)
#define ENB 6  // D6 to ENB PWM speed pin for Left motor(K3/K4)

#define Sensor_Result_Pin 2 // Ultrasonic Echo pin connect to D2
#define Sensor_Trigger_Pin 3 // Ultrasonic Trig pin connect to D3

const int FAST_SPEED = 150;
const int SPEED = 100;
const int TURN_SPEED = 150;
const int BACK_SPEED1 = 100;
const int BACK_SPEED2 = 90;

const int FORWARDDISTANCELIMIT = 30; // distance limit for obstacles in front
const int SWEEPDISTANCELIMIT = 30; // minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)
const int TURNWAITTIME = 500;

const int TIMETOWAITWHILETURNING = 200; // Time the robot spends turning (miliseconds)
const int SERVOMIDDLEPOINT = 105; //this should be 90 if the servo is mounted/manufactured correct;y. Use this to adjust the forward direction
const int SERVOSWEEPSIZE = 30; //the angle to sweep left/right when encountering a obstacle
const int SERVODWELLTIME = 300; //how long to wait for the servo to reach its destination
Servo head;

void Forward()
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
}

void Backward()
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
}

void Stop()
{
  analogWrite(ENB, 0);
  analogWrite(ENA, 0);
}

//turn right by having the left motor go forward and the right motor go backward
void Right()
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
}

//turn left by having the right motor go forward and the left motor go backward
void Left()
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
}

void SetSpeed(int speed)
{
  analogWrite(ENB, speed);
  analogWrite(ENA, speed);
}

//This method will return the distance measured by the ultrasonic sensor
int Distance()
{
  long duration;
  digitalWrite(Sensor_Trigger_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(Sensor_Trigger_Pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(Sensor_Trigger_Pin, LOW);
  duration = pulseIn(Sensor_Result_Pin, HIGH);
  const float speedOfSound = 340.29; // speed of sound in m/s
  const float speedOfSoundInCMperMicroSec = speedOfSound / 10000;
  return duration * speedOfSoundInCMperMicroSec / 2;
}

//initial setup
void setup()
{
  //set pin modes
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  //stop the motors
  Stop();

  //init servo
  head.attach(SERVO_PIN);
  head.write(SERVOMIDDLEPOINT);

  //init serial
  Serial.begin(9600);

  //init ultrasonic sensor
  pinMode(Sensor_Trigger_Pin, OUTPUT);
  pinMode(Sensor_Result_Pin, INPUT);
  digitalWrite(Sensor_Trigger_Pin, LOW);

  //start moving forward
  SetSpeed(SPEED);
  Forward();
}

//main loop
void loop()
{
  //we will already be moving forward
  //check if there is an obstacle in front
  int distance = Distance();
  if (distance < FORWARDDISTANCELIMIT)
  {
    //stop the car
    Stop();

    //turn the head to the left
    head.write(SERVOMIDDLEPOINT + SERVOSWEEPSIZE);
    delay(SERVODWELLTIME);
    //check if there is an obstacle to the left
    int leftDistance = Distance();
    bool leftClear = Distance() < SWEEPDISTANCELIMIT;

    //turn the head to the right
    head.write(SERVOMIDDLEPOINT - SERVOSWEEPSIZE);
    delay(SERVODWELLTIME);
    //check if there is an obstacle to the right
    int rightDistance = Distance();
    bool rightClear = Distance() < SWEEPDISTANCELIMIT;

    //turn the head back to the front
    head.write(SERVOMIDDLEPOINT);
    delay(SERVODWELLTIME);

    //determine which direction to move. if either side distance is available, use that one. if not, try to turn all the way around
    SetSpeed(SPEED);
    if(rightClear)
    {
      Right();
      delay(TURNWAITTIME);
    }
    else if(leftClear)
    {
      Left();
      delay(TURNWAITTIME);
    }
    else
    {
      Right();
      delay(TURNWAITTIME);
      Right();
      delay(TURNWAITTIME);
    }

    //start moving forward again
    Forward();
  }
}
