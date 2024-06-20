#include <Servo.h>

#define LPT 2 // scan loop counter
#define BUZZ_PIN 13

#define IN1 7    //Right motor(K1/K2) direction Pin 7
#define IN2 8    //Right motor(K1/K2) direction Pin 8
#define IN3 9    //Left motor(K3/K4) direction Pin 9
#define IN4 10   //Left motor(K3/K4) direction Pin 10
#define ENA 5    //D5 to ENA PWM speed pin for Right motor(K1/K2)
#define ENB 6    //D6 to ENB PWM speed pin for Left motor(K3/K4)

#define Echo_PIN 2 // Ultrasonic Echo pin connect to D2
#define Trig_PIN 3 // Ultrasonic Trig pin connect to D3

#define FAST_SPEED 150
#define SPEED 100
#define TURN_SPEED 150
#define BACK_SPEED1 100
#define BACK_SPEED2 90

int distance;
int numcycles = 0;
int thereis;
Servo head;

void go_Advance() // motor rotate clockwise -->robot go ahead
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
}

void go_Back() // motor rotate counterclockwise -->robot go back
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
}

void stop_Stop() // motor brake -->robot stop
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void go_Right() // left motor rotate clockwise and right motor rotate counterclockwise -->robot turn right
{
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN1, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void go_Left() // left motor rotate counterclockwise and right motor rotate clockwise -->robot turn left
{
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN1, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void buzz_ON() // open buzzer
{
  for (int i = 0; i < 100; i++)
  {
    digitalWrite(BUZZ_PIN, LOW);
    delay(2); //wait for 1ms
    digitalWrite(BUZZ_PIN, HIGH);
    delay(2); //wait for 1ms
  }
}

void buzz_OFF() // close buzzer
{
  digitalWrite(BUZZ_PIN, HIGH);
}

void alarm()
{
  buzz_ON();
  buzz_OFF();
}

int watch()
{
  long echo_distance;
  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig_PIN, LOW);
  echo_distance = pulseIn(Echo_PIN, HIGH);
  echo_distance = echo_distance * 0.01657; //how far away is the object in cm
  Serial.println((int)echo_distance);
  return round(echo_distance);
}

void rotate_to_farthest()
{
  int maxDistance = 0;
  int maxAngle = 0;
  
  for (int angle = 0; angle <= 180; angle += 10)
  {
    head.write(angle);
    delay(100);
    int dist = watch();
    if (dist > maxDistance)
    {
      maxDistance = dist;
      maxAngle = angle;
    }
  }
  
  head.write(maxAngle);
}

void auto_avoidance()
{
  ++numcycles;

  if (numcycles >= LPT)
  {
    stop_Stop();
    rotate_to_farthest();

    distance = watch();
    if (distance < 30)
    {
      stop_Stop();
      alarm();
      delay(1000); // Delay for alarm sound
    }
    else
    {
      analogWrite(ENA, SPEED);
      analogWrite(ENB, SPEED);
      go_Advance();

      // Add a delay for forward motion
      delay(500); // Adjust this delay as needed

      // Check if there's an obstacle ahead after moving forward
      distance = watch();
      if (distance < 30)
      {
        stop_Stop();
        alarm();
        delay(1000); // Delay for alarm sound
      }
    }

    numcycles = 0;
  }
  else
  {
    // Continue moving forward without frequent stops
    analogWrite(ENA, SPEED);
    analogWrite(ENB, SPEED);
    go_Advance();
    delay(400);
    stop_Stop();
  }

  distance = watch();

  if (distance < 30)
  {
    go_Back();
    delay(400);
    ++thereis;
  }

  if (distance > 30)
  {
    thereis = 0;
  }

  if (thereis > 25)
  {
    stop_Stop();
    thereis = 0;
  }
}

void setup()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stop_Stop(); //stop move
  /*init HC-SR04*/
  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  /*init buzzer*/
  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, HIGH);
  buzz_OFF();
  digitalWrite(Trig_PIN, LOW);
  /*init servo*/
  head.attach(11);
  head.write(90);
  delay(2000);
  Serial.begin(9600);
}

void loop()
{
  auto_avoidance();
}
