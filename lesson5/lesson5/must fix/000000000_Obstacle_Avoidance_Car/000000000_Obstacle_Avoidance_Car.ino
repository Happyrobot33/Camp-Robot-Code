//www.elegoo.com
//2016.09.23
/*
Use "myservo.write (angle)" to command the micron servo 
to the angle degree which has a range from 10 to 180. 
If it exceeds the range, the micro servo won’t recognize 
this angle and will keep rotating.
*/
#include <Servo.h>
Servo myservo; // create servo object to control servo
#define SERVO_PIN  11
#define LPT 2 // scan loop coumter
#define BUZZ_PIN     13

#define IN1  7    //Right motor(K1/K2) direction Pin 7
#define IN2  8  //Right motor(K1/K2) direction Pin 8
#define IN3  9  //Left motor(K3/K4) direction Pin 9
#define IN4  10  //Left motor(K3/K4) direction Pin 10
#define ENA  5   //D5 to ENA PWM speed pin for Right motor(K1/K2)
#define ENB  6   //D6 to ENB PWM speed pin for Left motor(K3/K4)
int ABS = 100;
int ABSS = 200;
int Echo = A4;  
int Trig = A5;
#define Echo_PIN    2 // Ultrasonic Echo pin connect to D2
#define Trig_PIN    3  // Ultrasonic Trig pin connect to D3


int rightDistance = 0,leftDistance = 0,middleDistance = 0 ;
void _mForward()
{
 analogWrite(ENA,ABS);
 analogWrite(ENB,ABS);
  //digitalWrite(in1,HIGH);//digital output
  //digitalWrite(in2,LOW);
  //igitalWrite(in3,LOW);
  //digitalWrite(in4,HIGH);
  digitalWrite(IN4,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN2,HIGH );
  digitalWrite(IN1,LOW);
 Serial.println("go forward!");
}

void _mBack()
{
 analogWrite(ENA,ABS);
 analogWrite(ENB,ABS);
  //digitalWrite(in1,LOW);
  //digitalWrite(in2,HIGH);
  //digitalWrite(in3,HIGH);
  //digitalWrite(in4,LOW);
  digitalWrite(IN4,LOW);
  digitalWrite(IN3,HIGH); 
  digitalWrite(IN2,LOW);
  digitalWrite(IN1,HIGH);
 Serial.println("go back!");
}

void _mleft()
{
 analogWrite(ENA,ABSS);
 analogWrite(ENB,ABSS);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW); 
 Serial.println("go left!");
}

void _mright()
{
 analogWrite(ENA,ABSS);
 analogWrite(ENB,ABSS);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
 Serial.println("go right!");
} 
void _mStop()
{
  //digitalWrite(ENA,LOW);
  //digitalWrite(ENB,LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW); 
  Serial.println("Stop!");
} 
 /*Ultrasonic distance measurement Sub function*/
int Distance_test()   
{
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(5);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance/70;       
  return (int)Fdistance;
}  

void setup() 
{ 
  myservo.attach(3);// attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  _mStop();
} 

void loop() 
{ 
    myservo.write(90);//setservo position according to scaled value
    delay(500); 
    middleDistance = Distance_test();
    #ifdef send
    Serial.print("middleDistance=");
    Serial.println(middleDistance);
    #endif

    if(middleDistance<=20)
    {     
      _mStop();
      delay(500); 	  
      myservo.write(10);//10°-180°          
      delay(1000);      
      rightDistance = Distance_test();

      #ifdef send
      Serial.print("rightDistance=");
      Serial.println(rightDistance);
      #endif

      delay(500);
       myservo.write(90);              
      delay(1000);                                                  
      myservo.write(180);              
      delay(1000); 
      leftDistance = Distance_test();

      #ifdef send
      Serial.print("leftDistance=");
      Serial.println(leftDistance);
      #endif

      delay(500);
      myservo.write(90);              
      delay(1000);
      if(rightDistance>leftDistance)  
      {
        _mright();
        delay(360);
       }
       else if(rightDistance<leftDistance)
       {
        _mleft();
        delay(360);
       }
       else if((rightDistance<=20)||(leftDistance<=20))
       {
        _mBack();
        delay(180);
       }
       else
       {
        _mForward();
       }
    }  
    else
        _mForward();                     
}
