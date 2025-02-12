
/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/    
 * www.osoyoo.com IR remote control smart car
 * program tutorial https://osoyoo.com/2019/09/19/osoyoo-model-3-robot-learning-kit-lesson-2-ir-remote-controlled/
 *  Copyright John Yu
 */
#include "IRremote.hpp" 
#define IR_RECEIVE_PIN  12 //IR receiver Signal pin connect to Arduino pin D12  

#define speedPinR 5    			//  D5  connect MODEL-X ENA (PWM of right wheels)
#define RightDirectPin1  7    //Right Motor direction pin D7 to MODEL-X IN1 
#define RightDirectPin2  8    //Right Motor direction pin D8 to MODEL-X IN2
#define speedPinL 6    			// D6 connect MODEL-X ENB (PWM of left wheels)
#define LeftDirectPin1  9    	//Left Motor direction pin 9 to MODEL-X IN3 
#define LeftDirectPin2  10   	//Left Motor direction pin 10 to MODEL-X IN4 

 #define IR_ADVANCE       24       //code from IR controller "▲" button
 #define IR_BACK          82       //code from IR controller "▼" button
 #define IR_RIGHT         90       //code from IR controller ">" button
 #define IR_LEFT          8       //code from IR controller "<" button
 #define IR_STOP          28       //code from IR controller "OK" button
 #define IR_turnsmallleft 13       //code from IR controller "#" button

enum DN
{ 
  GO_ADVANCE, //go forward
  GO_LEFT, //left turn
  GO_RIGHT,//right turn
  GO_BACK,//backward
  STOP_STOP, 
  DEF
}Drive_Num=DEF;

bool stopFlag = true;//set stop flag
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;
uint8_t motor_update_flag = 0;
/***************motor control***************/
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1,LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,100);
  analogWrite(speedPinR,100);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightDirectPin1,LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,100);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightDirectPin1,HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  analogWrite(speedPinL,100);
  analogWrite(speedPinR,0);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightDirectPin1,HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  analogWrite(speedPinL,100);
  analogWrite(speedPinR,100);
  delay(t);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1,LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
}

/**************detect IR code***************/
void do_IR_Tick()
{
  if(IrReceiver.decode())
  {
    uint16_t command = IrReceiver.decodedIRData.command;
    if(command==IR_ADVANCE)
    {
      Drive_Num=GO_ADVANCE;
    }
    else if(command==IR_RIGHT)
    {
       Drive_Num=GO_RIGHT;
    }
    else if(command==IR_LEFT)
    {
       Drive_Num=GO_LEFT;
    }
    else if(command==IR_BACK)
    {
        Drive_Num=GO_BACK;
    }
    else if(command==IR_STOP)
    {
        Drive_Num=STOP_STOP;
    }
    command = 0;
    IrReceiver.resume();
  }
}

/**************car control**************/
void do_Drive_Tick()
{
    switch (Drive_Num) 
    {
      case GO_ADVANCE:go_Advance();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_ADVANCE code is detected, then go advance
      case GO_LEFT: go_Left();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_LEFT code is detected, then turn left
      case GO_RIGHT:  go_Right();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_RIGHT code is detected, then turn right
      case GO_BACK: go_Back();JogFlag = true;JogTimeCnt = 1;JogTime=millis();break;//if GO_BACK code is detected, then backward
      case STOP_STOP: stop_Stop();JogTime = 0;break;//stop
      default:break;
    }
    Drive_Num=DEF;
   //keep current moving mode for  200 millis seconds
    if(millis()-JogTime>=200)
    {
      JogTime=millis();
      if(JogFlag == true) 
      {
        stopFlag = false;
        if(JogTimeCnt <= 0) 
        {
          JogFlag = false; stopFlag = true;
        }
        JogTimeCnt--;
      }
      if(stopFlag == true) 
      {
        JogTimeCnt=0;
        stop_Stop();
      }
    }
}

void setup()
{
  pinMode(RightDirectPin1, OUTPUT); 
  pinMode(RightDirectPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(LeftDirectPin1, OUTPUT);
  pinMode(LeftDirectPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);      
}


void loop()
{
  do_IR_Tick();
  do_Drive_Tick();
}
