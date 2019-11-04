//First, uncomment the timer references.

//#include <Event.h>
//#include <Timer.h>

//#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
//#include <MeEncoderMotor.h>
#include <MeMegaPiPro.h>
#include <timer.h>
//#include <MeMegaPi.h>
/** Function List:
 *
 *    1. void MeEncoderMotor::begin();
 *    2. boolean MeEncoderMotor::moveTo(float angle, float speed);
 *
*/
//MeGasSensor GasSensor1(PORT_8);
//MeUltrasonicSensor usSensor(PORT_8);
//MeHumiture humiture(PORT_6);

//MeHumiture humiture(PORT_6);

//MeUltrasonicSensor ultraSensor_6(PORT_6); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

MeUltrasonicSensor ultraSensor_7(PORT_7); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */


boolean isSelfGuidedAnt=0;
int distance=0;
int randnum = 0;
String messageFromSerial;

int messageCounter =0;
//const int _timeToRunConst = 3000;
const int pingPin = 28; //ultrasonic sensor, connected native because the RJ45 is not working
//need to connect to pin 24, 5v and ground

//SoftwareSerial ESPserial(5, 6); // RX | TX

boolean isStart = false;
boolean isAvailable = false;
boolean leftflag = false;
boolean rightflag = false;

double durationOfTurningRight= 1000;  //For ant number 1 with 2% in enc 1 = 1000
double durationOfTurningLeft= 1100; //For ant number 1 with 2% in enc 1 = 1110
double durationOfMovement = 4000;
unsigned long time_now = 0;
bool isMovingFlag= false;
bool isTurningRightFlag= false;
bool isTurningLeftFlag= false;
char movementType = 'N'; //N == None, F == Forward, B == Backward, L == Left, R == Right
double correctionFactor = 1.02;
int turnSpeed = 200;

//int stepWorth = 780;
int stepWorth = 360;

int speedToMoveAt =100;

String antIdFromServer="";

Timer timer;
Timer timerForUS;
long distanceInCm;

long randNumber; //for demo purposes only.

//*******Defenition of motor related
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);


//#include <PID_v1.h>

double SetpointLeft ; // will be the desired value
double SetpointRight;
double InputLeft;
double InputRight;
double OutputLeft ; 
double OutputRight ; 
//PID parameters

double Kp=4, Ki=0, Kd=1; 

 // Encoder_1.setSpeedPid(0.18,0,0);
 
//create PID instance 
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//PID Encoder_1_PID(&InputLeft, &OutputLeft, &SetpointLeft, Kp, Ki, Kd,  DIRECT);
//PID Encoder_2_PID(&InputRight, &OutputRight, &SetpointRight, Kp, Ki, Kd,  DIRECT);
 


void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

//*********

String msgIN = ""; //Will contain message from Serial port.

void printEncoderCurrentPosition()
{
 float current1Pos = Encoder_1.getCurPos();
 float current2Pos = Encoder_2.getCurPos();
 Serial.print("Current position before moving, first: ");
 Serial.print(current1Pos);
 Serial.print("second: ");
 Serial.print(current2Pos);
 Serial.println();
}


void printTimeStampForMovement()
{
      Serial.print(" ,time_now:");
      Serial.print(time_now/1000);
      Serial.print(" ,millis():");
      Serial.print(millis()/1000);
      Serial.print(" ,time_now + durationOfMovement:");
      Serial.println((time_now + durationOfMovement)/1000);
}

//
//void printTimeStampForTurning()
//{
//      Serial.print(" ,time_now:");
//      Serial.print(time_now/1000);
//      Serial.print(" ,millis():");
//      Serial.print(millis()/1000);
//      Serial.print(" ,time_now + durationOfTurning:");
//      Serial.println((time_now + durationOfTurning)/1000);
//}

/************************** 4. Start Of Ant movement functions*********************************/
void Forward(int runTime)
{
    time_now = millis();
    SendMessageOverSerialconcatID("Started forward_SF");        
    printTimeStampForMovement();
    isMovingFlag = true;
    movementType = 'F';
    Encoder_1.runSpeed(-speedToMoveAt * correctionFactor);
    Encoder_2.runSpeed(speedToMoveAt);
    //printTimeStampForMovement();
   // SendMessageOverSerialconcatID("Finished forward_FF");  
}

void Backward(int runTime)
{  
   time_now = millis();
   SendMessageOverSerialconcatID("Started backwards_SB"); 
   printTimeStampForMovement(); 
   isMovingFlag = true; 
   movementType = 'B';
   Encoder_1.runSpeed(speedToMoveAt * correctionFactor);
   Encoder_2.runSpeed(-speedToMoveAt);
   //printTimeStampForMovement();
   //SendMessageOverSerialconcatID("Finished backwards_FB");
}
 
void TurnLeft(int runTime)
{  
  Serial.print("Turning Left for duration of:");
  Serial.println(durationOfTurningLeft);
  time_now = millis();
  SendMessageOverSerialconcatID("Started turning left_STL");
  isTurningLeftFlag= true;
  movementType = 'L';
  //printTimeStampForTurning();
  //Encoder_1.runSpeed(speedToMoveAt * correctionFactor);
  Encoder_1.runSpeed(speedToMoveAt);
  Encoder_2.runSpeed(speedToMoveAt);
  //Encoder_2.runSpeed(speedToMoveAt * correctionFactor);
  //printTimeStampForTurning();
  //SendMessageOverSerialconcatID("Finished turning left_FTL");
}

void TurnRight(int runTime)
{
  Serial.print("Turning Right for duration of:");
  Serial.println(durationOfTurningRight);
  time_now = millis();
  SendMessageOverSerialconcatID("Started turning right_STR");
  isTurningRightFlag= true;
  movementType = 'R';
  //printTimeStampForTurning();
 //Encoder_1.runSpeed(-speedToMoveAt * correctionFactor);
  Encoder_1.runSpeed(-speedToMoveAt);
  //Encoder_2.runSpeed(-speedToMoveAt * correctionFactor);
  Encoder_2.runSpeed(-speedToMoveAt);
  //printTimeStampForTurning();
  //SendMessageOverSerialconcatID("Finished turning right_FTR");
}

void Stop()
{
  Serial.println("Stopping Ant");
  
  Encoder_1.runSpeed(0);
  Encoder_2.runSpeed(0);    
  switch (movementType)
  {
    case 'F':
    SendMessageOverSerialconcatID("Finished forward_FF");  
    break;
    case 'B':
    SendMessageOverSerialconcatID("Finished backwards_FB");
    break;
    case 'R':
    SendMessageOverSerialconcatID("Finished turning right_FTR");
    break;
    case 'L':
    SendMessageOverSerialconcatID("Finished turning left_FTL");
    break;
    default:
    break;
    
  
  }
  
  SendMessageOverSerialconcatID("Stopped ant_SA");
   Serial.print(" Stopping ant at time_now:");
   Serial.print(time_now/1000);
   Serial.print(" ,millis():");
   Serial.println(millis()/1000);

}

void ChangeSpeed(int spd)
{
  Serial.print("Changing Speed to ");
  Serial.print(spd);
  Serial.println("");
  speedToMoveAt = spd;
}

void ChangeDurationOfTurning(double duration, char directionOfTurn)
{
  if (directionOfTurn == 'L')
  {
    Serial.print("Changing durationOfTurningLeft to ");  
    Serial.print(duration);
    Serial.println("");
    durationOfTurningLeft = duration;
  }
  else 
    if(directionOfTurn == 'R')
    {
      Serial.print("Changing durationOfTurningRight to ");
      Serial.print(duration);
      Serial.println("");
      durationOfTurningRight = duration;
    }
}


void ChangeDurationOfMoving(double duration)
{
  Serial.print("Changing durationOfMoving to ");
  Serial.print(duration);
  Serial.println("");
  durationOfMovement = duration;
}

void ChangeStepWorth(int newStepWorth)
{
  Serial.print("Changing Step Worth to ");
  Serial.print(newStepWorth);
  Serial.println("");
  stepWorth = newStepWorth;
}
 
void processAntCommand(String commandToExecute , int numOfSteps )
{      
  Serial.print("first step in processAntCommand, the command after trim is: ");
  commandToExecute.trim();
  Serial.println(commandToExecute);  
  int runTime = numOfSteps * durationOfMovement;//3000;
  Serial.println("Inside processAntCommand, got command:");
  Serial.println(commandToExecute);

  if (commandToExecute =="GF")
    {
    Serial.println("Got Go_Forward Command, Calling Forward function");
    Forward(runTime);
    }
     else if (commandToExecute =="GB")
        {
        Serial.println("Got Go_Backwards Command, Calling Backward function");
        Backward(runTime);
        }
        
     else if (commandToExecute =="TL")
        {
        Serial.println("Got Turn left Command, Calling Backward commandToExecuteWithWhiteSpfunction");
        TurnLeft(runTime);
        }
        
     else if (commandToExecute =="TR")
        {
        Serial.println("Got Turn right Command, Calling Backward function");
        TurnRight(runTime);
        }
        
         else if (commandToExecute =="BTL")
        {
        Serial.println("Got Backward Turn left Command, Calling Backward & Left function");
        TurnLeft(runTime);
        }
        
     else if (commandToExecute =="BTR")
        {
        Serial.println("Got Backwards Turn right Command, Calling Backward & Right function");
        TurnRight(runTime);
        }
        
     else if (commandToExecute.startsWith("CS"))
        {
        Serial.println("Got change speed command");
        String newSpd = commandToExecute.substring(2);
        int spd = newSpd.toInt();
        ChangeSpeed(spd);
        }
    else if (commandToExecute =="S")
        {
        Serial.println("Got Stop Command, Calling Stop function");
        Stop();
        }
    else 
    {
    Serial.println("Could not find the right command");
    }
 }

   /************************** 3. End of Of Handling Ant *********************************/


void sendDataOverSerial()
{

//  humiture.update();
  float humidity = (float) random(50,80); //humiture.getHumidity();
  float temp = (float) random(18,25); //humiture.getTemperature();
  String buf;
  buf += F("Humidity (%) = ");
  buf += String(humidity, 2);
  buf += F(", Temperature (oC) = ");
  buf += String(temp, 2);
 
  Serial.println(buf);
  buf = "WS_" + buf;
  Serial2.println(buf);
 
 // sendDistanceOverSerial();
}


void sendDistanceOverSerial()
{
//  String buf;
//  buf += String(distanceInCm);
//  buf += " cm to nearest front obstacle. ";  
//  Serial.println(buf);
// 
//  buf = "WS_" + buf;
//  Serial2.println(buf);

  String buf;
  buf += ultraSensor_7.distanceCm();
  buf += " cm to nearest front obstacle. ";  
  Serial.println(buf);

  buf = "WS_" + buf;
 Serial2.println(buf);
  
}

void sendStartedToMoveOverSerial()
{
  String buf;
  buf += "Started Movement";  
  Serial.println(buf);
  buf = "WS_" + buf;
  Serial2.println(buf);
}

void sendFinishedToMoveOverSerial()
{
  String buf;
  buf += "Finished Movement";  
  Serial.println(buf);
  buf = "WS_" + buf;
  Serial2.println(buf);
}


void SendMessageOverSerialconcatID(char *message)
{  
  String buf;
  buf += message;  
  Serial.println(buf);
  buf = "WS_" + buf;
  buf =   buf + "_" + antIdFromServer ;
  Serial.println(buf);
  Serial2.println(buf);

 
}


void SendMessageOverSerial(char *message)
{  
  String buf;
  buf += message;  
  Serial.println(buf);
  buf = "WS_" + buf ;
  Serial.println(buf);
  Serial2.println(buf);
}

void SendAntIdMessageOverSerial()
{  
  String buf;  
  Serial.println(buf);
  buf = "WS_ID_IS" + buf;
  buf =   buf + antIdFromServer ;
  Serial.println(buf);
  Serial2.println(buf);
}
 int counter =0; 

 
void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
 
  randomSeed(analogRead(0));
  
  timer.setInterval(9000);
  timer.setCallback(sendDataOverSerial);
  timer.start();

 
  timerForUS.setInterval(3000);
  timerForUS.setCallback(sendDistanceOverSerial);
  timerForUS.start();
  /*******************************************************************************/
 TCCR1A = _BV(WGM10);//timer1 will be set to 490hz in setup function
 TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);//970hz

  /*******************************************************************************/

//--------------------- Motor Related ----------------------------//

 attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
 attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
//  Serial.begin(115200);
//
////set pwm 1khz
  TCCR1A = _BV(WGM10);//PIN12
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
 
  TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
  TCCR2B = _BV(CS22);

  TCCR3A = _BV(WGM30);//PIN9
  TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

  TCCR4A = _BV(WGM40);//PIN5
  TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);

  Encoder_1.setPulse(7);
  Encoder_2.setPulse(7);
  Encoder_1.setRatio(26.9);
  Encoder_2.setRatio(26.9);
 
  Encoder_1.setPosPid(Kp,Ki,Kd);
  Encoder_2.setPosPid(Kp,Ki,Kd);
  //Encoder_1.setSpeedPid(0.18,0,0);
  //Encoder_2.setSpeedPid(0.18,0,0);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
//
//  
//  //Set PWM 8KHz
//  TCCR1A = _BV(WGM10);
//  TCCR1B = _BV(CS11) | _BV(WGM12);
//
//  TCCR2A = _BV(WGM21) | _BV(WGM20);
//  TCCR2B = _BV(CS21);
//
//  Encoder_1.setPulse(7);
//  Encoder_2.setPulse(7);
//  Encoder_1.setRatio(26.9);
//  Encoder_2.setRatio(26.9);
//  Encoder_1.setPosPid(1.8,0,1.2);
//  Encoder_2.setPosPid(1.8,0,1.2);
//  Encoder_1.setSpeedPid(0.18,0,0);
//  Encoder_2.setSpeedPid(0.18,0,0);
  
//--------------------- Motor Related ----------------------------//

}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}


void loop()
{
   if(isMovingFlag==true && (millis() > time_now + durationOfMovement))
   {
     // time_now = millis();
     isMovingFlag = false;
     Serial.print("Stopping ant from moving ");
     Stop();
   }

   if(isTurningRightFlag==true && (millis() > time_now + durationOfTurningRight))
   {
    isTurningRightFlag = false;
     // time_now = millis();
     Serial.print("Stopping ant from Turning ");
     Stop();
   }
   
   if(isTurningLeftFlag==true && (millis() > time_now + durationOfTurningLeft))
   {
    isTurningLeftFlag = false;
     // time_now = millis();
     Serial.print("Stopping ant from Turning ");
     Stop();
   }
   
  if(Serial.available())
  {
   String msgInTest = Serial.readStringUntil('\n');
  msgInTest.trim();
    Serial.println(msgInTest);
    if (msgInTest=="0")
    {
      Serial.println("Option 0");
      Encoder_1.runSpeed(0);
      Encoder_2.runSpeed(0);        
   
    }
    else 
    if (msgInTest=="1")
    {
      Serial.println("Option 1");
      String ddd2="TL";
      processAntCommand(ddd2, 1);
    }
    else 
    if (msgInTest=="2")
    {
      Serial.println("Option 2");
      String ddd3="TR";
      processAntCommand(ddd3, 1);
    }
    else
    if (msgInTest=="3")
    {
      Serial.println("Option 3");
      String ddd="GF";
      processAntCommand(ddd, 1);
    }
    else
    if (msgInTest.startsWith("ctdr_"))
    {
       
      String newDuration = msgInTest.substring(5);   
      int duration = newDuration.toInt();
      Serial.print("Got change turn duration command with parameter: ");
      Serial.println(duration);
      ChangeDurationOfTurning(duration, 'R');
    }      
     else
    if (msgInTest=="ptdr")
    {
      Serial.println("Option ptdr");      
      Serial.print("Turn duration of right is: ");
      Serial.println(durationOfTurningRight);   
    }    
    else  
    if (msgInTest.startsWith("ctdl_"))
    {
     
      String newDuration = msgInTest.substring(5);   
      int duration = newDuration.toInt();
      Serial.print("Got change turn duration command with parameter: ");
      Serial.println(duration);
      ChangeDurationOfTurning(duration, 'L');
    }      
    else
    if (msgInTest=="ptdl")
    {
      Serial.println("Option ptdl");      
      Serial.print("Turn duration of left is: ");
      Serial.println(durationOfTurningLeft);   
    }      
      
  }
  
  Encoder_1.loop();
  Encoder_2.loop();
  
  timer.update();
  timerForUS.update();
   
     if (Serial2.available())
     {
       while(Serial2.available()== 0);
       msgIN = Serial2.readStringUntil('\n');
       Serial.println("Got this from serial2 - from the esp8266:");
       Serial.println(msgIN);
       if (messageCounter ==0 )
        {
          SendMessageOverSerial("Hello Server");
          messageCounter =1;
        }

        // delay(5000);
          if (msgIN.startsWith("AID_"))
       {
         String antIdToAssign = msgIN.substring(4);      
         Serial.println("Assigning id to the ant");      
        Serial.println(antIdToAssign);      
       antIdFromServer =antIdToAssign;
       // int antIntIdToAssign = antIdToAssign.toInt();//= (msgIN.substring(5,1)).toInt(); //need to change to dynamicaly check the number. not only one digit;      
        Serial.print("Got this as an ID:");
        Serial.print(antIdFromServer);
        Serial.println("");      
        SendAntIdMessageOverSerial();
        //antIdFromServer = antIntIdToAssign;
       // SendMessageOverSerial(antIntIdToAssign);
        //SendMessageOverSerial(antIdFromServer);
         
        }
        else
       if (msgIN.startsWith("AC_"))
       {
         String antCommand = msgIN.substring(5);      
         Serial.println("Sending the command to processing");      
   
        int numOfSteps = (msgIN.substring(3,4)).toInt(); //need to change to dynamicaly check the number. not only one digit;    
       // int positionToGoTo =360;
        Serial.print("numOfSteps");
        Serial.print(numOfSteps);
        Serial.println("");
//
//         
         processAntCommand(antCommand, numOfSteps * stepWorth);
//
//
//         
//       //  delay(5000);
        }
      else
       if (msgIN.startsWith("CS_"))
       {
         
         Serial.println("Got maintenance command change speed:");      
         Serial.println(msgIN);
        int newSpeed = (msgIN.substring(3)).toInt(); //need to change to dynamicaly check the number. not only one digit;    
       // int positionToGoTo =360;
        Serial.print("newSpeed");
        Serial.print(newSpeed);
        Serial.println("");
      
          ChangeSpeed(newSpeed);
        }
      else
       if (msgIN.startsWith("CST_"))
       {
             
         Serial.println("Got maintenance command change step worth:");      
         Serial.println(msgIN);
         int newStepWorth = (msgIN.substring(4)).toInt(); //need to change to dynamicaly check the number. not only one digit;    
       // int positionToGoTo =360;
        Serial.print("newStepWorth");
        Serial.print(newStepWorth);
        Serial.println("");
      
        }
//
//     
      }
//     }
// 
//    long duration;//,  distanceInCm;
//    pinMode(pingPin, OUTPUT);
//    digitalWrite(pingPin, LOW);
//    delayMicroseconds(2);
//    digitalWrite(pingPin, HIGH);
//    delayMicroseconds(5);
//
//    duration = pulseIn(pingPin, HIGH);
//    distanceInCm = microsecondsToCentimeters(duration);
////    Serial.print(distanceInCm);
////    Serial.print("cm");
////    Serial.println();  
////    delay(100);
//
//// this is the stop command. remarked because when no ultrasonic connected, the distance is 0cm and it stops.
////  if(distanceInCm < 15)
////    {    
////      Serial.println("Less then 15 cm ahead, stopping");
////      Stop();
////    }

    
}
