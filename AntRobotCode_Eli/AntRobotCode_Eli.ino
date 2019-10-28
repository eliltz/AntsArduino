#include <Event.h>
#include <Timer.h>

//#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
//#include <MeOrion.h>
#include <MeMegaPiPro.h>
#include <timer.h>

/** Function List:
 *
 *    1. void MeEncoderMotor::begin();
 *    2. boolean MeEncoderMotor::moveTo(float angle, float speed);
 *
*/
//MeGasSensor GasSensor1(PORT_8);
MeUltrasonicSensor usSensor(PORT_8);
MeHumiture humiture(PORT_6);
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

int turnSpeed = 200;

//int stepWorth = 780;
int stepWorth = 360;

int speedToMoveAt =150;

String antIdFromServer="";

Timer timer;
Timer timerForUS;
long distanceInCm;

long randNumber; //for demo purposes only.

//*******Defenition of motor related 
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

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

/************************** 4. Start Of Ant movement functions*********************************/
void Forward(int positionToGoTo)
{
  Serial.print("Moving Forward to position:");
  Serial.print(positionToGoTo);
  Serial.println();
  printEncoderCurrentPosition();
  Encoder_1.moveTo(positionToGoTo,speedToMoveAt);
  Encoder_2.moveTo(-positionToGoTo,speedToMoveAt);
  printEncoderCurrentPosition();    
  SendMessageOverSerialconcatID("Started forward_SF");  
    
  delay(300);
    
  SendMessageOverSerialconcatID("Finished forward_FF");  
}

void Backward(int positionToGoTo)
{  
   Serial.println("Moving Backward to position:");  
   Serial.print(positionToGoTo);
   Serial.println("");
    
   SendMessageOverSerialconcatID("Started backwards_SB");  
   printEncoderCurrentPosition();
   Encoder_1.moveTo(-positionToGoTo , speedToMoveAt);
   Encoder_2.moveTo(positionToGoTo , speedToMoveAt);
   printEncoderCurrentPosition();
   
   delay(300);
   
   SendMessageOverSerialconcatID("Finished backwards_FB");
}
  
void TurnLeft()
{
  Serial.println("Turning Left"); 
  SendMessageOverSerialconcatID("Started turning left_STL");
  printEncoderCurrentPosition();
  Encoder_1.moveTo(-360, speedToMoveAt);
  Encoder_2.moveTo(-360, speedToMoveAt);
  printEncoderCurrentPosition();
  delay(300);

  SendMessageOverSerialconcatID("Finished turning left_FTL");
}

void TurnRight()
{
  Serial.println("Turning Right");
  SendMessageOverSerialconcatID("Started turning right_STR");
  printEncoderCurrentPosition();
  Encoder_1.moveTo(360  , speedToMoveAt);
  Encoder_2.moveTo(360, speedToMoveAt);
  printEncoderCurrentPosition();
  delay(300);

  SendMessageOverSerialconcatID("Finished turning right_FTR");
}

void Stop()
{
  Serial.println("Stopping Ant");

  SendMessageOverSerialconcatID("Stopped ant_SA");
  printEncoderCurrentPosition();
}

void ChangeSpeed(int spd)
{
  Serial.print("Changing Speed to ");
  Serial.print(spd);
  Serial.println("");
  speedToMoveAt = spd;
}
/************************** 4. End of Ant movement functions*********************************/

void processAntCommand(String commandToExecuteWithWhiteSp , int positionToGoTo)
{      
  int str_len = commandToExecuteWithWhiteSp.length();// + 1; 
  Serial.print("first step in processAntCommand. Number of chars in the parameter:");
  Serial.print(str_len);
  Serial.println("");
  Serial.print("And the command is:");
  Serial.print(commandToExecuteWithWhiteSp);
  Serial.println("");
  char char_array[str_len];
  commandToExecuteWithWhiteSp.toCharArray(char_array, str_len);
  String commandToExecute( char_array);
  Serial.println("second step in processAntCommand. Number of chars after passing to char array:");
  Serial.println(commandToExecute.length()); 

//if (str_len == 3)
////commandToExecute = commandToExecuteWithWhiteSp; // to bypass the whitespace that suddently disappeared
//else 

 String CommandWith2chars = commandToExecuteWithWhiteSp.substring(0,2);
 Serial.print("CommandWith2chars after substring:");
 Serial.print(CommandWith2chars);
 Serial.println("");
 Serial.println("Inside processAntCommand, got command:");
 Serial.println(commandToExecute);
 commandToExecute =CommandWith2chars;
  Serial.println("");
  if (commandToExecute =="GF")
    {
      Serial.println("Got Go_Forward Command, Calling Forward function");
      Forward(positionToGoTo);
    }
     else if (commandToExecute =="GB")
       {
          Serial.println("Got Go_Backwards Command, Calling Backward function");
          Backward(positionToGoTo);
       }       
     else if (commandToExecute =="TL")
        {
          Serial.println("Got Turn left Command, Calling Backward commandToExecuteWithWhiteSpfunction");
          TurnLeft();
        }        
     else if (commandToExecute =="TR")
        {
          Serial.println("Got Turn right Command, Calling Backward function");
          TurnRight();
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
  String buf;
  buf += String(distanceInCm);
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
  //timerForUS.start();
  /*******************************************************************************/
 TCCR1A = _BV(WGM10);//timer1 will be set to 490hz in setup function
 TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);//970hz

  /*******************************************************************************/

//--------------------- Motor Related ----------------------------//

 attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
 attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
//  Serial.begin(115200);

//set pwm 1khz
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
  
  Encoder_1.setPosPid(1.8,0,0.5);
  Encoder_2.setPosPid(1.8,0,0.5);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
//--------------------- Motor Related ----------------------------//

}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}


void loop() 
{

//  For testing purposes - read from user serial
   if (Serial.available())
     {
       while(Serial.available()== 0);
       msgIN = Serial.readStringUntil('\n');
       Serial.println("Got this from serial as user input:");
       Serial.println(msgIN);
       if (msgIN.startsWith("c"))
       {
        //distancetogo
        //move
        //moveto
         switch(msgIN)
          {
            case '0':
              printEncoderCurrentPosition();
              Encoder_1.moveTo(0,speedToMoveAt);
              Encoder_2.moveTo(0,speedToMoveAt);
              printEncoderCurrentPosition();
            break;
            case '1':
              printEncoderCurrentPosition();
              Encoder_1.move(stepWorth,speedToMoveAt);
              Encoder_2.move(-stepWorth,speedToMoveAt);
              printEncoderCurrentPosition();
            break;
            case '2':
              printEncoderCurrentPosition();
              Encoder_1.distanceToGo(stepWorth,speedToMoveAt);
              Encoder_2.distanceToGo(-stepWorth,speedToMoveAt);
              printEncoderCurrentPosition();
            break;
            case 'cc':
               int distanceToGoCommand = (msgIN.substring(2)).toInt(); //need to change to dynamicaly check the number. not only one digit;      
              //int positionToGoTo = 0;
              Serial.print("distanceToGoCommand:");
              Serial.print(distanceToGoCommand);
              Serial.print(", stepWorth:");
              Serial.print(stepWorth);             
              Serial.print(", speedToMoveAt:");
              Serial.print(speedToMoveAt);              
              Serial.println("");                
              printEncoderCurrentPosition();
              Encoder_1.distanceToGo(distanceToGoCommand,speedToMoveAt);
              Encoder_2.distanceToGo(-distanceToGoCommand,speedToMoveAt);
              printEncoderCurrentPosition();
            break;                    
             case 'cs':              
              int numberOfStepsToTake = (msgIN.substring(2)).toInt(); //need to change to dynamicaly check the number. not only one digit;      
              //int positionToGoTo = 0;
              Serial.print("Number of steps:");
              Serial.print(numberOfStepsToTake);
              Serial.print(", stepWorth:");
              Serial.print(stepWorth);             
              Serial.print(", speedToMoveAt:");
              Serial.print(speedToMoveAt);              
              Serial.println("");                
              printEncoderCurrentPosition();
              Encoder_1.distanceToGo(numberOfStepsToTake * stepWorth,speedToMoveAt);
              Encoder_2.distanceToGo(-(numberOfStepsToTake * stepWorth),speedToMoveAt);
              printEncoderCurrentPosition();
            break;   
            case '6':
              printEncoderCurrentPosition();
            break;          
            default:
            break;
          }
         String antCommand = msgIN.substring(1);      
         Serial.print("Sending this command to processing:");       
         Serial.print(antCommand);       
         Serial.println();
         int numOfSteps = (msgIN.substring(3,4)).toInt(); //need to change to dynamicaly check the number. not only one digit;      
         int positionToGoTo = 0;
         Serial.print("Number of steps:");
         Serial.print(numOfSteps);
         Serial.println("");   
         if (numOfSteps ==0)
          positionToGoTo = 0;     
         if (numOfSteps ==1)
          positionToGoTo = 360;
          if (numOfSteps ==2)
          positionToGoTo = 1800;
          if (numOfSteps ==3)
          positionToGoTo = 2000;
          if (numOfSteps ==4)
          positionToGoTo = 3600;      
            Serial.print("positionToGoTo:");
           Serial.print(positionToGoTo);
            Serial.println("");  
         //processAntCommand(antCommand, positionToGoTo);
          Encoder_1.moveTo(positionToGoTo,speedToMoveAt);
          Encoder_2.moveTo(-positionToGoTo,speedToMoveAt);
         distanceToGo
       }
        if (msgIN.startsWith("s"))
       {
         int speedOf = (msgIN.substring(1,4)).toInt(); //need to change to dynamicaly check the number. not only one digit;   
         Serial.print("Speed to change to: ");  
         Serial.print(speedOf);  
         Serial.println("");  
         changeSpeed(speedOf);
         Serial.print("speedToMoveAt changed to: ");  
         Serial.print(speedToMoveAt);  
         Serial.println("");  
       }
     }

     // delay(2000);
       Encoder_1.loop();
       Encoder_2.loop();
      // delay(2000);




      
//        if (msgIN.startsWith("AID_"))
//       {
//         String antIdToAssign = msgIN.substring(4);      
//         Serial.println("Assigning id to the ant");       
//         Serial.println(antIdToAssign);       
//       antIdFromServer =antIdToAssign;
//       // int antIntIdToAssign = antIdToAssign.toInt();//= (msgIN.substring(5,1)).toInt(); //need to change to dynamicaly check the number. not only one digit;      
//        Serial.print("Got this as an ID:");
//        Serial.print(antIdFromServer);
//        Serial.println("");       
//        SendAntIdMessageOverSerial();
//        }
//AC_1_GF
//      // Forward(120);
//      // Backward(12321);
//     }

 // Forward(2500);
  //TurnRight(1100);
 // Forward(2500);
  //Forward(2500);
  //TurnLeft(1100);
  
  
  timer.update();
  timerForUS.update();
    if (isSelfGuidedAnt == 1) //If we want the ant to act autonomously. Needs Development
    {
      //selfGuidedAntProcess();
    }
    else //If The ant is being given commands remotely
    {           
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
        int positionToGoTo =0; 
        Serial.print("positionToGoTo");
        Serial.print(positionToGoTo);
        Serial.println("");

         
         processAntCommand(antCommand, positionToGoTo);


         
       //  delay(5000);
        }

     
      }
     }

//   Serial.println("*********************");       
//   distance = usSensor.distanceCm();
//   Serial.print("Distance from UltraSonic: ");
//   Serial.print(distance);
//   Serial.println();
//   if(distance < 5)
//    {
//      Serial2.println("Less then 5cm ahead, stopping");
//      Serial.println("Less then 5cm ahead, stopping");
//      Stop();
//    }
    
    long duration;//,  distanceInCm;
    pinMode(pingPin, OUTPUT);
    digitalWrite(pingPin, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin, HIGH);
    delayMicroseconds(5);

    duration = pulseIn(pingPin, HIGH);
    distanceInCm = microsecondsToCentimeters(duration);
//    Serial.print(distanceInCm);
//    Serial.print("cm");
//    Serial.println();  
//    delay(100);

// this is the stop command. remarked because when no ultrasonic connected, the distance is 0cm and it stops.
//  if(distanceInCm < 15)
//    {    
//      Serial.println("Less then 15 cm ahead, stopping");
//      Stop();
//    } 

    
//  
//  /************************** 1. Start Of Gas Sensor*********************************/
//  Serial.print("Analog Value is: ");
//  Serial.print(GasSensor1.readAnalog());
//  Serial.print("----Status: ");
//  if(GasSensor1.readDigital() == Gas_Exceeded)
//  {
//    Serial.println("The concentration exceeds");
//  }
//  else if(GasSensor1.readDigital() == Gas_not_Exceeded)
//  {
//    Serial.println("The concentration of the gas in the range");
//  }
//  delay(200);
//   /************************** 1. End Of Gas Sensor*********************************/

 /************************** 4. Start Of Temp and Humidity Sensor*********************************/
//
//
////  humiture.update();
////  float humidity = humiture.getHumidity();
////  float temp =  humiture.getTemperature();
////  String buf;
////  buf += F("Humidity (%) = ");
////  buf += String(humidity, 6);
////  buf += F(", Temperature (oC) =:");
////  buf += String(temp, 6);
////  Serial2.println(buf);
//
//  
   /************************** 4.  End Of Temp and Humidity Sensor*********************************/

}
