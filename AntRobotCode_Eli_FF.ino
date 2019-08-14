//#include "MeOrion.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
//#include <MeOrion.h>
#include <MeMegaPiPro.h>
#include <timer.h>
#include <ArduinoJson.h>

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

float radius =29.05;
const int _timeToRunConst = 3000;
const int pingPin = 28; //ultrasonic sensor, connected native because the RJ45 is not working
//need to connect to pin 24, 5v and ground

//SoftwareSerial ESPserial(5, 6); // RX | TX

boolean isStart = false;
boolean isAvailable = false;
boolean leftflag = false;
boolean rightflag = false;
//v=  2π/60⋅r⋅RPM
uint8_t motorSpeed = 80;//60;
//int moveSpeed = 190;
int turnSpeed = 200;

Timer timer;
Timer timerForUS;
long distanceInCm;

long randNumber; //for demo purposes only.
int rows =3, cols = 4;
char mapMatrixArr [3] [4] = {
  //as many vals as dim1
 {'u','u','u','u'},
 {'u','u','u','u'},
 {'u','u','u','u'}
};

MeMegaPiDCMotor motor2(PORT1B);
MeMegaPiDCMotor motor4(PORT2B);




String msgIN = ""; //Will contain message from Serial port.


/*motors control */
void leftMotor(int speed){
  Serial.println("leftMotor:");
  Serial.println(speed);
  if (speed){
    motor2.run(motorSpeed); /* value: between -255 and 255. */
  }else{
    motor2.stop();
  }
}

void rightMotor(int speed){
  Serial.println("rightMotor:");
  Serial.println(speed);
  if (speed){
    motor4.run(-motorSpeed); /* value: between -255 and 255. */
  }else{
    motor4.stop();
  }
}
/* end of motors control */

/************************** 4. Start Of Ant movement functions*********************************/
void Forward(int runTime)
{
  Serial.println("Moving Forward..");
//  float currentPos = motor2.GetCurrentPosition();
// Serial.print("Current position before moving");
// Serial.print(currentPos );
// Serial.println();
  
  //sendStartedToMoveOverSerial();
   SendMessageOverSerial("Started forward_SF");
   motor2.run(motorSpeed);
   motor4.run(-motorSpeed);
   if (runTime != 0)
  {
    delay(runTime);
    Stop();
    SendMessageOverSerial("Finished forward_FF");
  }
  else
  {
    delay(30);
  }
 //float currentPos = motor2.GetCurrentPosition();
// Serial.print("Current position before moving");
// Serial.print(currentPos );
// Serial.println();
}

void Backward(int runTime)
{  
  Serial.println("Moving Backward");
  Serial.print("Run Time:");
  Serial.print(runTime);
  Serial.println("");
 // sendStartedToMoveOverSerial();
  SendMessageOverSerial("Started backwards_SB");
   motor2.run(-motorSpeed);
   motor4.run(motorSpeed);
  if (runTime != 0)
  {
  Serial.print("Delaying...");  
    delay(runTime);
    Serial.print("Stopping.");
    Stop();
    SendMessageOverSerial("Finished backwards_FB");
  }
  else
  {
    delay(30);
  }
}

void BackwardAndTurnLeft(int runTime)
{
  runTime =2300;
  Serial.println("Moving BackwardAndTurnLeft");
   motor2.run(-motorSpeed);
   motor4.run(motorSpeed/2);
   if (runTime != 0)
  {
    delay(runTime);
    Stop();
  }
  else
  {
    delay(30);
  }
}

void BackwardAndTurnRight(int runTime)
{
  runTime =2300;
  Serial.println("Moving BackwardAndTurnRight");
   motor2.run(motorSpeed/2);
   motor4.run(-motorSpeed);
   if (runTime != 0)
  {
    delay(runTime);
    Stop();
  }
  else
  {
    delay(30);
  }
}
  
void TurnLeft(int runTime)
{
runTime =950;
  Serial.println("Turning Left");
  //sendStartedToMoveOverSerial();
  SendMessageOverSerial("Started turning left_STL");
   motor2.run(-motorSpeed);
   motor4.run(-motorSpeed);
    if (runTime != 0)
  {
    delay(runTime);
    Stop();
    SendMessageOverSerial("Finished turning left_FTL");
  }
  else
  {
    delay(30);
    Stop();
  }
}

void TurnRight(int runTime)
{
  runTime =1100;
  Serial.println("Turning Right");
  //sendStartedToMoveOverSerial();
  SendMessageOverSerial("Started turning right_STR");
   motor2.run(motorSpeed);
   motor4.run(motorSpeed);
   if (runTime != 0)
  {
    delay(runTime);
    Stop();
    SendMessageOverSerial("Finished turning right_FTR");
  }
  else
  {
    delay(30);
  }

}

void Stop()
{
  Serial.println("Stopping Ant");
   motor2.stop();
   motor4.stop();
   //sendFinishedToMoveOverSerial();
   SendMessageOverSerial("Stopped ant_SA");
}

void ChangeSpeed(int spd)
{
  Serial.print("Changing Speed to ");
  Serial.print(spd);
  Serial.println("");
  motorSpeed = spd;
}
/************************** 4. End of Ant movement functions*********************************/


void processAntCommand(String commandToExecuteWithWhiteSp , int numOfSteps )
{      
  int str_len = commandToExecuteWithWhiteSp.length();// + 1; 
  Serial.println("first step in processAntCommand. Number of chars in the parameter:");
  Serial.println(str_len);
  char char_array[str_len];
  commandToExecuteWithWhiteSp.toCharArray(char_array, str_len);
  String commandToExecute( char_array);
 Serial.println("second step in processAntCommand. Number of chars after passing to char array:");
  Serial.println(commandToExecute.length());
  int runTime = numOfSteps * _timeToRunConst;//3000;
  //9.2515 //d= C/π
  int blockDim = 50; //cm

//if (str_len == 3)
////commandToExecute = commandToExecuteWithWhiteSp; // to bypass the whitespace that suddently disappeared
//else 


  double wheelDiameter = 58.1;
 
  
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


void SendMessageOverSerial(char *message) 
{  
  String buf;
  buf += message;  
  Serial.println(buf);
  buf = "WS_" + buf;
  Serial2.println(buf);
//    Serial.print(hour());
//    Serial.print(":");
//    Serial.print(minute());
//    Serial.print(":");
//    Serial.print(second());
//    Serial.print(" ");  
//    Serial.println(logmessage);
//    Serial2.println(buf);
  
}

void sendMatrixToSerialAsJson(char mapMatrix[][4])
{  
//  StaticJsonDocument<256> doc;
//  char a[]  = {1,2,3};
////  char mapMatrix2[][4] ={
////  {'u','u','u','u'},
////  {'u','u','u','u'},
////  {'u','u','u','u'}
////  };
//  //JsonArray array = jsonBuffer.createArray();
//  copyArray(mapMatrix[0][4], doc.to<JsonArray>());
//  serializeJson(doc, Serial);
// // Serial.println();
}

void setup() 
{


  Serial.begin(9600);
 // Serial2.begin(9600);
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


//u = uncharted
//b = charted and blocked
//c = charted and clear 
//e = entry point
//x - exit
//d - destination

char mapMatrixArr [rows] [cols] = {
  //as many vals as dim1
 {'u','u','e','u'},
 {'u','u','u','u'},
 {'u','u','u','u'}
};


}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}


void loop() 
{

  //For testing purposes - read from user serial
   if (Serial.available())
     {
       while(Serial.available()== 0);
       msgIN = Serial.readStringUntil('\n');
       Serial.println("Got this from serial1 User?");
       Serial.println(msgIN);
       Forward(12321);
       Backward(12321);
     }


  
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

        // delay(5000);
       if (msgIN.startsWith("AC_"))
       {
         String antCommand = msgIN.substring(5);      
         Serial.println("Sending the command to processing");       
    
        int numOfSteps = (msgIN.substring(3,4)).toInt(); //need to change to dynamicaly check the number. not only one digit;      
        Serial.print("Number of steps:");
        Serial.print(numOfSteps);
        Serial.println("");

         
         processAntCommand(antCommand, numOfSteps);


         
       //  delay(5000);
        }
      }
     }
//  Serial.println("Printing matrix");       
//  sendMatrixToSerialAsJson(&mapMatrixArr[0]);


//  Serial.println("*********************");       
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
//    digitalWrite(pingPin, LOW);
//    pinMode(pingPin, INPUT);
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



