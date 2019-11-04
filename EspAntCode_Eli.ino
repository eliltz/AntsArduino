/*******************Esp8266_Websocket.ino****************************************/
#include <ESP8266WiFi.h>
#include "WebSocketClient.h"
#include <SoftwareSerial.h>
//SoftwareSerial ArduinoSerial(D6,D5); // (Rx, Tx)

volatile unsigned long previousMillis2;
boolean handshakeFailed=0;
String data;

char path[] = "/";   //identifier of this device

//const char* ssid     = "Cisco07009";
const char* ssid     = "Cisco07009";
//const char* password = "";
const char* password = "";
//char* host = "192.168.1.141";  //replace this ip address with the ip address of your raspberry pi
const int espport= 3000;
  
WebSocketClient webSocketClient;
unsigned long previousMillis = 0;
unsigned long currentMillis;
unsigned long interval=10000; //interval for sending data to the websocket server in ms

String msgIN = ""; //Will contain message from Serial port.

// Use WiFiClient class to create TCP connections
WiFiClient client;


//*********************************************************************************************************************
//***************function definitions**********************************************************************************
void wsconnect(){
  // Connect to the websocket server
  if (client.connect(host, espport)) {
    Serial.println("Connected");
  } else {
    Serial.println("Connection failed.");
      delay(1000);  
   
   if(handshakeFailed){
    handshakeFailed=0;
    ESP.restart();
    }
    handshakeFailed=1;
  }

  // Handshake with the server
  webSocketClient.path = path;
  webSocketClient.host = host;
  if (webSocketClient.handshake(client)) {
    Serial.println("Handshake successful");
  } else {
    
    Serial.println("Handshake failed.");
   delay(4000);  
   
   if(handshakeFailed){
    handshakeFailed=0;
    ESP.restart();
    }
    handshakeFailed=1;
  }
}

void setup() {
  Serial.begin(9600);
//    pinMode(readPin, INPUT);     // Initialize the LED_BUILTIN pin as an output
 // ArduinoSerial.begin(9600);
  delay(10);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  //WiFi.begin(ssid, password);
  WiFi.begin(ssid, NULL);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(1000);//3000
  
wsconnect();
//  wifi_set_sleep_type(LIGHT_SLEEP_T);
}

void loop() {

 if (client.connected()) {
    currentMillis=millis(); 
    webSocketClient.getData(data); 
    // webSocketClient.sendData("Eli Eli Eli from ESP"); //DEBUG
     // Serial.println("outside data not empty");  //DEBUG
    if (data != "")
    {           
      
//          Serial.println("Inside data not empty"); //DEBUG
       //    Serial.println(data); //DEBUG
      // Length (with one extra character for the null terminator)
      int str_len = data.length() + 1; 
      
      // Prepare the character array (the buffer) 
      char char_array[str_len];
      
      // Copy it over 
      data.toCharArray(char_array, str_len);
      //  Serial.println(data);
      if (str_len >1)
      
      {
        Serial.println(char_array);  //Sending the data to the arduino via serial port
        delay(30);
      }
      data ="";         
    }
    else 
    {
      

//u = uncharted
//b = charted and blocked
//c = charted and clear 
//e = entry point
//x - exit
//d - destination
//int rows =3, cols = 4;
//char mapMatrixArr [rows] [cols] = {
//  //as many vals as dim1
// {'u','c','e','u'},
// {'c','c','u','u'},
// {'b','c','u','u'}
// {'u','c','u','u'}
// {'u','x','u','u'}
// 
//};



      //Serial.println("data is empty or no data was received from arduinoAnt");
     }


 // Getting data from the arduino board
    while(Serial.available()== 0);
    msgIN = Serial.readStringUntil('\n');
   // Serial.println(msgIN);  
    String subMsgToWebSocket;
    if (msgIN.startsWith("WS_"))
    {
      subMsgToWebSocket = msgIN.substring(3);
    //  Serial.println(subMsgToWebSocket);  
      webSocketClient.sendData(subMsgToWebSocket);
    }
   }//end of if client connected
else
{
  Serial.println("not connected...");
}
    delay(5);


    

  }
             
//        webSocketClient.sendData(msgIN);
        //Serial.println("Got a message command");
    

    //*************send log data to server in certain interval************************************
 //currentMillis=millis();   
// if (abs(currentMillis - previousMillis) >= interval) {
//previousMillis = currentMillis;
//data=analogRead(A0); //read adc values, this will give random value, since no sensor is connected. 
////For this project we are pretending that these random values are sensor values
//
//webSocketClient.sendData(data);//send sensor data to websocket server


//  }
//  else{
//}
