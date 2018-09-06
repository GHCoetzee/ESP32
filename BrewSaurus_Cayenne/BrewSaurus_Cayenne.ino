/*
This example shows how to connect to Cayenne using an ESP32 and send/receive sample data.

The CayenneMQTT Library is required to run this sketch. If you have not already done so you can install it from the Arduino IDE Library Manager.

Steps:
1. If you have not already installed the ESP32 Board Package install it using the instructions here: https://github.com/espressif/arduino-esp32/blob/master/README.md#installation-instructions.
2. Select your ESP32 board from the Tools menu.
3. Set the Cayenne authentication info to match the authentication info from the Dashboard.
4. Set the network name and password.
5. Compile and upload the sketch.
6. A temporary widget will be automatically generated in the Cayenne Dashboard. To make the widget permanent click the plus sign on the widget.
*/

//#define CAYENNE_DEBUG
#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP32.h>
#include <WiFi.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include <PID_v1.h>
#include <SimpleTimer.h>

// WiFi network info.
char ssid[] = "";
char wifiPassword[] = "";

// Cayenne authentication info. This should be obtained from the Cayenne Dashboard.
char username[] = "";
char password[] = "";
char clientID[] = "";

#define Pump_Control 1
#define Temp_Control_Automatic 2
#define Temp_Control_Manual 3
#define Temp_Control_Off 12
#define Start_Timer 10
#define Slider_Input 11
#define Pump_PIN 22

// Temperature probe pin config
#define MAXDO   18
#define MAXCS   21
#define MAXCLK  19
#define SSRPin 23

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);

//PID variables
double Setpoint, Input, Output, c;
double Kp=5, Ki=30, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;
unsigned long windowTimer;
int TempControlAutomatic, TempControlManual, TempControlOff, InputValue, PumpState, StartTimer;
int mash_s, mash_m, mash_h, value;
SimpleTimer timer;

CAYENNE_IN(Pump_Control)
{
  PumpState = getValue.asInt();
  //CAYENNE_LOG("Channel %d, pin %d, value %d", VIRTUAL_CHANNEL_1, Pump_PIN, value);
  // Write the value received to the digital pin.
  digitalWrite(Pump_PIN, PumpState);
}

CAYENNE_IN(Slider_Input)
{
  InputValue = getValue.asDouble();
  Serial.print("Slider adjust ");
  Serial.print(InputValue);
}

CAYENNE_IN(Temp_Control_Automatic)
{
  TempControlAutomatic = getValue.asInt();

  if (TempControlAutomatic == 1)
  {
    Cayenne.virtualWrite(Temp_Control_Manual, 0);
    TempControlManual=0;

    Cayenne.virtualWrite(Temp_Control_Off, 0);
    TempControlOff=0;
  }
}

CAYENNE_IN(Temp_Control_Manual)
{
  TempControlManual = getValue.asInt();

  if (TempControlManual == 1)
  {
    Cayenne.virtualWrite(Temp_Control_Automatic, 0);
    TempControlAutomatic=0;

    Cayenne.virtualWrite(Temp_Control_Off, 0);
    TempControlOff=0;
  }
}

CAYENNE_IN(Temp_Control_Off)
{
  TempControlOff = getValue.asInt();

  if (TempControlOff == 1)
  {
    Cayenne.virtualWrite(Temp_Control_Automatic, 0);
    TempControlAutomatic=0;

    Cayenne.virtualWrite(Temp_Control_Manual, 0);
    TempControlManual=0;
  }
}

CAYENNE_IN(Start_Timer)
{
  StartTimer = getValue.asInt();

  if (StartTimer == 1)
  {
    windowTimer=millis();
  }
  
  if (StartTimer == 0)
  {
    mash_h =0;
    mash_s = 0;
    mash_m = 0;
  }
  
}

void setup() {
	Serial.begin(115200);
    delay(10);

    // We start by connecting to a WiFi network

    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, wifiPassword);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    pinMode(Pump_PIN, OUTPUT);
    digitalWrite(Pump_PIN, LOW);
    pinMode(SSRPin, OUTPUT);
    digitalWrite(SSRPin, LOW);
	  Cayenne.begin(username, password, clientID);

   //PID Setup
    myPID.SetOutputLimits(0, WindowSize);
    myPID.SetSampleTime(200);
    mash_s=0;
    Setpoint=0;

    delay(1000);
    
    Cayenne.virtualWrite(Pump_Control, 0);  //pump
    Cayenne.virtualWrite(Temp_Control_Automatic, 0);  //Auto T control
    Cayenne.virtualWrite(Temp_Control_Manual, 0);  //Manual T control
    Cayenne.virtualWrite(Temp_Control_Off, 0); //T control off
     Cayenne.virtualWrite(Start_Timer, 0); //Timer off
    Cayenne.virtualWrite(Slider_Input, 0); //Set slider input as 0
}

void loop() {
  Cayenne.loop();

  c = thermocouple.readCelsius();


  if (TempControlAutomatic == 1)
  {
    //Output=WindowSize;
    Input=c;
    Setpoint=InputValue;
    myPID.SetMode(AUTOMATIC);
    Serial.println("Automatic control");
    Serial.println(Output);
  }
  
  if (TempControlManual == 1) 
  {
    Output=InputValue*50;
    Setpoint=0;
    myPID.SetMode(MANUAL);
     Serial.println("Manual control");
     Serial.println(Output);
    
  }
  
  if (TempControlOff == 1)
  {
    Output=0;
    Setpoint=0;
    Serial.println("Control off");
     Serial.println(Output);
    myPID.SetMode(MANUAL);
  }

  if (StartTimer == 1) 
   {
    mash_s = ((millis() - windowTimer)/1000);
    mash_m = mash_s / 60L;
    mash_h = mash_s / 3600L;
    mash_s = mash_s - mash_m * 60L;
    mash_m = mash_m - mash_h * 60L;
   }

     
  myPID.Compute();
 //  Serial.println(Output);
 //  Serial.println(myPID.GetMode()); 

  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
  if (millis() - windowStartTime > WindowSize)
  { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  if (Output > millis() - windowStartTime) digitalWrite(SSRPin, HIGH);
  else digitalWrite(SSRPin, LOW);
  //Read thermocouple and print on serial monitor
}

CAYENNE_OUT_DEFAULT()
{
  // Write data to Cayenne here. This example just sends the current uptime in milliseconds on virtual channel 0.
  // Some examples of other functions you can use to send data.
  Cayenne.virtualWrite(4, c);
  Cayenne.virtualWrite(5, Setpoint);
  Cayenne.virtualWrite(6, Output/WindowSize*100);
  Cayenne.virtualWrite(7, mash_h);
  Cayenne.virtualWrite(8, mash_m);
  Cayenne.virtualWrite(9, mash_s);  
}
