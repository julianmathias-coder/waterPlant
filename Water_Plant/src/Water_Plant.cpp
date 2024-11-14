/* 
 * Project: waterPlant
 * Author: Julian  Mathias
 * Date: 13-NOV-24
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "IoTClassroom_CNM.h"
#include "Colors.h"
#include <DFRobot_PN532.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Grove_4Digit_Display.h"
#include <neopixel.h>
#include "Button.h"
#include "DFRobotDFPlayerMini.h"
#include "credentials.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "JsonParserGeneratorRK.h"
#include "Air_Quality_Sensor.h"
#include "Adafruit_BME280.h"
#include "IoTTimer.h"


/************ Global State (you don't need to change this!) ******************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>//must include /feeds/ before the feedname!
Adafruit_MQTT_Publish soilMoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilMoisture");
Adafruit_MQTT_Publish roomEnvironment = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roomEnvironment");
//Adafruit_MQTT_Subscribe subFeedButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/IoT14_buttonOnOff");

/************Declare Variables*************/
unsigned int lastTime; //unsigned means the value does not go negative. It will time out at approx 49 days.
int quality;
int bri = 35; //data type is int, bri is the name of the variable, and 35 is the initial value or assignment. bri is a mutable variable that can be changed later in the code.
int color;
int litPixel; //The pixels that are being lit up
float tempF;  // Global variable for temperature in Fahrenheit
float inHg;  // Global variable for pressure in inches of mercury
float tempC;
float pressPA;
float humidRH;
int status;
int currentTime;
int lastSecond;
int Moisture;
int i;

/************Declare Constants*************/
//Declorations are sequential, constants need to go before objects.
const int CLK = D4;
const int DIO = D5;
const int PIXELCOUNT = 19; //PIXELCOUNT name of constant, datatype is integer, initial value (assignment) will always hold 19
const int OLED_RESET=-1; //OLED_RESET is the name of the constant variable, it's a variable because it holds data, but it is not a regular variable and uses uppercase letters
const int hexAddressBME = 0x76; //I2C address of BME sensor. I2C can have 128 connections and the hexidicimal maps it.
const char degree = 0xF8;
const char percent = 0x25;
const int AUOT = A2; //is this int or float and may need to use D pin. Moisture sensor.
String dateTime, timeOnly;
const int Pump = S0;

//Define constants for NFC card reading
#define PN532_IRQ 2
#define POLLING 0
#define READ_BLOCK_NO 2


/************Declare Objects***************/
IoTTimer pumpTimer;  //IoTTimer is the class and pumpTimer is an object, or instance, of that class
Adafruit_NeoPixel pixel(PIXELCOUNT, SPI1, WS2812B); //Adafruit_NeoPixel is the class, pixel is the object, or instance, and can use mothods like pixel.begin() and pixel.setpixelcolor(). The first argument, PIXELCOUNT is defined below as a constant integer.
Adafruit_BME280 bme;
Button startCountdown(D3);//Button is class/type and startCountdown is the object and D3 is the argument
AirQualitySensor sensor(A1); //so confused???????? A0?
DFRobotDFPlayerMini myDFPlayer;//Default constructor (specicialized function initializes and doesn't need a value like a pin number, because it uses a default value from library
TM1637 tm1637(CLK,DIO);
bool keepCounting;//declares variable keepCounting, but does not initialize a value because it's a bool
bool nfcScanned = false;//declared variable nfcScanned, but initializes with value of false when program starts
Adafruit_SSD1306 display(OLED_RESET);
DFRobot_PN532_IIC nfc(PN532_IRQ, POLLING);
uint8_t dataRead[16] = {0};//unit8_t is datatype of variable, dataRead declares name of array that can hold 16 elements, the 0 initializes the array with the first element as 0 (zero initialization of array)


/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();
void printDetail(uint8_t type, int value);//Tells compiler a function named printDetail exists. Semicolon indicates it is only a declaration. when printDetail() is called it has two arguments (parameters), uint8_t is unsigned 8-bit integer and int is integer 
void displayNFCData();
bool countDown(bool restart = false, int countStart = 60);//bool is the return type, countdown is the name of the function and will return a datatype of true/false
void PixelFill (int startP, int endP, int color);

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

// setup() runs once, when the device is first turned on
void setup() {
  Serial.begin(9600); //Enables serial monitor
  waitFor(Serial.isConnected, 10000);  //wait for Serial monitor. Serial, not serial1 is the USB main connection.
  Serial1.begin(9600);//Begins Serial 1, but no handshake needed
  //delay(1000); //Don't see a need for a delay?

status = bme.begin(hexAddressBME);
  if (status == false) {
    Serial.printf("BME280 at address 0x%02X failed to start", hexAddressBME);
  }
  
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.display();
    display.clearDisplay();
   

    pixel.begin();
    pixel.setBrightness(bri); //bri variable can be changed in header from 0-255
    pixel.show(); //initialize all pixels off
    pixel.clear();




  //// Connect to Internet but not Particle Cloud
 // WiFi.on();
 // WiFi.connect();
 // while(WiFi.connecting()) {
 //   Serial.printf(".");
// }
 // Serial.printf("\n\n");

  // Setup MQTT subscription
  //mqtt.subscribe(&subFeedButton);
  //mqtt.subscribe(&subFeedSlider);//need to have this for each MQTT subscription or nothing comes through feed
  // Time.zone (-7); //MST = -7, MDT = -6
  // Particle.syncTime(); //Sync time with Particle Cloud

  myDFPlayer.begin(Serial1);
  
  Serial.printf("DFRobot DFPlayer Mini Demo\n");
  Serial.printf("Initializing DFPlayer ... (May take 3~5 seconds)\n");

  //WiFi.on();
  //WiFi.clearCredentials();
  //WiFi.setCredentials("IoTNetwork");
  
    tm1637.init();
    tm1637.set(7);
    tm1637.point(POINT_ON);

    keepCounting = false;

  while (!nfc.begin()) {
    Serial.printf("NFC initialization failed. Retrying...\n");
    delay(1000);
  }
    Serial.printf("NFC initialized. Waiting for a card...\n");

pinMode (AUOT,INPUT);//Used global variable instead of A2
pinMode (Pump,OUTPUT);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {

  tempC = bme.readTemperature(); //deg C
  pressPA = bme.readPressure(); //pascals
  humidRH = bme.readHumidity(); // %RH

  Moisture=analogRead(AUOT);
  
  currentTime = millis();
  tempF = ((tempC*9/5)+32); //Assigning value to global float tempF. If "float" added it will run a local iteration of float, which negates declaring them as floats in the header. It's also confusing because I have no idea if the global variable is being used or the local variable, especially problematic with more code.
  inHg = (pressPA*0.00029529983071445); //Assigning value to global float inHg
  
  if ((currentTime-lastSecond)>500) { //half second
    lastSecond = millis ();
    Serial.printf("Pressure %0.1f\n",inHg);
    Serial.printf("Humidity %0.1f %c\n",humidRH,0x25);
    Serial.printf("Temp %0.1f%cF\n",tempF,degree);
    Serial.printf("Moisture is %i\n",Moisture);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,16);
    display.printf("Temperature is %0.1f%c \n",tempF,degree);
    display.printf("Pressure is %0.01finHg \n",inHg);
    display.printf("Humidity is %0.02f%c \n", humidRH,percent);
    display.printf("Moisture is %i\n",Moisture);
    display.display();
    display.clearDisplay();
  }



  int quality = sensor.slope();

    //Serial.printf("Air quality %i\n",quality);
    //Serial.printf("Raw sensor input %0.2f\n",sensor.getValue());

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
        Serial.printf("High pollution! Force signal active. Quality: %i\n", quality);
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        Serial.printf("High pollution! Quality: %i\n", quality);
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
        Serial.printf("Low pollution! Quality: %i\n", quality);
    } else if (quality == AirQualitySensor::FRESH_AIR) {
        Serial.printf("Fresh air. Quality: %i\n", quality);
    }
  
  
  if (startCountdown.isClicked()) {
    countDown(true, 6); //testing change to 600
    keepCounting = true;
    nfcScanned = false; //Reset NFC scan flag
    myDFPlayer.volume(0); // Mute the player when starting a new countdown
}
if (keepCounting) {
      keepCounting = countDown();
    if (!keepCounting) {
      Serial.printf("Countdown is complete\n");
    if (!nfcScanned) {
      myDFPlayer.volume(20); //Set volume to 30 if NFC not scanned
      myDFPlayer.playFolder(1, 1); //Plays the first MP3 in folder 1
      }  
    }
  }

  static unsigned long lastNfcScanTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastNfcScanTime >= 100) { // 100ms delay between scans to reduce power consumption
    lastNfcScanTime = currentTime; 
  
  if (nfc.scan()) {
    nfcScanned = true; // Set NFC scan flag
    myDFPlayer.volume(10); // Lower volume when NFC scanned, but still finish the track 
   
 
  if (nfc.readData(dataRead, READ_BLOCK_NO) == 1) {
    Serial.printf("Block %d read success!\n", READ_BLOCK_NO);
    Serial.printf("Data read (string): %s\n", (char *)dataRead);
    displayNFCData();
  } 
  else {
    Serial.printf("Block %d read failure!\n", READ_BLOCK_NO);
    }
   }
  }
}

void displayNFCData() {
    
    color = 0X000FF;

   for (i=0; i<19; i++) { //order: initiallization, condition, incremement--where to start, where do you want to go, how do you want to get there
    pixel.setPixelColor (i,color);
    pixel.show();
    digitalWrite (Pump,HIGH); 
    delay (500);
    digitalWrite (Pump,LOW);
    
  }

  pixel.clear();
  pixel.show();
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 32);
  display.printf("%s\n", (char *)dataRead);
  display.display();
  delay(5000);

  display.clearDisplay();
  display.setTextSize(3);
  display.setCursor(12, 8);
  display.printf("Great\n Job!\n");
  display.display();
  delay(1800);
}

bool countDown(bool restart, int countStart) {
  static int count = 60;
  static unsigned long lastTime = 0;

  if (restart) {
    count = countStart;
  }

  if (millis() - lastTime > 1000) {
    lastTime = millis();
    count--;

    int min10 = count / 600;
    int min01 = (count / 60) % 10;
    int sec10 = (count % 60) / 10;
    int sec01 = (count % 60) % 10;

    tm1637.display(0, min10);
    tm1637.display(1, min01);
    tm1637.display(2, sec10);
    tm1637.display(3, sec01);

    Serial.printf("Countdown: %02d:%02d\n", count / 60, count % 60);
  }

  if (count <= 0) {
    return false;
  }

  return true;
} 


//litPixel = ((PIXELCOUNT/3015)*Moisture);
  //if (tempF>90) {
    //pixel.clear();
    //PixelFill (0, 15, red);
  //}

  //else (tempF<90) {
   // pixel.clear();
    //PixelFill (0, 15, blue);
  //}  

  //}

 //void PixelFill (int startP, int endP, int color) {
  //for (int litPixel=startP; litPixel<endP; litPixel++)
    //pixel.show();
    //pixel.clear();
   //pixel.show(); 
 //}