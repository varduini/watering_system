/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Air_Quality_Sensor.h"
#include <math.h>
#include "Adafruit_BME280.h"


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);
#define OLED_RESET D4
Adafruit_SSD1306 display (OLED_RESET);
Adafruit_BME280 bme;

// Run the application and system concurrently in separate threads
//SYSTEM_THREAD(ENABLED);

// Show system, cloud connectivity, and application logs over USB
// View logs with CLI using 'particle serial monitor --follow'
//SerialLogHandler logHandler(LOG_LEVEL_INFO);

const int PUMPPIN = D16;
const int SOILPIN = A1;
String DateTime, TimeOnly ;
int lastTime;
const int DUSTPIN = D6;
int duration;
int lastRead;
int lowpulseoccupancy = 0;
int sampletime_ms = 30000;
float ratio;
float concentration;
bool status;


AirQualitySensor airSensor (A5);

int airSensing ();
void dataBME ();
int soilMoisture ();

// setup() runs once, when the device is first turned on
void setup() {

  pinMode (PUMPPIN, OUTPUT);
  pinMode (SOILPIN, INPUT);
  
  Serial.begin (9600);
  waitFor (Serial.isConnected,10000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.display ();

  Time.zone (-7); 
  Particle.syncTime ();

  airSensor.init ();

  pinMode(DUSTPIN,INPUT);
    //startTime = millis();
    //lastTime= -1000000;

  status=bme.begin (0x76);
    delay(2000);
    if (status==false) {
        Serial.printf ("BME280 at address 0x76 X failed to start" );
    }
    else {
    Serial.printf ("All good\n" );  
    }
    delay (5000);
}

void loop() {

// ensors reads BME, AQ, DUST

duration = pulseIn(DUSTPIN, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;

while ((millis()-lastRead) > 30000) //if the sampel time == 30s
{    
   ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    if ((millis()-lastTime > 60000)) {   
      Serial.printf ("lowpulseoccupancy=%i\n", lowpulseoccupancy);
        Serial.printf ("ratio=%f \n", ratio);
        Serial.printf ("concentration= %f\n", concentration);
        lowpulseoccupancy = 0;
        lastRead = millis();


 airSensing ();
Serial.printf ("Air Quality %i\n", airSensing ());
dataBME ();
soilMoisture ();
Serial.printf ("Soil moisture is %i\n", soilMoisture ());

lastTime = millis ();
    }
  }
}

void dataBME () {
  float tempC;
  float tempF;
  int pressPA;
  int humidRH;
  int hgmm;

  tempC=bme.readTemperature();
  pressPA=bme.readPressure ();
  hgmm=map(pressPA,0,133322,0,1000);
  humidRH=bme.readHumidity ();

  Serial.printf ("Temperature= %f\n Pressure=%i\n Humidity=%i\n", tempC, hgmm, humidRH);
}

int airSensing () {
  int pollution;
int quality = airSensor.slope();
     {
        if (quality == AirQualitySensor::FORCE_SIGNAL) {
            Serial.printf("High pollution! Force signal active! \n");}
        else if (quality == AirQualitySensor::HIGH_POLLUTION) {
            Serial.printf("High pollution!\n");}
        else if (quality == AirQualitySensor::LOW_POLLUTION) {
            Serial.printf("Low pollution!\n"); }
        else if (quality == AirQualitySensor::FRESH_AIR) {
            Serial.printf("Fresh air\n"); }
    } 
  pollution=airSensor.slope();
  return pollution;
}

int soilMoisture (){
  int moisture;

 moisture= analogRead(SOILPIN);

 if (moisture<2000) {
     while ((millis()-lastTime < 500)) {  
      digitalWrite (PUMPPIN, HIGH); }
      lastTime = millis ();
      digitalWrite (PUMPPIN, LOW);
 }

 return moisture;

}


//publish soil levels
//   DateTime= Time.timeStr (); // Current Date and Time from Particle Time class
//   TimeOnly= DateTime.substring (11 ,19) ;

// display.clearDisplay();

//   display.setTextSize(2);
//   display.setTextColor(WHITE);
//   display.setCursor (0,5);
//   display.printf ("%i \n %s \n", moisture, TimeOnly.c_str());
//   display.display();
//   delay (2000);

