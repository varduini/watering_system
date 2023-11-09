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
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"


// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(AUTOMATIC);
//#define OLED_RESET D4
Adafruit_SSD1306 display (-1);
Adafruit_BME280 bme;
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

//feeds
Adafruit_MQTT_Publish soilMoistureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/soilmoisturefeed");
Adafruit_MQTT_Publish temperatureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperaturefeed");
Adafruit_MQTT_Publish pressureFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressurerefeed");
Adafruit_MQTT_Publish humidityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidityfeed");
Adafruit_MQTT_Subscribe waterButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterbutton"); 
Adafruit_MQTT_Publish airQualityFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/airqualityfeed");

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

int sampletime_ms = 30000;
float ratio;
float concentration;
bool status;
float subValue;
int dustTimer;

AirQualitySensor airSensor (A5);

//Declare functions:
int airSensing ();
void dataBME ();
int soilMoisture ();
void MQTT_connect();
bool MQTT_ping();
float getDust (int DUSTPIN);

// setup() runs once, when the device is first turned on
void setup() {

  pinMode (PUMPPIN, OUTPUT);
  pinMode (SOILPIN, INPUT);
  
  Wire.begin(); 

  Serial.begin (9600);
  waitFor (Serial.isConnected,10000);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
  display.display ();

  Time.zone (-7); 
  Particle.syncTime ();

  airSensor.init ();

  mqtt.subscribe(&waterButton);
  pinMode(DUSTPIN,INPUT);

    //startTime = millis();
    //lastTime= -1000000;

  status=bme.begin(0x76);
    delay(2000);
    if (status==false) {
        Serial.printf ("BME280 at address 0x76 X failed to start" );
    }
    else {
    Serial.printf ("All good\n" );  
    }
    delay (5000);

  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
}

void loop() {
  MQTT_connect();
  MQTT_ping();

Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &waterButton) {
      subValue = atof((char *)waterButton.lastread);
      //Serial.printf("got value\n");
      if (subValue == 1){
      digitalWrite (PUMPPIN, HIGH);
      Serial.printf ("Watering the plant\n");
      delay (500);
      digitalWrite (PUMPPIN, LOW);
      }
        }
    }

if ((millis()-dustTimer)>300000) { 

airSensing ();
//Serial.printf ("Air Quality %i\n", airSensing ());
dataBME ();
//soilMoisture ();
Serial.printf ("Soil moisture is %i\n", soilMoisture());
concentration= getDust (DUSTPIN);
Serial.printf ("concentration= %0.2f\n", concentration);
   if(mqtt.Update()) {
    soilMoistureFeed.publish(soilMoisture());}
    dustTimer=millis();
}
//       }
//lastTime = millis ();

    }


float getDust(int DUSTPIN) {
  float concentration;
  int duration;
  int lowpulseoccupancy;
  unsigned int startTime;

  lowpulseoccupancy = 0;
 
  while ((millis()-startTime)<30000) {
    duration = pulseIn(DUSTPIN, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;
  
  }

   ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
        startTime = millis ();
    return concentration;
}


void dataBME() {
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
     if(mqtt.Update()) {
    temperatureFeed.publish(tempC);
    pressureFeed.publish(hgmm);
    humidityFeed.publish (humidRH);
  }
}

int airSensing() {
  int pollution;
int quality = airSensor.slope();

        if (quality == AirQualitySensor::FORCE_SIGNAL) {
            Serial.printf("High pollution! Force signal active! \n");
            }
        else if (quality == AirQualitySensor::HIGH_POLLUTION) {
            Serial.printf("High pollution!\n");
            }
        else if (quality == AirQualitySensor::LOW_POLLUTION) {
            Serial.printf("Low pollution!\n");
            }
        else if (quality == AirQualitySensor::FRESH_AIR) {
            Serial.printf("Fresh air\n");
            }
    
  
  if(mqtt.Update()) {
    airQualityFeed.publish(quality);

      } 
  pollution=airSensor.slope();
  return pollution;
}

int soilMoisture(){
  int moisture;

 moisture= analogRead(SOILPIN);

 //Serial.printf ("Soil moisture is %i\n", soilMoisture ());
   if(mqtt.Update()) {
    soilMoistureFeed.publish(moisture);}

 if (moisture<2000) {
     //while ((millis()-lastTime < 500)) {  
      digitalWrite (PUMPPIN, HIGH);
       delay (500); 
      //}
      //lastTime = millis ();
      digitalWrite (PUMPPIN, LOW);
  //publish soil levels

      
  // DateTime= Time.timeStr (); // Current Date and Time from Particle Time class
  // TimeOnly= DateTime.substring (11 ,19) ;

display.clearDisplay();
display.setTextSize(2);
display.setTextColor(WHITE);
display.setCursor (0,5);
display.printf ("moisture=%i \n, watering plant\n", moisture);
display.display();
delay (2000);
display.clearDisplay();
 }

 else {
  display.clearDisplay();

display.setTextSize(2);
display.setTextColor(WHITE);
display.setCursor (0,5);
display.printf ("optimal soil moisture \n");
display.display();
delay (2000);
display.clearDisplay();
 }

 return moisture;

}


void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
