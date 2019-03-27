// Sylvain Giraud – February 19, 2019
// Format output in JSON format that Gladys can read immediately
// By default use:
//     a Arduino Nano
//     Si7021 to get temperature/humidity
//     MQ135 to get aur quality
//     a USB cable to Gladys (Raspberry Pi or any other)

/* Output looks like:
{"devicetype":13,"value":50}
{"devicetype":14,"value":20}
{"devicetype":15,"value":21}
*/

#include <ArduinoJson.h>  // Librairie pour manipuler les messages JSON
// la bonne version: https://github.com/bblanchon/ArduinoJson
// la version de l'IDE ne marche pas (Error: StaticJsonBuffer not found)

// For test only. Do not plug Gladys if enabled.
#define DEBUG false
#define INFO false

// What sensors are connected ?
// ( Do not connect Si7021 and DHT21 at same time)
#define SI7021_Sensor
//#define DHT_Sensor
#define MQ135_Sensor


/* **************  Si7021    **************** */
#ifdef SI7021_Sensor
// Si_7021_bare
// this sketch reads temperature and relative humidity from a SI7021 I2C sensor
// sketch reports to Serial Monitor
// Floris Wouterlood – February 19, 2018
// public domain
// library
#include <Wire.h>

// declared variables for Si7021
const int ADDR = 0x40;    // Connected A4=SDA / A5=SCL
int X0,X1,Y0,Y1,Y2,Y3;
float X,Y,X_out,Y_out1,Y_out2;
#endif
/* ************* End of Si7021 ************* */

/*  ************* MQ135 Config **************** */
#ifdef MQ135_Sensor
/*
  Programme de test du MQ135 permettant de mesurer la présence de polluants dans l'atmosphère (CO, CO2 Alcool, fumées...)
  Pour plus d'infos ici http://www.projetsdiy.fr/mq135-mesure-qualite-air-polluant-arduino
  Utilise la librairie mq135.cpp mise à disposition par Georg Krocker https://github.com/GeorgK/MQ135
  Projets DIY - Mars 2016 - www.projetsdiy.fr
*/ 
#include "MQ135.h"
const int mq135Pin = 0;             // Pin sur lequel est branché de MQ135 : Analog 0 = A0 
MQ135 gasSensor = MQ135(mq135Pin);  // Initialise l'objet MQ135 sur le Pin spécifié
#endif
/*  ************* End of MQ135 Config **************** */

/*  ************* DHT Config **************** */
#ifdef DHT_Sensor
#include <DHT.h>
#define DHTPIN 2     // what pin we're connected to

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302)
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
DHT dht(DHTPIN, DHTTYPE);
#endif
/*  ************* End of DHT Config **************** */

// Make global variables to test changes between periods, if needed
// List of sensors measures variables:
float h = 50.0;
float t = 20.0;
float ppm = 0;

struct sensorData {       // Création du type "sensorData" pour enregistrer les mesures.
  int id;
  float valeur;
};

int devicetype_Temperature = 13; 	// id du devicetype de gladys "température"
int devicetype_Humidity    = 14;	// id du devicetype de gladys "Humidité"
int devicetype_Quality     = 15; 	// id du devicetype de gladys "MQ135"

#define SENSORDATA_JSON_SIZE (JSON_OBJECT_SIZE(2))  // Variable indiquant la taille du buffer qui sera utilisé pour manipuler le JSON

void setup() {
  Serial.begin(9600);

/* **************  Si7021    **************** */
#ifdef SI7021_Sensor
  Wire.begin ();
#endif
/* ************* End of Si7021 ************* */
  
//  dht.begin();        // Initialisation de l'objet dht
}

void loop() {
  
  sensorData dataT;       // Objet pour l'enregistrement de la température
  sensorData dataH;       // Objet pour l'enregistrement de l'humidité
  sensorData dataTr;      // Objet pour l'enregistrement de la température ressentie.
  sensorData dataQ;       // Objet pour l'enregistrement de la qualité.


/* **************  Si7021    **************** */
#ifdef SI7021_Sensor
readAir();
#endif
/* ************* End of Si7021 ************* */

/*  ************* DHT read **************** */
#ifdef DHT_Sensor
  // Lecture du taux d'humidité
  float h = dht.readHumidity();
  // Lecture de la température en °C
  float t = dht.readTemperature();
  // Pour lire la température en farenheit
  //float f = 73;  //dht.readTemperature(true);
#endif
/*  ************* End of DHT read **************** */

  // Stop le programme et renvoie un message d'erreur si le capteur ne renvoie aucune mesure
  if (isnan(h) || isnan(t)) {
    if (DEBUG) {Serial.println("Echec de lecture !");}
  } else {

    // Message JSON humidité
    dataT.id = devicetype_Temperature;
    dataT.valeur = t;
    serialize(dataT);

    // Message JSON température
    dataH.id = devicetype_Humidity;
    dataH.valeur = h;
    serialize(dataH);

    /*
    // Message JSON température ressentie
    dataTr.id = devicetype_id3;
    dataTr.valeur = 21;   // dht.convertFtoC(dht.computeHeatIndex(f, h));
    serialize(dataTr);
    */

#ifdef MQ135_Sensor
    // Message JSON quality
    dataQ.id = devicetype_Quality;
    dataQ.valeur = ppm;
    serialize(dataQ);
#endif
  }
  // Délai de 15 min entre chaque mesure. La lecture prend 250 millisecondes pour un DHT22, et 1 seconde pour le DT11
  delay(900000);
  //delay(60000);

}

void serialize(sensorData &data)
{
  StaticJsonBuffer<55> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["devicetype"] = data.id;
  root["value"] = data.valeur;
  root.printTo(Serial);               // Impression du JSON sur le serial
  Serial.println("");                 // permet de séparer les JSON 
}

// Version Si7021 + MQ135
void readAir()
{
    // start temperature measurement
    Wire.beginTransmission (ADDR);
    Wire.write (0xE3);
    Wire.endTransmission ();
    
    // read temperature
    Wire.requestFrom (ADDR,2);
    
    if(Wire.available ()<=2);
    {
    X0 = Wire.read ();
    X1 = Wire.read ();
    X0 = X0<<8;
    X_out = X0+X1;
    }
    
    // calculate temperature
    X=(175.72*X_out)/65536;
    X=X-46.85;
    
    // start relative humidity measurement
    Wire.beginTransmission (ADDR);
    Wire.write (0xE5);
    Wire.endTransmission ();
    
    // read relative humidity data
    Wire.requestFrom (ADDR,2);
    if(Wire.available()<=2);
    {
    Y0 = Wire.read ();
    Y2 = Y0/100;
    Y0 = Y0%100;
    Y1 = Wire.read ();
    Y_out1 = Y2*25600;
    Y_out2 = Y0*256+Y1;
    }
    
    // calculate relative humidity
    Y_out1 = (125*Y_out1)/65536;
    Y_out2 = (125*Y_out2)/65536;
    Y = Y_out1+Y_out2;
    Y = Y-6;

    // Push results to global variables
    t = X; h = Y;   
     
#ifdef MQ135_Sensor
    //float ppm = gasSensor.getPPM();
    // SGi correction with temperature and humidity
    ppm = gasSensor.getCorrectedPPM(h,t);
#endif

    if (DEBUG) 
       {
        // send temperature and humidity to Serial Monitor
        Serial.print ("t=");
        Serial.print (X,1);
        Serial.print ("C;");
        Serial.print ("h=");
        Serial.print (Y,1);
        Serial.print ("%;");
        Serial.print (ppm,1);
        Serial.print ("ppm;");
       }

}
// End of version Si7021

