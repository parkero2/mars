#include <math.h>
#include <Arduino.h>

// Other libaries, included in the project folder
#include "include/LCD_I2C/src/LCD_I2C.h"
#include "include/TinyGPSPlus/src/TinyGPSPlus.h"
#include "include/IRremote/src/IRremote.h"

/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;
LCD_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change according 

//updateGPS(); will reset Serial communication between the Arduino and the GPS module

double lat1, lat2, lon1, lon2;
double latR1, latR2, lonR1, lonR2;
double dlon, dlat;
double a, e, d;
double R = 6371.00;
double toDegrees = 57.295779;
char sb[10];

bool setStart = true;

int setLocation; // Pin to set the current location

/*-----------------------------Setup---------------------------------*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
   lcd.begin(); // If you are using more I2C devices using the Wire library use lcd.begin(false)
                 // this stop the library(lcd_I2C) from calling Wire.begin()
    lcd.backlight();
  Serial1.begin(GPSBaud);

  //displayInfo();
  lcd.clear();
  pinMode(setLocation, INPUT_PULLUP); // Connect 1k resistor between pin and +5V
}

void displayInfo()
{
 
  if (gps.location.isValid())
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println(gps.location.lat());
    lcd.setCursor(0, 1);
    lcd.println(gps.location.lng());
    delay(10000);
  }
 
  Serial.println();
}

void updateGPS() {
  //Reset GPS communication
  Serial1.begin(GPSBaud);
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read()))
      displayInfo();
  }
}

/*-----------------------------Main Loop---------------------------------*/
void loop() {

  //Setting the start/end location
  if (!digitalRead(setLocation)) {
    lcd.clear();
    //Set the current location
    setStart ? lat1 = gps.location.lat() : lat2 = gps.location.lat();
    setStart ? lon1 = gps.location.lng() : lon2 = gps.location.lng();
    String Start = setStart ? "Start loc: "  + String(lat1, 6) + ", " + String(lon1, 6) : "End loc" + String(lat1, 6) + ", " + String(lon1, 6);
    lcd.print(Start);
    setStart = !setStart;
    while (!digitalRead(setLocation));
  }

  // This sketch displays information every time a new sentence is correctly encoded.

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    delay(1000);
    while(true);
  }
  while(!Serial1.available() > 0);
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read()))
      displayInfo();
  }
  //Get Starting GPS Coordinates
  lon1 = gps.location.lng();
  lat1 = gps.location.lat();
  String Start = "GPS Starting Location: " + String(lat1, 6) + ", " + String(lon1, 6);
  lcd.print(Start);
  delay(10000);

  //Get Destination GPS Coordinates
  lat2 = -37.677285;
  lon2 = 176.130491;
  String Dest = "GPS Destination Location: " + String(lat2, 6) + ", " + String(lon2, 6);
  Serial.print(Dest);
  delay(10000);

  calcdist(); //Call the distance and bearing calculation function

}

/*-----------------------------Distance & Bearing Calculator---------------------------------*/
void calcdist(){ //This is a haversine based distance calculation formula
  //This portion converts the current and destination GPS coords from decDegrees to Radians
  lonR1 = lon1*(PI/180);
  lonR2 = lon2*(PI/180);
  latR1 = lat1*(PI/180);
  latR2 = lat2*(PI/180);
  
  //This portion calculates the differences for the Radian latitudes and longitudes and saves them to variables
  dlon = lonR2 - lonR1;
  dlat = latR2 - latR1;
  
  //This portion is the Haversine Formula for distance between two points. Returned value is in KM
  a = (sq(sin(dlat/2))) + cos(latR1) * cos(latR2) * (sq(sin(dlon/2)));
  e = 2 * atan2(sqrt(a), sqrt(1-a)) ;
  d = R * e;

  lcd.println();
  Serial.print("Distance to destination(KM): ");
  //lcd.println(a);
  //lcd.println(e);
  lcd.println(d, 6);
  lcd.println();
  
  //This portion is the Haversine Formula for required bearing between current location and destination. Returned value is in Degrees
  double x = cos(latR2)*sin(lonR2-lonR1); //calculate x
  
  lcd.print("X = ");
  lcd.println(x, 6);

  double y = cos(latR1)*sin(latR2)-sin(latR1)*cos(latR2)*cos(lonR2-lonR1); //calculate y

  lcd.print("Y = ");
  lcd.println(y, 6);
  float brRad = atan2(x, y); //return atan2 result for bearing. Result at this point is in Radians

  lcd.print("atan2(x, y) (Radians) = ");
  lcd.println(brRad, 6);

  float reqBear = toDegrees*brRad;
  lcd.print("Bearing: ");
  lcd.println(reqBear, 4);
}