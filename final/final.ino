
/*
  STEPS:
  1. Set the end location using IR remote
  2. move rover to the start location
  3. Set the start location using IR remote
  4. Rover grabs GPS coordinates
  5. Rover grabs compass heading
  6. Rover moves to end location
    - If rover is off course, it will correct itself
  7. Rover stops at end location
*/

#include <math.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// Other libaries, included in the project folder
#include <TinyGPSPlus.h>
#include <LCD_I2C.h>
#include "./include/IRremote/src/IRremote.h"
#include "./include/DRV/MotorDrive.h"

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;
LCD_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change accordingly

double lat1, lat2, lon1, lon2;
double dlon, dlat;
double a, e, d;
double R = 6371.00;

bool setStart = true;
int setLocation; // Pin to set the current location

int lf, lb, rf, rb, sp;
int irpin = 2;

/*-----------------------------Setup---------------------------------*/
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  lcd.begin(); // If you are using more I2C devices using the Wire library use lcd.begin(false)
               // this stop the library(lcd_I2C) from calling Wire.begin()
  lcd.backlight();
  Serial1.begin(GPSBaud);

  // displayInfo();
  lcd.clear();
  pinMode(setLocation, INPUT_PULLUP); // Connect 1k resistor between pin and +5V
  Serial.println("READY");
  delay(1000);

  // IR Receiver
  pinMode(irpin, INPUT);

  // Motor Driver
  pinMode(lf, OUTPUT);
  pinMode(lb, OUTPUT);
  pinMode(rf, OUTPUT);
  pinMode(rb, OUTPUT);
  pinMode(sp, OUTPUT);
}

/*-----------------------------Main Loop---------------------------------*/
void loop()
{
  //GPSRead();
  positionHandle();
}

void positionHandle()
{
  // Setting the start/end location
  Serial.println("GPS HANDLE");

  // Change to !digitalRead(setLocation)
  if (digitalRead(setLocation) && gps.location.isValid())
  {                                                                   // if the button is depressed (PULL UP), and GPS is valid
    setStart ? lat1 = gps.location.lat() : lat2 = gps.location.lat(); // If setStart is true, set lat1, else set lat2
    setStart ? lon1 = gps.location.lng() : lon2 = gps.location.lng(); // If setStart is true, set lon1, else set lon2
    displayLocation();
    Serial.println("GPS set to: ");
    Serial.print("Lat: ");
    Serial.println(gps.location.lat());
    Serial.print("Lng: ");
    Serial.println(gps.location.lng());
    lcd.setCursor(4, 0);
    lcd.println("GPS LOC SET ✔️"); // the tick might break shit; that's not my problem
    setStart = !setStart;
    while (digitalRead(setLocation));
  }
  else if (digitalRead(setLocation))
  {
    Serial.println("GPS cannot be validated");
    lcd.clear();
    lcd.setCursor(0, 0);
    return lcd.println("GPS INVALID");
  }
}

/*-----------------------------Start of GPS---------------------------------*/

void displayLocation()
{

  if (gps.location.isValid())
  {
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Lat: ");
    lcd.println(gps.location.lat());
    lcd.setCursor(1, 2);
    lcd.print("Lng: ");
    lcd.println(gps.location.lng());
    Serial.print("Lat: ");
    Serial.println(gps.location.lat());
    Serial.print("Lng: ");
    Serial.println(gps.location.lng());
  }
  else
  {
    lcd.clear();
    lcd.println("Location not found");
  }
  Serial.println();
}

void GPSRead()
{
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    delay(1000);
    while (true)
      ;
  }
  while (!Serial1.available() > 0)
    ;
  while (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
      displayLocation();
  }
}
/*-----------------------------End of GPS---------------------------------*/

/*-----------------------------Motor Control---------------------------------*/
void fwd()
{
  digitalWrite(lf, HIGH);
  digitalWrite(lb, LOW);
  digitalWrite(rf, HIGH);
  digitalWrite(rb, LOW);
}

void bwd()
{
  digitalWrite(lf, LOW);
  digitalWrite(lb, HIGH);
  digitalWrite(rf, LOW);
  digitalWrite(rb, HIGH);
}

void left()
{
  digitalWrite(lf, LOW);
  digitalWrite(lb, HIGH);
  digitalWrite(rf, HIGH);
  digitalWrite(rb, LOW);
}

void right()
{
  digitalWrite(lf, HIGH);
  digitalWrite(lb, LOW);
  digitalWrite(rf, LOW);
  digitalWrite(rb, HIGH);
}

void stop()
{
  digitalWrite(lf, LOW);
  digitalWrite(lb, LOW);
  digitalWrite(rf, LOW);
  digitalWrite(rb, LOW);
}
