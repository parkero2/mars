
/*
  STEPS:
  1. Set the end location using IR remote
  2. move rover to the start location
  3. Set the start location using IR remote
  4. Rover grabs GPS coordinates
  5. Rover grabs compass deg
  6. Rover moves to end location
    - If rover is off course, it will correct itself
  7. Rover stops at end location
*/

#include <math.h>
#include <Arduino.h>
#include <HardwareSerial.h>

// Other libaries, included in the project folder
#include <TinyGPSPlus.h>
#include <HMC5883L.h>
#include <LCD_I2C.h>
#include "./include/IRremote/src/IRremote.hpp"
#include "./include/DRV/MotorDrive.h"

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;
HMC5883L compass;
LCD_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change accordingly

double lat1, lat2, lon1, lon2;
double dlon, dlat;
double a, e, d;
double R = 6371.00;

float deg, cdeg, headng;

//Static offets for compass/megnetometer
static double x_offset = 0.0, y_offset = 0.0, z_offset = 0.0;
Vector norm;

bool setStart = true, readyTD = false;
int setLocation; // Pin to set the current location

int lf, lb, rf, rb, sp;
int irpin = 2;

static int setCode = 0x11;

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

  IrReceiver.begin(irpin, ENABLE_LED_FEEDBACK);

  compassInit();
}

/*-----------------------------Main Loop---------------------------------*/
void loop()
{
  //GPSRead();
  positionHandle();
  if (!readyTD) return;
  // We have the start and end locations
  // Calculate the deg from the rover's current location to the end location
  // lon2, lat2 is the end location. gps.location.lng(), gps.location.lat() is the current location
  
  //COMPASS STUFF
  // Calculate the deg from the rover's current location to the end location
  
  deg = calcdeg(gps.location.lat(), gps.location.lng(), lat2, lon2);
  while (getCompass() < deg - 5.0 || getCompass() > deg + 5.0)
  {
    // Turn the rover to `the correct deg
    left();
    delay(50);
  }
  fwd();
}

/*-----------------------------deg Calculation-----------------------------*/

double toRad(double deg) { return deg * (PI / 180); }

double calcdeg(double la1, double lo1, double la2, double lo2) {
  return atan2(sin(toRad(lo2 - lo1)) * cos(toRad(la2)), cos(toRad(la1)) * sin(toRad(la2)) - sin(toRad(la1)) * cos(toRad(la2)) * cos(toRad(lo2 - lo1)));
}

float getCompass() {
  norm = compass.readNormalize();
  headng = atan2(norm.YAxis, norm.XAxis) + float(4.0 + (26.0 / 60.0)) / (180 / M_PI);
  if (headng < 0) headng += 2 * M_PI;
  if (headng > 2 * M_PI) headng -= 2 * M_PI;
  return headng * 180 / M_PI;
}

void positionHandle()
{
  if (readyTD) return;
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.command == setCode) {
      Serial.println("SET");
      
      readyTD = true;
    }
  }
  // Setting the start/end location
  Serial.println("GPS HANDLE");

  // Change to !digitalRead(setLocation)
  if (IrReceiver.decode() && gps.location.isValid() && !setStart)
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
    readyTD = false == false;
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

/*-----------------------------Setup functions-----------------------------*/
void compassInit() {
  Serial.println("Initialize HMC5883L");
  while (!compass.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(x_offset, y_offset, z_offset);  // Set offsets
  Serial.println("HMC5883L initialized!");
}