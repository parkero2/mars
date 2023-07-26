#include <TinyGPSPlus.h>
#include <LCD_I2C.h>

/* 
   This sample sketch should be the first you try out when you are testing a TinyGPSPlus
   (TinyGPSPlus) installation.  In normal use, you feed TinyGPSPlus objects characters from
   a serial NMEA GPS device, but this example uses static strings for simplicity.
*/

// A sample NMEA stream.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// The TinyGPSPlus object
TinyGPSPlus gps;
LCD_I2C lcd(0x27, 20, 4);

void setup()
{
  Serial.begin(9600);

  lcd.begin();
  lcd.backlight();

lcd.setCursor(0,0);
  lcd.println("Getting my shit together");
  delay(1000);

  Serial.println(F("BasicExample.ino"));
  Serial.println(F("Basic demonstration of TinyGPSPlus (no device needed)"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();

  while (Serial1.available() > 0)
    if (gps.encode(Serial1.read()))
      displayInfo();

  Serial.println();
  Serial.println(F("Done."));
}

void loop()
{while (*gpsStream)
    if (gps.encode(Serial1.read()))
      displayInfo();
    delay(500);
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println("Location: ");
    lcd.println(gps.location.lat());
    lcd.println(gps.location.lng());
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}