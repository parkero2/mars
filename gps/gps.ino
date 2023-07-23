#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

#include <Vector.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <LCD_I2C.h>
#include <Wire.h>

/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

LCD_I2C lcd(0x27, 16, 4);
// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// Buffer vals
uint8_t hr;
uint8_t mn;
uint8_t sc;
double lon;
double lat;

// Real vals
uint8_t rhr;
uint8_t rmn;
uint8_t rsc;
double rlon;
double rlat;

byte refresh[] = {
    0x04,
    0x1E,
    0x15,
    0x11,
    0x11,
    0x15,
    0x0F,
    0x04};

byte locationn[] = {
    0x00,
    0x0E,
    0x11,
    0x15,
    0x11,
    0x0A,
    0x04,
    0x00};

byte timee[] = {
    0x0E,
    0x11,
    0x15,
    0x15,
    0x17,
    0x11,
    0x11,
    0x0E};

byte datee[] = {
    0x00,
    0x00,
    0x1F,
    0x1F,
    0x11,
    0x11,
    0x1F,
    0x00};

bool ongps = true;

void setup()
{

    Serial.begin(115200);
    ss.begin(GPSBaud);
    Serial1.begin(115200);
    Serial.println(F("DeviceExample.ino"));
    Serial.println(F("A simple demonstration of TinyGPSPlus with an attached GPS module"));
    Serial.print(F("Testing TinyGPSPlus library v. "));
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();
    lcd.begin(); // If you are using more I2C devices using the Wire library use lcd.begin(false)
    // this stop the library(LCD_I2C) from calling Wire.begin()
    lcd.backlight();

    lcd.createChar(0, refresh);
    lcd.createChar(1, datee);
    lcd.createChar(2, timee);
    lcd.createChar(3, locationn);

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange())
    {
    case MPU6050_RANGE_2_G:
        Serial.println("+-2G");
        break;
    case MPU6050_RANGE_4_G:
        Serial.println("+-4G");
        break;
    case MPU6050_RANGE_8_G:
        Serial.println("+-8G");
        break;
    case MPU6050_RANGE_16_G:
        Serial.println("+-16G");
        break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange())
    {
    case MPU6050_RANGE_250_DEG:
        Serial.println("+- 250 deg/s");
        break;
    case MPU6050_RANGE_500_DEG:
        Serial.println("+- 500 deg/s");
        break;
    case MPU6050_RANGE_1000_DEG:
        Serial.println("+- 1000 deg/s");
        break;
    case MPU6050_RANGE_2000_DEG:
        Serial.println("+- 2000 deg/s");
        break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth())
    {
    case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
    case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
    case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
    case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
    case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
    case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
    case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }
}

void loop()
{
    // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0)
        if (gps.encode(ss.read()))
            displayInfo();

    if (digitalRead(2))
    {
        ongps = !ongps;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(ongps ? "GPS ON" : "COMPASS ON");
        while (digitalRead(2));
        lcd.clear();
        return;
    }
    if (!ongps) {
        lcd.clear();
        lcd.setCursor(0, 1);

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        /* Print out the values */
        lcd.print("X: ");
        lcd.print(a.acceleration.x);
        lcd.setCursor(0, 2);
        lcd.print("Y: ");
        lcd.print(a.acceleration.y);
        lcd.setCursor(0, 3);
        lcd.print("Z: ");
        lcd.print(a.acceleration.z);

        Serial.println("");
        delay(500);
        return;
    }
    if (ongps)
    {
        if (millis() > 5000 && gps.charsProcessed() < 10)
        {
            Serial.println(F("No GPS detected: check wiring."));
            lcd.setCursor(16, 0);
            lcd.print("!");
            delay(500);
            return;
        }
    }
    
}

void displayInfo()
{
    lcd.clear();

    lcd.setCursor(0, 1);
    lcd.write(3);
    lcd.print(" ");
    lcd.setCursor(0, 0);
    lcd.write(3);
    lcd.print(" ");
    Serial.print(F("Location: "));
    // lcd.print(F("Location: "));
    if (gps.location.isValid())
    {
        rlon = gps.location.lng();
        rlat = gps.location.lat();

        Serial.print(rlon, 6);
        Serial.print(F(","));
        Serial.print(rlat, 6);

        lcd.print(rlat, 6);
        if (!rlat == lat)
        {
            lcd.setCursor(15, 0);
            lcd.write(0);
        }
        lcd.setCursor(0, 1);
        lcd.write(3);
        lcd.print(" ");
        lcd.print(rlon, 6);
        if (!rlon == lon)
        {
            lcd.setCursor(15, 1);
            lcd.write(0);
        }
        lon = rlon;
        lat = rlat;
    }
    else
    {
        Serial.print(F("PENDING"));
        lcd.print(F("PENDING"));
    }

    // Date and time
    Serial.print(F("  Date/Time: "));

    lcd.setCursor(0, 2);
    lcd.write(1);
    lcd.print(" ");
    if (gps.date.isValid())
    {
        Serial.print(gps.date.month());
        Serial.print(F("/"));
        Serial.print(gps.date.day());
        Serial.print(F("/"));
        Serial.print(gps.date.year());
        lcd.print(gps.date.month());
        lcd.print(F("/"));
        lcd.print(gps.date.day());
        lcd.print(F("/"));
        lcd.print(gps.date.year());
    }
    else
    {
        Serial.print(F("PENDING"));
        lcd.print(F("PENDING"));
    }
    lcd.setCursor(0, 3);
    Serial.print(F(" "));

    // Time
    if (gps.time.isValid())
    {
        rhr = gps.time.hour();
        rmn = gps.time.minute();
        rsc = gps.time.second();

        lcd.write(2);
        lcd.print(" ");

        if (rhr < 10)
            Serial.print(F("0"));
        Serial.print(rhr);
        Serial.print(F(":"));
        lcd.print(rhr);
        lcd.print(F(":"));

        if (rmn < 10)
            Serial.print(F("0"));
        Serial.print(rmn);
        Serial.print(F(":"));
        lcd.print(rmn);
        lcd.print(F(":"));

        if (rsc < 10)
            Serial.print(F("0"));
        Serial.print(rsc);
        Serial.print(F("."));
        lcd.print(rsc);
        lcd.print(F("."));

        // Refresh detection
        if ((hr != rhr) || (mn != rmn) || (sc != rsc))
        {
            lcd.setCursor(15, 3);
            lcd.write(0);
        }

        /**if (gps.time.centisecond() < 10) Serial.print(F("0"));
        Serial.print(gps.time.centisecond());*/
        hr = rhr;
        mn = rmn;
        sc = rsc;
    }
    else
    {
        Serial.print(F("PENDING"));
        lcd.print(F("PENDING"));
    }

    Serial.println();
    delay(500);
}