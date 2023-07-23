#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

int rf = 10, rb = 11, lf = 6, lb = 9;
int trig = 7, echo = 8;

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
#include <LCD_I2C.h>

LCD_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// Real vals
uint8_t rhr;
uint8_t rmn;
uint8_t rsc;
double rlon;
double rlat;

int sonicSense()
{
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    int duration = pulseIn(echo, HIGH, 300000);
    int distance = duration * 0.034 / 2; // cm
    return distance;
}

void moveForward()
{
    digitalWrite(rf, HIGH);
    digitalWrite(rb, LOW);
    digitalWrite(lf, HIGH);
    digitalWrite(lb, LOW);
}

void moveBackward()
{
    digitalWrite(rf, LOW);
    digitalWrite(rb, HIGH);
    digitalWrite(lf, LOW);
    digitalWrite(lb, HIGH);
}

void turnRight()
{
    digitalWrite(rf, LOW);
    digitalWrite(rb, HIGH);
    digitalWrite(lf, HIGH);
    digitalWrite(lb, LOW);
}

void turnLeft()
{
    digitalWrite(rf, HIGH);
    digitalWrite(rb, LOW);
    digitalWrite(lf, LOW);
    digitalWrite(lb, HIGH);
}
void setup()
{
    pinMode(rf, OUTPUT);
    pinMode(rb, OUTPUT);
    pinMode(lf, OUTPUT);
    pinMode(lb, OUTPUT);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);

    Serial.begin(9600);
    moveForward();
    delay(1000);

    lcd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);

    ss.begin(GPSBaud);
    digitalWrite(rf, HIGH);
    digitalWrite(lf, HIGH);
}

int d;

void loop()
{
    while (ss.available() > 0)
        if (gps.encode(ss.read()))
            lcd.clear();
            Serial.println(gps.location.lat(), 6);
            Serial.println(gps.location.lng(), 6);
            lcd.setCursor(0, 0);
            lcd.print("Lat: ");
            lcd.print(gps.location.lat(), 6);
            lcd.setCursor(0, 1);
            lcd.print("Lng: ");
            lcd.print(gps.location.lng(), 6);


    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
        Serial.println(F("No GPS detected: check wiring."));
        lcd.setCursor(16, 0);
        lcd.print("!");
        delay(500);
    }

    // Serial.println(sonicSense());
    // return;
    //    d = sonicSense();
    // if (d < 0) return;
    /**Serial.println("Distance: " + String(d) + " cm");
    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(d);
    lcd.print("     ");
    if (d > 100)
    {
        Serial.println("Searching....");
        turnLeft();
        delay(100);
    }
    else
    {
        Serial.println("Moving forward....");
        moveForward();
        delay(100);
    }*/
}