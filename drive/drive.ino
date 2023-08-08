int rf = 10, rb = 11, lf = 9, lb = 8;
int spdr, spdl;
int dmode = 0;

void fwd()
{
    analogWrite(rf, spdr);
    digitalWrite(rb, LOW);
    analogWrite(lf, spdl);
    digitalWrite(lb, LOW);
    dmode = 1;
}

void bwd()
{
    digitalWrite(rf, LOW);
    analogWrite(rb, spdr);
    digitalWrite(lf, LOW);
    analogWrite(lb, spdl);
    dmode = 2;
}

void right()
{
    digitalWrite(rf, LOW);
    analogWrite(rb, spdr);
    analogWrite(lf, spdl);
    digitalWrite(lb, LOW);
    dmode = 3;
}

void left()
{
    analogWrite(rf, spdr);
    digitalWrite(rb, LOW);
    digitalWrite(lf, LOW);
    analogWrite(lb, spdl);
    dmode = 4;
}

void stop()
{
    digitalWrite(rf, LOW);
    digitalWrite(rb, LOW);
    digitalWrite(lf, LOW);
    digitalWrite(lb, LOW);
}

void setSpeed(int l, int r = -1)
{
    if (r == -1) r = l;
    spdr = r;
    spdl = l;
    switch (dmode)
    {
    case 1:
        fwd();
        break;
    case 2:
        bwd();
        break;
    case 3:
        right();
        break;
    case 4:
        left();
        break;
    default:
        stop();
        break;
    }
}

void setup()
{
    pinMode(rf, OUTPUT);
    pinMode(rb, OUTPUT);
    pinMode(lf, OUTPUT);
    pinMode(lb, OUTPUT);
    setSpeed(255);
}

void loop()
{
    fwd();
    delay(1000);
    bwd();
    delay(1000);
    right();
    delay(1000);
    left();
    delay(1000);
}