#include "Arduino.h"
#include "MotorDrive.h"

short status = "STOPPED";

MotorDrive::definePins(int Lsp, int Lf, int Lb, int Rsp, int Rf, int Rb) {
    int _Lsp = Lsp;
    int _Lf = Lf;
    int _Lb = Lb;
    int _Rsp = Rsp;
    int _Rf = Rf; 
    int _Rb = Rb;
    haltMotors();
}

MotorDrive::setOffset(int LMOffset) {
    int _OffSet = LMOffset; //Favours left   
}

MotorDrive::changeSpeed(int speed) {
    //Calculate speeds
    analogWrite(_Lsp, (speed + (speed/100 * _OffSet)));
    analogWrite(_rsp, speed);
}

MotorDrive::haltMotors() {
    status = "STOPPED";
    digitalWite(_Lf, LOW);
    digitalWrite(_Rf, LOW);
    digitalWrite(_Lb, LOW);
    digitalWrite(_Rb, LOW);
}

MotorDrive::getStatus() {
    return status;
}

MotorDrive::Forward() {
    haltMotors();
    status = "FORWARD";
    digitalWrite(_Lf, HIGH);
    digitalWrite(_Rf, HIGH);
}

MotorDrive::Backward() {
    haltMotors();
    status = "BACKWARD";
    digitalWrite(_Lb, HIGH);
    digitalWrite(_Rb, HIGH);
}

MotorDrive::Left() {
    haltMotors();
    status = "LEFT";
    digitalWrite(_Lb, HIGH);
    digitalWrite(_Rf, HIGH);
}

MotorDrive::Right() {
    haltMotors();
    status = "RIGHT";
    digitalWrite(_Lf, HIGH);
    digitalWrite(_Rb, HIGH);
}

MotorDrive::setOffset(int offset) {
    if (offset < 100 && offset > -100) {
        //This ensures that no values passed into the functions exceed to 100% differential limit
        if (offset == 0) {
            if (analogRead(_Rsp) > analogRead(_Lsp)) {
                ChangeSpeed(analogRead(_Rsp));
            }
            else {
                ChangeSpeed(analogRead(_Lsp));
            }
        }
        else if (offset < 0) {
            swerve(0); //resets both motors to the same speed
            //This means that the offset is a negative interger and needs to swerve to the left
            analogWrite(_Lsp, (analogRead(_Rsp) / 100 * offset) * -1); //Changes speed and ensures it is a positive interger
        }
        else {
            swerve(0); //resets both motors to the same speed
            //Offset is positive, swerve to the right
            analogWrite(_Lsp, analogRead(_Rsp) / 100 * offset);
        }
    }
}