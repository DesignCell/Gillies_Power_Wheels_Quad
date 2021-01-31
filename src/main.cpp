#include <Arduino.h>

/*
2020.08.21 
DesignCell

Completed:
Forward & Reverse
Hardcoded Ramp interval
No PID without feedback
Hardcoded reverse reduction factor
Adjustable speed setpoint
Hardcoded upper limit 

Open:
Idle sleep to reduce power draw
Convert limits to RPM or FPS vs 0-254.

/**/

// BTS7960
uint8_t R_EN = 5;   // R_EN : OUTPUT, Forward Drive Enable, Active High, Low Disable
uint8_t R_PWM = 6;  // RPWM : OUTPUT, Forward PWM, Active High
uint8_t R_IS = 7;   // R_IS : INPUT, Forward Drive Side Current Alarm,

uint8_t L_EN = 8;   // L_EN : OUTPUT, Reverse Drive Enable, Active High, Low Disable
uint8_t L_PWM = 9;  // LPWM : OUTPUT, Reverse PWM, Active High
uint8_t L_IS = 10;  // L_IS : INPUT, Reverse Drive Side Current Alarm.

                    // Vcc  : INPUT, +5VDC Power Supply to Microcontroller
                    // Gnd  : INPUT, Gound Power Supply to Microcontroller

// Drive Direction
uint8_t button_FWD = 3; //Drive Forward button
uint8_t button_REV = 4; //Drive Reverse button
uint8_t Drive = 0;      // Set drive flag 0=Stopped, 1=FWD, 2=REV

// Speed Adjustment
uint8_t potAdjust = A0; //Poteniometer Speed Adjustment, Analong In
uint8_t Adj_Spd;       //Analog read to trapped scaled Speed
double rev_factor = 3.0; // Reverse Speed Reduction Factor
uint8_t Adj_Spd_limit = 152; //Upper scalling limit from ADC to Adj_Spd
uint8_t Adj_Spd_bottom = 10; // Lower drive floor limit
uint32_t previousMillis = 0; //Analog read timeer
const uint16_t interval = 75; //Analog read interval
double ramp = 8.5; //Ramp interval     
double ramp_output = 0; // Output speed to PWMs

// Diagnostic Run Serial Print
// Note: Serial Print may affect loop time derived ramp.
const bool diag = false;

void setup() {

    pinMode(R_EN ,OUTPUT);  // R_EN : Forward Drive Enable, Active High, Low Disable
    pinMode(R_PWM,OUTPUT);  // RPWM : Forward PWM, Active High
    pinMode(R_IS ,INPUT);   // R_IS : Forward Drive Side Current Alarm

    pinMode(L_EN ,OUTPUT);  // L_EN : Reverse Drive Enable, Active High, Low Disable
    pinMode(L_PWM,OUTPUT);  // LPWM : Reverse PWM, Active High
    pinMode(L_IS ,INPUT);   // L_IS : Reverse Drive Side Current Alarm

    pinMode(button_FWD,INPUT_PULLUP);
    pinMode(button_REV,INPUT_PULLUP);

    //Disable Motors
    analogWrite(R_PWM,0);
    analogWrite(L_PWM,0);
    digitalWrite(R_EN,LOW);
    digitalWrite(L_EN,LOW);

    pinMode(potAdjust, INPUT);
    pinMode(A1,INPUT);

    if (diag == true) Serial.begin(250000);

}

void SerialPrint()
{
    Serial.print(Adj_Spd);
    Serial.print("\t");
    Serial.print(Drive);
    Serial.print("\t");
    Serial.print(ramp);
    Serial.print("\t");
    Serial.println(ramp_output);
}

void loop() {

    // Analog Read BWD Interval
    uint32_t currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
        Adj_Spd = map(analogRead(potAdjust), 0,1023,0,Adj_Spd_limit); //Speed 0-254 trapped by limit

    // Drive Buttons
    Drive = 0; //Set flag to stop
    if (digitalRead(button_REV) == LOW && digitalRead(button_FWD) == HIGH) {
        Drive = 2; // REV If only Rev is being pressed
        if (ramp_output > -Adj_Spd/rev_factor) ramp_output -= ramp / 2.0;
        if (ramp_output < -Adj_Spd/rev_factor) ramp_output += ramp;
    } 
    else if (digitalRead(button_FWD) == LOW && digitalRead(button_REV) == HIGH) {
        Drive = 1; // FWD takes prioity while only FWD is being pressed
        if (ramp_output < Adj_Spd) ramp_output += ramp / 2.0;
        if (ramp_output > Adj_Spd) ramp_output -= ramp;

    }
    else {
        if (ramp_output > 0) ramp_output -= ramp;
        if (ramp_output < 0) ramp_output += ramp;
    }

    } // Ramp captured within interval to control duration
    
    // If output greater than bottom speed = Forward
    if (ramp_output > Adj_Spd_bottom ) {
        digitalWrite(R_EN,HIGH);
        analogWrite(R_PWM,ramp_output); //FWD PWM
        digitalWrite(L_EN,HIGH);
        analogWrite(L_PWM,0);   // Set Rev PWM to 0
    }
    // Not greater, but less than negative bottom speed = Reverse
    else if (ramp_output < -Adj_Spd_bottom) {
        digitalWrite(R_EN,HIGH);
        analogWrite(R_PWM,0);   //Set FWD PWM to 0
        digitalWrite(L_EN,HIGH);
        analogWrite(L_PWM,-ramp_output); //Reverse PMW (Note, output is negative but PWM signal absolut)
    }
    // Neither, in bottom +/- range = Stopped. Allows natural ramp down to stop then de-enable.
    else {
        digitalWrite(R_EN,LOW);
        analogWrite(R_PWM,0);
        digitalWrite(L_EN,LOW);
        analogWrite(L_PWM,0);
    }

    if (diag == true) SerialPrint();
}