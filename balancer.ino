#include <PID_v1.h>

//*** INCLUDES ***
#include "MeAuriga.h"
#include <Wire.h>
#include "math.h"

//*** VARIABLES ***
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeGyro gyroSensor(1,0x69);
#ifdef MeAuriga_H
// on-board LED ring, at PORT0 (onboard), with 12 LEDs
MeRGBLed led_ring( 0, 12 );
#endif


//'''''' PREFERRED POSITION '''''
double STAND_X = 70;
double STAND_Y = 1.2;
double STAND_Z = 4.5;
double CURRENT_X;
double NEW_PWD;
double Kp=50, Ki=0, Kd=0;

// GOOD: 40/0/0.06 and 180 PWD oder 24/0/0
// Workable: 50 / 0 / 0 / 80 PWD

PID myPIDD(&CURRENT_X, &NEW_PWD, &STAND_X, Kp, Ki, Kd, DIRECT);
PID myPIDR(&CURRENT_X, &NEW_PWD, &STAND_X, Kp, Ki, Kd, REVERSE);


void setup() {
  Serial.begin(115200); // Set serial connection at 9600 bits per second

  
 #ifdef MeAuriga_H
    // 12 LED Ring controller is on Auriga D44/PWM
    led_ring.setpin( 44 );
 #endif

 //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(0.18,0,0);
  Encoder_2.setPosPid(0.18,0,0);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
  gyroSensor.begin(); 

  // Init variables
  CURRENT_X = STAND_X;
  myPIDD.SetMode(AUTOMATIC);
  myPIDR.SetMode(AUTOMATIC);

  
}




void loop() {
    gyroSensor.update();

    Serial.read();
    CURRENT_X = gyroSensor.getAngleX();
    Serial.print(" CURRENT X: ");
    Serial.print(CURRENT_X);
    Serial.print(" SET PWD: ");
    Serial.print(NEW_PWD);
  
    if (CURRENT_X > STAND_X){
    myPIDR.Compute();
    Serial.println("Front");
    if (NEW_PWD > 80){
     NEW_PWD = 80;
    }
    if (CURRENT_X < 30){
     NEW_PWD = 0;
    }
    Encoder_1.setMotorPwm(-NEW_PWD);    
    Encoder_2.setMotorPwm(NEW_PWD);

    
    }
    if (CURRENT_X < STAND_X){
    myPIDD.Compute();
    // Back lean
    Serial.println("Back");
    if (NEW_PWD > 80){
      NEW_PWD = 80;
    }
    if (CURRENT_X < 30){
     NEW_PWD = 0;
    }
    Encoder_1.setMotorPwm(NEW_PWD);    
    Encoder_2.setMotorPwm(-NEW_PWD);
    }
        
}

