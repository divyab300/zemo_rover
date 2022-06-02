/*
    Developed by: Divya Prakash Biswas on 31st May,2022
*/

#include "Arduino.h"
#include "motor.h"

motor::motor(int inA, int inB, int pwm_pin, int encA, int encB)
  {
    INA = inA;
    INB = inB;
    PWM_pin = pwm_pin;
    ENCA = encA;
    ENCB = encB;
  }

  void motor::init_pins()
  {
    pinMode(INA,OUTPUT);
    pinMode(INB,OUTPUT);
    pinMode(PWM_pin,OUTPUT);
    pinMode(ENCA,INPUT);
    pinMode(ENCB,INPUT);
  }

  int motor::get_encA() {return ENCA;}
  int motor::get_encB() {return ENCB;}

  void motor::set_pwm(int pwm_val)
  {
    pwm_val = constrain(pwm_val,-255,255);
    if (pwm_val>=0)
    {
      digitalWrite(INA,HIGH);
      digitalWrite(INB,LOW);
      analogWrite(PWM_pin,pwm_val);
    }
    else
    {
      digitalWrite(INA,LOW);
      digitalWrite(INB,HIGH);
      analogWrite(PWM_pin,abs(pwm_val));
    }
  }