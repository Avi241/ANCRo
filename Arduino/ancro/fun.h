
#include <analogWrite.h>
#include "pins.h"

void pinSetup()
{
  pinMode(A1,OUTPUT);
  pinMode(B1,OUTPUT);
  pinMode(P1,OUTPUT);
  pinMode(A2,OUTPUT);
  pinMode(B2,OUTPUT);
  pinMode(P2,OUTPUT);
}

void mr_clk(int pwm)
{
  digitalWrite(A1,0);
  digitalWrite(B1,1);
  analogWrite(P1,pwm);
  
}

void mr_aclk(int pwm)
{
  digitalWrite(A1,1);
  digitalWrite(B1,0);
  analogWrite(P1,pwm);
  
}

void ml_clk(int pwm)
{
  digitalWrite(A2,0);
  digitalWrite(B2,1);
  analogWrite(P2,pwm);
  
}

void ml_aclk(int pwm)
{
  digitalWrite(A2,1);
  digitalWrite(B2,0);
  analogWrite(P2,pwm);
  
}





