#include "Arduino.h"
#include "NoisyTableSensing.h"

#define P_HEARTBEAT 13
#define P_SHCLK 4
#define P_STCLK 3
#define P_DAT   2

#define LEDS_A 0x2000
#define LEDS_B 0x0200
#define LEDS_C 0x0400
#define LEDS_D 0x1000
#define LEDS_MISREAD1 0x0800
#define LEDS_MASK1 0xff00

#define LEDS_E 0x0020
#define LEDS_F 0x0002
#define LEDS_G 0x0004
#define LEDS_H 0x0010
#define LEDS_MISREAD2 0x0008
#define LEDS_MASK2 0x00ff

#define UI_TIME_ERROR 500
#define UI_TIME_MISREAD 500
#define UI_TIME_READING 200
#define UI_HEARTBEAT_PERIOD 500

unsigned long leds = 0;
unsigned long timeToClear1 = 0;
unsigned long timeToClear2 = 0;

byte heartBeatState = 0;
unsigned long nextHeartBeat = 0;

void UISetup() {
  pinMode(P_SHCLK, OUTPUT);
  pinMode(P_STCLK, OUTPUT);
  pinMode(P_DAT, OUTPUT);
  pinMode(P_HEARTBEAT, OUTPUT);
  heartBeatState = 0;
}

void UIRefresh() 
{
    unsigned long mask = 0x8000L;
    digitalWrite(P_STCLK, LOW);
    while(mask > 0)
    {
      digitalWrite(P_SHCLK, LOW);
      digitalWrite(P_DAT, !!(leds & mask));
      digitalWrite(P_SHCLK, HIGH);
      mask >>= 1;
    }
    digitalWrite(P_STCLK, HIGH);
}

void UIReport(byte which, unsigned int result, byte row, byte col, unsigned long milliseconds) {


  if(!which) // 0
  {
    leds &= ~LEDS_MASK1;
    if(!!(result & RESULT_MISREAD))
    {
      leds |= LEDS_MISREAD1;
      if(!!(result & RESULT_SENSORA)) leds |= LEDS_A;
      if(!!(result & RESULT_SENSORB)) leds |= LEDS_B;
      if(!!(result & RESULT_SENSORC)) leds |= LEDS_C;
      if(!!(result & RESULT_SENSORD)) leds |= LEDS_D;
      timeToClear1 = milliseconds + UI_TIME_MISREAD;
    }
    else if(!!(result & RESULT_ERROR))
    {
      leds |= LEDS_MISREAD1;
      timeToClear1 = milliseconds + UI_TIME_ERROR;
    } 
    else
    {
      leds |= LEDS_MASK1;
      timeToClear1 = milliseconds + UI_TIME_READING;
    }  
  }
  else
  {
    leds &= ~LEDS_MASK2;
    if(!!(result & RESULT_MISREAD))
    {
      leds |= LEDS_MISREAD2;
      if(!!(result & RESULT_SENSORA)) leds |= LEDS_E;
      if(!!(result & RESULT_SENSORB)) leds |= LEDS_F;
      if(!!(result & RESULT_SENSORC)) leds |= LEDS_G;
      if(!!(result & RESULT_SENSORD)) leds |= LEDS_H;
      timeToClear2 = milliseconds + UI_TIME_MISREAD;
    }
    else if(!!(result & RESULT_ERROR))
    {
      leds |= LEDS_MISREAD2;
      timeToClear2 = milliseconds + UI_TIME_ERROR;
    } 
    else
    {
      leds |= LEDS_MASK2;
      timeToClear2 = milliseconds + UI_TIME_READING;
    }  
  }
  UIRefresh();  
}
void UIRun(unsigned long milliseconds) {
  if(milliseconds > nextHeartBeat)
  {
    digitalWrite(P_HEARTBEAT, heartBeatState);
    heartBeatState = !heartBeatState;
    nextHeartBeat = milliseconds + UI_HEARTBEAT_PERIOD;
  }
  if(timeToClear1 && milliseconds > timeToClear1)
  {
    leds &= ~LEDS_MASK1;
    timeToClear1 = 0;
    UIRefresh();  
  }
  if(timeToClear2 && milliseconds > timeToClear2)
  {
    leds &= ~LEDS_MASK2;
    timeToClear2 = 0;
    UIRefresh();  
  }
}
