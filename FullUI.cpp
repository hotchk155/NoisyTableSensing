

/*

 2.2  2.1       0.6  1.6  0.5     0.2  1.2  0.1  
 GR   GR        GR   YL   GR      GR   YL   GR
 
 2.0  2.7       1.3  2.6  1.5     1.7  2.5  1.1
 GR   GR        YL   RD   YL      YL   RD   YL
 
 2.3  2.4       0.3  1.4  0.4     0.7  1.0  0.0
 RD   RD        GR   YL   GR      GR   YL   GR
 
 */
#include "Arduino.h"
#include "NoisyTableSensing.h"

#if 0

#define P_HEARTBEAT 13
#define P_DIN 4
#define P_CLK 5
#define P_LOAD 6

#define LEDS2_GR1 (1<<2)
#define LEDS2_GR2 (1<<1)
#define LEDS2_GR3 (1<<0)
#define LEDS2_GR4 (1<<7)
#define LEDS2_ER1 (1<<3)
#define LEDS2_ER2 (1<<4)
#define LEDS2_MR1 (1<<6)
#define LEDS2_MR2 (1<<5)
#define LEDS0_A   (1<<3)
#define LEDS0_B   (1<<6)
#define LEDS0_C   (1<<5)
#define LEDS0_D   (1<<4)
#define LEDS0_E   (1<<1)
#define LEDS0_F   (1<<0)
#define LEDS0_G   (1<<7)
#define LEDS0_H   (1<<2)
#define LEDS1_1T  (1<<3)
#define LEDS1_1R  (1<<6)
#define LEDS1_1B  (1<<5)
#define LEDS1_1L  (1<<4)
#define LEDS1_2B  (1<<7)
#define LEDS1_2L  (1<<2)
#define LEDS1_2T  (1<<1)
#define LEDS1_2R  (1<<0)

#define LEDS0_ALLA (LEDS0_A|LEDS0_B|LEDS0_C|LEDS0_D)
#define LEDS1_ALLA (LEDS1_1T|LEDS1_1R|LEDS1_1L|LEDS1_1B)
#define LEDS2_ALLA (LEDS2_ER1|LEDS2_MR1)

#define LEDS0_ALLB (LEDS0_E|LEDS0_F|LEDS0_G|LEDS0_H)
#define LEDS1_ALLB (LEDS1_2T|LEDS1_2R|LEDS1_2L|LEDS1_2B)
#define LEDS2_ALLB (LEDS2_ER2|LEDS2_MR2)

byte UIDisplay0[8] = {0};
byte UIDisplay1[8] = {0};
byte UIDisplay2[8] = {0};

unsigned long timeToClear1 = 0;
unsigned long timeToClear2 = 0;
unsigned long timeForHeartbeat = 0;
byte heartBeat = 0;

#define UI_TIME_ERROR 500
#define UI_TIME_MISREAD 500
#define UI_TIME_READING 200
#define UI_HEARTBEAT_PERIOD 500

#define MAX7219_DECODE_MODE 	0x09
#define MAX7219_INTENSITY 		0x0A
#define MAX7219_SCAN_LIMIT 		0x0B
#define MAX7219_SHUTDOWN 		0x0C
#define MAX7219_DISPLAY_TEST	0x0F


void UISendData(byte addr, byte data)
{
  unsigned long d = (addr << 8) | data;
  unsigned long m = 0x8000;
  while(m)
  {
    digitalWrite(P_CLK,LOW);
    digitalWrite(P_DIN,(!!(d&m))? HIGH:LOW);
    digitalWrite(P_CLK,HIGH);
    m>>=1;
  }
}

void UIControl(byte addr, byte data0, byte data1, byte data2)
{
  digitalWrite(P_LOAD,LOW);
  UISendData(addr,data2);
  UISendData(addr,data1);
  UISendData(addr,data0);
  digitalWrite(P_LOAD,HIGH);
}

void UIRefresh()
{
  for(int i=0; i<8; ++i)
  {
    digitalWrite(P_LOAD,LOW);
    UISendData(i+1, UIDisplay2[i]);
    UISendData(i+1, UIDisplay1[i]);
    UISendData(i+1, UIDisplay0[i]);
    digitalWrite(P_LOAD,HIGH);
  }
}

void UISetup() {

  memset(UIDisplay0, 0, sizeof(UIDisplay0));
  memset(UIDisplay1, 0, sizeof(UIDisplay1));
  memset(UIDisplay2, 0, sizeof(UIDisplay2));

  pinMode(P_HEARTBEAT,OUTPUT);
  pinMode(P_DIN,OUTPUT);
  pinMode(P_CLK,OUTPUT);
  pinMode(P_LOAD,OUTPUT);
  delay(100);

  UIControl(MAX7219_SHUTDOWN, 1, 1, 1);  
  UIControl(MAX7219_DISPLAY_TEST, 0, 0, 0);
  UIControl(MAX7219_DECODE_MODE, 0, 0, 0);
  UIControl(MAX7219_SCAN_LIMIT, 7, 7, 2);
  UIControl(MAX7219_INTENSITY, 0x07, 0x07, 0x07);
  UIRefresh();
}


void UIReport(byte which, unsigned int result, byte row, byte col, unsigned long milliseconds)
{
  byte *grid = which? UIDisplay1 : UIDisplay0;

  if(!which)
  {
    UIDisplay2[0] &= ~LEDS0_ALLA;
    UIDisplay2[1] &= ~LEDS1_ALLA;
    UIDisplay2[2] &= ~LEDS2_ALLA;
    if(!!(result & RESULT_MISREAD))
    {
      if(!!(result & RESULT_SENSORA)) UIDisplay2[0] |= LEDS0_A;
      if(!!(result & RESULT_SENSORB)) UIDisplay2[0] |= LEDS0_B;
      if(!!(result & RESULT_SENSORC)) UIDisplay2[0] |= LEDS0_C;
      if(!!(result & RESULT_SENSORD)) UIDisplay2[0] |= LEDS0_D;
      UIDisplay2[2] |= LEDS2_MR1;
      timeToClear1 = milliseconds + UI_TIME_MISREAD;
    }
    else if(!!(result & RESULT_ERROR))
    {
      UIDisplay2[2] |= LEDS2_ER1;
      timeToClear1 = milliseconds + UI_TIME_ERROR;
    } 
    else
    {
      UIDisplay2[0] |= LEDS0_A|LEDS0_B|LEDS0_C|LEDS0_D;
      if(!!(result & RESULT_XMIN)) UIDisplay2[1] |= LEDS1_1L;
      if(!!(result & RESULT_XMAX)) UIDisplay2[1] |= LEDS1_1R;
      if(!!(result & RESULT_YMIN)) UIDisplay2[1] |= LEDS1_1T;
      if(!!(result & RESULT_YMAX)) UIDisplay2[1] |= LEDS1_1B;
      timeToClear1 = milliseconds + UI_TIME_READING;
    }  
  }
  else
  {
    UIDisplay2[0] &= ~LEDS0_ALLB;
    UIDisplay2[1] &= ~LEDS1_ALLB;
    UIDisplay2[2] &= ~LEDS2_ALLB;
    if(!!(result & RESULT_MISREAD))
    {
      if(!!(result & RESULT_SENSORA)) UIDisplay2[0] |= LEDS0_E;
      if(!!(result & RESULT_SENSORB)) UIDisplay2[0] |= LEDS0_F;
      if(!!(result & RESULT_SENSORC)) UIDisplay2[0] |= LEDS0_G;
      if(!!(result & RESULT_SENSORD)) UIDisplay2[0] |= LEDS0_H;
      UIDisplay2[2] |= LEDS2_MR2;
      timeToClear2 = milliseconds + UI_TIME_MISREAD;
    }
    else if(!!(result & RESULT_ERROR))
    {
      UIDisplay2[2] |= LEDS2_ER2;
      timeToClear2 = milliseconds + UI_TIME_ERROR;
    } 
    else
    {
      UIDisplay2[0] |= LEDS0_E|LEDS0_F|LEDS0_G|LEDS0_H;
      if(!!(result & RESULT_XMIN)) UIDisplay2[1] |= LEDS1_2L;
      if(!!(result & RESULT_XMAX)) UIDisplay2[1] |= LEDS1_2R;
      if(!!(result & RESULT_YMIN)) UIDisplay2[1] |= LEDS1_2T;
      if(!!(result & RESULT_YMAX)) UIDisplay2[1] |= LEDS1_2B;
      timeToClear2 = milliseconds + UI_TIME_READING;
    }  
  }

  if(!!(result & RESULT_ERROR))
  {
    memset(grid,0,8);
  }
  else if(!!(result & RESULT_MISREAD))
  {
    memset(grid,0,8);
  }
  else
  {  
    byte p;
    byte q;
    if(!which)
    {
      p = 7-row;
      q = 7-col; 
    }
    else
    {
      p = row;
      q = col; 
    }
    memset(grid,0,8);
    switch(p)
    {
    case 0: grid[q] = 128; break;
    case 1: grid[q] = 1; break;
    case 2: grid[q] = 2; break;
    case 3: grid[q] = 4; break;
    case 4: grid[q] = 8; break;
    case 5: grid[q] = 16; break;
    case 6: grid[q] = 32; break;
    case 7: grid[q] = 64; break;
    }
  }  
  UIRefresh();
}


void UIRun(unsigned long milliseconds) 
{
  if(timeToClear1 > 0 && milliseconds > timeToClear1)
  {
    UIDisplay2[0] &= ~LEDS0_ALLA;
    UIDisplay2[1] &= ~LEDS1_ALLA;
    UIDisplay2[2] &= ~LEDS2_ALLA;
    UIRefresh();
    timeToClear1 = 0;
  }
  if(timeToClear2 > 0 && milliseconds > timeToClear2)
  {
    UIDisplay2[0] &= ~LEDS0_ALLB;
    UIDisplay2[1] &= ~LEDS1_ALLB;
    UIDisplay2[2] &= ~LEDS2_ALLB;
    UIRefresh();
    timeToClear2 = 0;
  }
  if(milliseconds > timeForHeartbeat)
  {
    timeForHeartbeat = milliseconds + UI_HEARTBEAT_PERIOD;
    heartBeat = !heartBeat;
    digitalWrite(P_HEARTBEAT, heartBeat);
  }
}

#endif
