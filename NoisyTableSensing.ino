//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
////
//// NOISY TABLE SENSING MODULE
////
//// J.Hotchkiss 2012
////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

#include "NoisyTableSensing.h"
#include "HardwareSetup2.h"

//////////////////////////////////////////////////////////////
//
// CONSTANTS
//
//////////////////////////////////////////////////////////////

#define P_SWITCHP  6
#define P_SWITCHQ  5

// Timing: Prescalers
#define TIMER_PRESCALER1 0x04 // 1/256 of Fosc
#define TIMER_PRESCALER2 0x06  // 1/256 of Fosc

// Timing: How long the LEDs stay lit after a good read (ms)
#define READ_LED_MS  200

// Timing: How long the LEDs stay lit after a misread (ms)
#define MISREAD_LED_MS  1000

// Timing: How long to ignore input after a read (ms)
#define IGNORE_MS 300

// Timing: Debounce time on buttons
#define BUTTON_DEBOUNCE_MS 100

#define TABLE_WIDTH_1      (int)(WIDTH1*SCALE1)
#define TABLE_HEIGHT_1     (int)(HEIGHT1*SCALE1)
#define TABLE_WIDTH_2      (int)(WIDTH2*SCALE2)
#define TABLE_HEIGHT_2     (int)(HEIGHT2*SCALE2)

// timing states
enum {
  TIMING_IGNORE,    // ignore all input during the "ignore" phase
  TIMING_START,     // initial state, get set up
  TIMING_LISTEN,    // waiting for the first sensor to fire
  TIMING_RUN        // timing for the remaining sensors to fire
};

//////////////////////////////////////////////////////////////
//
// VARIABLES
//
//////////////////////////////////////////////////////////////

// debug mode
byte debugOutput = 0;

// variables for player 1
byte state1;
int timeOfArrival1A;
int timeOfArrival1B;
int timeOfArrival1C;
int timeOfArrival1D;
unsigned long timeout1;
unsigned long clearLeds1;
int TABLE_DIAGONAL_1 = 0;

// variables for player 2
byte state2;
int timeOfArrival2A;
int timeOfArrival2B;
int timeOfArrival2C;
int timeOfArrival2D;
unsigned long timeout2;
unsigned long clearLeds2;
int TABLE_DIAGONAL_2 = 0;

// Working buffers for circle intersect data
int intersectCount = 0;
int intersectX[12];
int intersectY[12];

unsigned long buttonPdebounce = 0;
unsigned long buttonQdebounce = 0;
byte buttonPstate = 0;
byte buttonQstate = 0;

//////////////////////////////////////////////////////
// MIDI CHANNEL CLASS
// We map to an 8x8 grid where the MIDI note number
// is equal to 16 * row + col where row and col range
// from 0..7. This is consistent with the Novation
// Launchpad note layout
#define NO_NOTE 255
#define MIDI_NOTE_MISREAD 127
#define MIDI_NOTE_BUTTON 126
class CMidiChannel
{
  byte m_chan;
  byte m_lastNote;
public:  
  CMidiChannel(byte chan)
  {
    m_chan = chan;
    m_lastNote = NO_NOTE;
  }
  void midiNote(byte note, byte vel)
  {
    Serial.write(0x90 | m_chan);
    Serial.write(note);
    Serial.write(vel);
  }
  void reset()
  {
    for(int note=0; note<128; ++note)
      midiNote(note,0);
  }
  void reading(byte row, byte col)
  {
    byte note = col + 16 * row;
    if(m_lastNote != NO_NOTE)
      midiNote(m_lastNote, 0);
    m_lastNote = note;
    midiNote(m_lastNote, 127);
  }  
  void misread()
  {
    if(m_lastNote != NO_NOTE)
      midiNote(m_lastNote, 0);
    m_lastNote = MIDI_NOTE_MISREAD;
    midiNote(m_lastNote, 127);
  }  
  void button()
  {
    if(m_lastNote != NO_NOTE)
      midiNote(m_lastNote, 0);
    m_lastNote = MIDI_NOTE_BUTTON;
    midiNote(m_lastNote, 127);
  }  
};

CMidiChannel MidiChannel1(0);
CMidiChannel MidiChannel2(1);

///////////////////////////////////////////////////////////////////////////////////////
// CALC CIRCLE INTERSECTIONS
// Utility function to find the coordinates the two points where the circumferences
// of a pair of circles intersect with each other. Store the results in the intersectX
// and intersectY arrays
// This approach from  http://paulbourke.net/geometry/2circle/
void CalcCircleIntersections(float x0, float y0, float r0, float x1, float y1, float r1)
{
  float a, dx, dy, d, h, rx, ry;
  float x2, y2;

  // get the offset of circle centres
  dx = x1 - x0;
  dy = y1 - y0;

  // and the distance between the,
  d = hypot(dx,dy); 

  // ensure the circles intersect
  if (d > (r0 + r1))
    return ;

  // ensure the circles cross
  if (d < fabs(r0 - r1))
    return;

  // 'point 2' is the point where the line through the circle
  // intersection points crosses the line between the circle
  // centers.  

  // Determine the distance from point 0 to point 2. 
  a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

  // Determine the coordinates of point 2.
  x2 = x0 + (dx * a/d);
  y2 = y0 + (dy * a/d);

  // Determine the distance from point 2 to either of the
  // intersection points.
  h = sqrt((r0*r0) - (a*a));

  // Now determine the offsets of the intersection points from
  // point 2.
  rx = -dy * (h/d);
  ry = dx * (h/d);

  // Calclulate the intersection points and store the 
  // coordinates
  intersectX[intersectCount] = x2 + rx;
  intersectY[intersectCount++] = y2 + ry;
  intersectX[intersectCount] = x2 - rx;
  intersectY[intersectCount++] = y2 - ry;
}

///////////////////////////////////////////////////////////////////////////////////////
// CALCULATE POINT FROM TIME OF ARRIVAL AT 4 SENSORS
// The labelling of the sensors is as follows
// A---B
// |   |
// D---C
unsigned int CalculatePointFromTOA(int tA, int tB, int tC, int tD, int tableWidth, int tableHeight, int tableDiagonal, int *pX, int *pY )
{  
  // get the minimum value and subtract from all values (typically
  // we expect the minimum value will be zero, but we make sure that
  // this is the case)
  int tmin = min(tA,tB);
  tmin = min(tmin,tC);
  tmin = min(tmin,tD);
  tA -= tmin;
  tB -= tmin;
  tC -= tmin;
  tD -= tmin;

  // We can visualise tA, tB, tC and tD as the radii of four circles 
  // centered on points A,B,C,D respectively. The smallest circle has 
  // a radius of zero. If we increase the radii of all four circles by 
  // the same amount, L, then there will be a value of L at which the four 
  // circles intersect at the same point. This is the point we need to 
  // calculate. We do this by a fast approximation. 

  // The idea is that each pair of circles AB, AC, AD, BC, BD, CD has its 
  // own value of L where those 2 circles just meet. If we work out L for 
  // each of the 6 pairs and take the maximum value we find, then we find 
  // a value of L where all 4 cirles intersect for the first time. It turns
  // out this is a reasonable (and very quick) approximation for the true value
  // of L

  // The value for L for each pair of circles is easy to calculate (work out 
  // the distance between the centre points, subtract the 2 radiuses, then 
  // divide the remainder by two)

  // we'll divide by 2 later...
  int L_AB = tableWidth - tA - tB;
  int L_BC = tableHeight - tB - tC;
  int L_DC = tableWidth - tD - tC;
  int L_AD = tableHeight - tA - tD;
  int L_AC = tableDiagonal  - tA - tC;
  int L_DB = tableDiagonal  - tD - tB;

  // which line is longest?
  int L_Max = max(L_AB, L_BC);
  L_Max = max(L_Max, L_DC);
  L_Max = max(L_Max, L_AD);
  L_Max = max(L_Max, L_AC);
  L_Max = max(L_Max, L_DB);

  // all important division by 2 only needs to be done once
  L_Max = L_Max / 2;

  // Increase the radii of each cicrle so we get to the
  // "first intersection point"
  tA = tA + L_Max + 1;
  tB = tB + L_Max + 1;
  tC = tC + L_Max + 1;
  tD = tD + L_Max + 1;

  // The cicles should all touch now, but we won't yet have
  // a single point of intersection. Here a fast approximation 
  // method is to find out where each pair of circles now
  // intersects and average them all out.
  //
  // Each pair of circles has 2 intersections and we only 
  // want to take the point that is closest to the other 
  // intersection points of other circles. We do this by
  // getting a mean of all the coordinates, rejecting the 
  // point of each pair that is furthest from the mean
  // point, then recalculating the mean of the remaining 
  // points. The result is a good approximation of the 
  // intersection point that would be derived if the final
  // true value of L was known (i.e. the point of the even
  // triggering our sensors)

  // Store the intersection points (2 per pair) for all 
  // six pairs of circles
  intersectCount = 0;	
  CalcCircleIntersections(0,0,tA,tableWidth,0,tB); // A and B
  CalcCircleIntersections(0,0,tA,tableWidth,tableHeight,tC); // A and C
  CalcCircleIntersections(0,0,tA,0,tableHeight,tD); // A and D
  CalcCircleIntersections(tableWidth,0,tB,tableWidth,tableHeight,tC); // B and C
  CalcCircleIntersections(tableWidth,0,tB,0,tableHeight,tD); // B and D
  CalcCircleIntersections(tableWidth,tableHeight,tC,0,tableHeight,tD); // C and D

    // make sure there are intersections before we continue
  if(!intersectCount)
    return RESULT_ERROR;

  int i;
  int iAvgX1 = 0;
  int iAvgY1 = 0;
  int iAvgX2 = 0;
  int iAvgY2 = 0;

  // average out the X and Y positions of all the 
  // intersections of all the pairs of circles
  for(i=0; i < intersectCount; ++i)
  {
    iAvgX1 += intersectX[i];
    iAvgY1 += intersectY[i];
  }
  iAvgX1 /= intersectCount;
  iAvgY1 /= intersectCount;

  // Loop through all the pairs of intersection 
  // points again
  for(i=0; i < intersectCount; i+=2)
  {
    // check which point of the pair is closest
    // to the average intersection point and 
    // include that point in the refined average
    unsigned long dx0 = intersectX[i] - iAvgX1;
    unsigned long dy0 = intersectY[i] - iAvgY1;
    unsigned long dx1 = intersectX[i+1] - iAvgX1;
    unsigned long dy1 = intersectY[i+1] - iAvgY1;
    if(dx0*dx0 + dy0*dy0 < dx1*dx1 + dy1*dy1)
    {
      iAvgX2 += intersectX[i];
      iAvgY2 += intersectY[i];
    }
    else
    {
      iAvgX2 += intersectX[i+1];
      iAvgY2 += intersectY[i+1];
    }
  }

  // Calculation of average
  *pX = iAvgX2 / (intersectCount/2);
  *pY = iAvgY2 / (intersectCount/2);

  unsigned int result = RESULT_READ;
  // disregard negatitve values
  if(*pX < 0) 
  {
    *pX = 0;
    result |= RESULT_XMIN;
  }
  if(*pX >= tableWidth) 
  {
    *pX = tableWidth-1;
    result |= RESULT_XMAX;
  }
  if(*pY < 0) 
  {
    *pY = 0;   
    result |= RESULT_YMIN;
  }
  if(*pY >= tableHeight) 
  {
    *pY = tableHeight-1;
    result |= RESULT_YMAX;
  }
  return result;
}


//////////////////////////////////////////////////////
// DUMP GRID SHOWING CALC LOCATION
// diagnostic function
#define DUMP_GRID_WIDTH   8
#define DUMP_GRID_HEIGHT  8
void dump(int x, int y, int width, int height)
{
  int cx = (DUMP_GRID_WIDTH*x)/width;
  int cy = (DUMP_GRID_HEIGHT*y)/height;
  for(int i = 0; i < DUMP_GRID_HEIGHT; ++i)
  {
    for(int j = 0; j < DUMP_GRID_WIDTH; ++j)
    {
      if(i == cy && j == cx)
        Serial.print("@");
      else
        Serial.print(".");
    }
    Serial.println();
  }
}

//////////////////////////////////////////////////////
// READ HANDLER FOR PLAYER 1
// invoked when there is a good read on first side 
// of the table
void onRead1(unsigned long milliseconds)
{
  int calcX = 0;  
  int calcY = 0;  
  unsigned int result = CalculatePointFromTOA(timeOfArrival1A, timeOfArrival1B, timeOfArrival1C, timeOfArrival1D, TABLE_WIDTH_1, TABLE_HEIGHT_1, TABLE_DIAGONAL_1, &calcX, &calcY);
  if(debugOutput)
  {
    Serial.println("***  PLAYER 1 READING ***");
    Serial.print(timeOfArrival1A);
    Serial.print(", ");
    Serial.print(timeOfArrival1B);
    Serial.print(", ");
    Serial.print(timeOfArrival1C);
    Serial.print(", ");
    Serial.print(timeOfArrival1D);
    Serial.print(" -> (");
    Serial.print(calcX);
    Serial.print(", ");
    Serial.print(calcY);
    Serial.print(")");
    Serial.println();
    if(!!(result & RESULT_ERROR))
    {
      Serial.println("Calculation error!");
    }
    else
    {
      dump(calcX, calcY, TABLE_WIDTH_1, TABLE_HEIGHT_1);
    }
  }
  else
  {
    if(!!(result & RESULT_ERROR))
    {
      MidiChannel1.reading(0, 0);
      UIReport(UI_BOTTOM, result, 0, 0, milliseconds);
    }
    else
    {
      byte row = 8 * ((float)calcY/TABLE_HEIGHT_1);
      byte col = 8 * ((float)calcX/TABLE_WIDTH_1);
      if(row > 7) row=7;
      if(col > 7) col=7;
      MidiChannel1.reading(row, col);
      UIReport(UI_BOTTOM, result, row, col, milliseconds);
    }
  }
}

//////////////////////////////////////////////////////
// MISREAD HANDLER FOR PLAYER 1
// invoked when there is a misread on first side 
// of the table
void onMisread1(unsigned long milliseconds)
{
  if(debugOutput)
  {
    Serial.println("*** PLAYER 1 MISREAD ");
    Serial.print(timeOfArrival1A, DEC);
    Serial.print(" ");
    Serial.print(timeOfArrival1B, DEC);
    Serial.print(" ");
    Serial.print(timeOfArrival1C, DEC);
    Serial.print(" ");
    Serial.print(timeOfArrival1D, DEC);
    Serial.println();
  }
  else
  {
    MidiChannel1.misread();
    unsigned long result = 0;
    if(timeOfArrival1A >= 0) result |= RESULT_SENSORA;
    if(timeOfArrival1B >= 0) result |= RESULT_SENSORB;
    if(timeOfArrival1C >= 0) result |= RESULT_SENSORC;
    if(timeOfArrival1D >= 0) result |= RESULT_SENSORD;
    UIReport(UI_BOTTOM, result|RESULT_MISREAD, 0, 0, milliseconds);    
  }  
}

//////////////////////////////////////////////////////
// READ HANDLER FOR PLAYER 2
// invoked when there is a good read on second side 
// of the table
void onRead2(unsigned long milliseconds)
{
  int calcX = 0;  
  int calcY = 0;  
  unsigned int result = CalculatePointFromTOA(timeOfArrival2A, timeOfArrival2B, timeOfArrival2C, timeOfArrival2D, TABLE_WIDTH_2, TABLE_HEIGHT_2, TABLE_DIAGONAL_2, &calcX, &calcY);
  if(debugOutput)
  {
    Serial.println("***  PLAYER 2 READING ***");
    Serial.print(timeOfArrival2A);
    Serial.print(", ");
    Serial.print(timeOfArrival2B);
    Serial.print(", ");
    Serial.print(timeOfArrival2C);
    Serial.print(", ");
    Serial.print(timeOfArrival2D);
    Serial.print(" -> (");
    Serial.print(calcX);
    Serial.print(", ");
    Serial.print(calcY);
    Serial.print(")");
    Serial.println();
    if(!!(result & RESULT_ERROR))
    {
      Serial.println("Calculation error!");
    }
    else
    {
      dump(calcX, calcY, TABLE_WIDTH_2, TABLE_HEIGHT_2);
    }
  }
  else
  {
    if(!!(result & RESULT_ERROR))
    {
      MidiChannel2.reading(0, 0);
      UIReport(UI_TOP, result, 0, 0, milliseconds);
    }
    else
    {
      unsigned long uiStatus = 0;
      byte row = 8 * ((float)calcY/TABLE_HEIGHT_2);
      byte col = 8 * ((float)calcX/TABLE_WIDTH_2);
      if(row > 7) row=7;
      if(col > 7) col=7;
      MidiChannel2.reading(row, col);
      UIReport(UI_TOP, result, row, col, milliseconds);
    }
  }
}

//////////////////////////////////////////////////////
void onButtonP()
{
  if(debugOutput)
  {
    Serial.println("*** BUTTON P");
  }
  else
  {
    MidiChannel2.button();
  }
}
//////////////////////////////////////////////////////
void onButtonQ()
{
  if(debugOutput)
  {
    Serial.println("*** BUTTON Q");
  }
  else 
  {
    MidiChannel1.button();
  }
}

//////////////////////////////////////////////////////
// MISREAD HANDLER FOR PLAYER 2
// invoked when there is a misread on second side 
// of the table
void onMisread2(unsigned long milliseconds)
{
  if(debugOutput)
  {
    Serial.println("*** PLAYER 2 MISREAD ");
    Serial.print(timeOfArrival2A, DEC);
    Serial.print(" ");
    Serial.print(timeOfArrival2B, DEC);
    Serial.print(" ");
    Serial.print(timeOfArrival2C, DEC);
    Serial.print(" ");
    Serial.print(timeOfArrival2D, DEC);
    Serial.println();
  }
  else
  {
    MidiChannel2.misread();
    unsigned long result = 0;
    if(timeOfArrival2A >= 0) result |= RESULT_SENSORA;
    if(timeOfArrival2B >= 0) result |= RESULT_SENSORB;
    if(timeOfArrival2C >= 0) result |= RESULT_SENSORC;
    if(timeOfArrival2D >= 0) result |= RESULT_SENSORD;
    UIReport(UI_TOP, result|RESULT_MISREAD, 0, 0, milliseconds);    
  }  
}

//////////////////////////////////////////////////////
// 
// PLAYER 1 INTERRUPT HANDLER
//
//////////////////////////////////////////////////////
ISR(PCINT1_vect) 
{
  TCCR1B = TIMER_PRESCALER1;
  if(!!(PCMSK1 & ~PINC & BITMASK_1A))
  {
    timeOfArrival1A = TCNT1;
    PCMSK1 &= ~BITMASK_1A;
  }
  if(!!(PCMSK1 & ~PINC & BITMASK_1B))
  {
    timeOfArrival1B = TCNT1;
    PCMSK1 &= ~BITMASK_1B;
  }
  if(!!(PCMSK1 & ~PINC & BITMASK_1C))
  {
    timeOfArrival1C = TCNT1;
    PCMSK1 &= ~BITMASK_1C;
  }
  if(!!(PCMSK1 & ~PINC & BITMASK_1D))
  {
    timeOfArrival1D = TCNT1;
    PCMSK1 &= ~BITMASK_1D;
  }
}

//////////////////////////////////////////////////////
// 
// PLAYER 2 INTERRUPT HANDLER
//
//////////////////////////////////////////////////////
ISR(PCINT0_vect) 
{
  TCCR2B = TIMER_PRESCALER2;
  if(!!(PCMSK0 & ~PINB & BITMASK_2A))
  {
    timeOfArrival2A = TCNT2;
    PCMSK0 &= ~BITMASK_2A;
  }
  if(!!(PCMSK0 & ~PINB & BITMASK_2B))
  {
    timeOfArrival2B = TCNT2;
    PCMSK0 &= ~BITMASK_2B;
  }
  if(!!(PCMSK0 & ~PINB & BITMASK_2C))
  {
    timeOfArrival2C = TCNT2;
    PCMSK0 &= ~BITMASK_2C;
  }
  if(!!(PCMSK0 & ~PINB & BITMASK_2D))
  {
    timeOfArrival2D = TCNT2;
    PCMSK0 &= ~BITMASK_2D;
  }
}

//////////////////////////////////////////////////////
//
// SETUP
//
//////////////////////////////////////////////////////
void setup()
{
  // calculate the diagonal dimension "contants"
  TABLE_DIAGONAL_1 = hypot(TABLE_WIDTH_1, TABLE_HEIGHT_1);
  TABLE_DIAGONAL_2 = hypot(TABLE_WIDTH_2, TABLE_HEIGHT_2);

  // setup the player 1 sensor inputs
  pinMode(P_1A,INPUT);
  pinMode(P_1B,INPUT);
  pinMode(P_1C,INPUT);
  pinMode(P_1D,INPUT);

  // setup the player 2 sensor inputs
  pinMode(P_2A,INPUT);
  pinMode(P_2B,INPUT);
  pinMode(P_2C,INPUT);
  pinMode(P_2D,INPUT);

  pinMode(P_SWITCHP,INPUT);  
  pinMode(P_SWITCHQ,INPUT);
  
  digitalWrite(P_SWITCHP,HIGH);  
  digitalWrite(P_SWITCHQ,HIGH);

  UISetup();

  PCMSK0 = 0;  
  PCMSK1 = 0;  
  PCICR |= (1<<PCIE0) | (1<<PCIE1);

  if(debugOutput)
  {
    Serial.begin(9600);
    Serial.println("Start...");
  }
  else
  {
    Serial.begin(31250);
    MidiChannel1.reset();
    MidiChannel2.reset();
  }
  interrupts();
  state1 = TIMING_START;   
  state2 = TIMING_START;   


}

//////////////////////////////////////////////////////
//
// LOOP
//
//////////////////////////////////////////////////////
void loop()
{
  // get count of milliseconds
  unsigned long milliseconds = millis();

  ///////////////////////////////////////////////
  //
  //
  // PLAYER ONE STATE MACHINE
  //
  // 
  ///////////////////////////////////////////////
  switch(state1)
  {    
    ///////////////////////////////////////////////
    // FOLLOWING A READ EVENT, IGNORE ALL INPUT FOR 
    // A PERIOD OF TIME
  case TIMING_IGNORE: 
    PCMSK1 = 0;  
    if(milliseconds < timeout1)
      break;
    // otherwise fall thru

    /////////////////////////////////////////
    // READY TO START LISTING FOR A NEW EVENT
  case TIMING_START: 
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = 0;  
    timeOfArrival1A = -1;
    timeOfArrival1B = -1;
    timeOfArrival1C = -1;
    timeOfArrival1D = -1;
    PCMSK1 = BITMASK_1ALL;
    state1 = TIMING_LISTEN;
    break;

    /////////////////////////////////////////
    // LISTENING FOR THE START OF A NEW EVENT 
  case TIMING_LISTEN:
    if((PCMSK1 & BITMASK_1ALL) != BITMASK_1ALL)
      state1 = TIMING_RUN;
    break;

    ////////////////////////////////////
    // LISTENING FOR COMPLETION OF EVENT 
  case TIMING_RUN:
    if(!(PCMSK1 & BITMASK_1ALL))
    {
      onRead1(milliseconds);        
      state1 = TIMING_IGNORE;
      timeout1 = milliseconds + IGNORE_MS;
    }
    else if(TCNT1 > TIMER_TIMEOUT1)
    {
      onMisread1(milliseconds);
      state1 = TIMING_IGNORE;
      timeout1 = milliseconds + IGNORE_MS;
    }
    break;      
  }

  ///////////////////////////////////////////////
  //
  //
  // PLAYER TWO STATE MACHINE
  //
  // 
  ///////////////////////////////////////////////
  switch(state2)
  {    
    ///////////////////////////////////////////////
    // FOLLOWING A READ EVENT, IGNORE ALL INPUT FOR 
    // A PERIOD OF TIME
  case TIMING_IGNORE: 
    PCMSK0 = 0;  
    if(milliseconds < timeout2)
      break;
    // otherwise fall thru

    /////////////////////////////////////////
    // READY TO START LISTING FOR A NEW EVENT
  case TIMING_START: 
    TCNT2 = 0;
    TCCR2A = 0;
    TCCR2B = 0;  
    timeOfArrival2A = -1;
    timeOfArrival2B = -1;
    timeOfArrival2C = -1;
    timeOfArrival2D = -1;
    PCMSK0 = BITMASK_2ALL;
    state2 = TIMING_LISTEN;
    break;

    /////////////////////////////////////////
    // LISTENING FOR THE START OF A NEW EVENT 
  case TIMING_LISTEN:
    if((PCMSK0 & BITMASK_2ALL) != BITMASK_2ALL)
      state2 = TIMING_RUN;
    break;

    ////////////////////////////////////
    // LISTENING FOR COMPLETION OF EVENT 
  case TIMING_RUN:
    if(!(PCMSK0 & BITMASK_2ALL))
    {
      onRead2(milliseconds);        
      state2 = TIMING_IGNORE;
      timeout2 = milliseconds + IGNORE_MS;
    }
    else if(TCNT2 > TIMER_TIMEOUT2)
    {
      onMisread2(milliseconds);
      state2 = TIMING_IGNORE;
      timeout2 = milliseconds + IGNORE_MS;
    }
    break;      
  }

  // reading the switches
  if(milliseconds > buttonPdebounce)
  {
    if(!digitalRead(P_SWITCHP))
    {
      if(!buttonPstate) {
        onButtonP();
        buttonPdebounce = milliseconds + BUTTON_DEBOUNCE_MS;
        buttonPstate = 1;
      }
    }
    else 
    {
      buttonPstate = 0;
    }
  }
  if(milliseconds > buttonQdebounce)
  {
    if(!digitalRead(P_SWITCHQ))
    {
      if(!buttonQstate) {
        onButtonQ();
        buttonQdebounce = milliseconds + BUTTON_DEBOUNCE_MS;
        buttonQstate = 1;
      }
    }
    else 
    {
      buttonQstate = 0;
    }
  }

  // maintain LEDs
  UIRun(milliseconds);  
}

//
// EOF
//

