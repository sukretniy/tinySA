// ---------------------------------------------------

#include "SI4432.h"		// comment out for simulation

#if 0
//-----------------SI4432 dummy------------------
void SI4432_Write_Byte(unsigned char ADR, unsigned char DATA ) {}
unsigned char SI4432_Read_Byte(unsigned char ADR) {return ADR;}
float SI4432_SET_RBW(float WISH) {return (WISH > 600.0?600: (WISH<3.0?3:WISH));}
void SI4432_SetReference(int p) {}
void SI4432_Set_Frequency(long f) {}
void PE4302_Write_Byte(unsigned char DATA ) {}
void PE4302_init(void) {}
#endif

#ifdef __SIMULATION__
unsigned long seed = 123456789;
extern float rbw;
float myfrand(void)
{
  seed = (unsigned int) (1103515245 * seed + 12345) ;
  return ((float) seed) / 1000000000.0;
}
#define NOISE  ((myfrand()-2) * 2)  // +/- 4 dBm noise
extern int settingAttenuate;

//#define LEVEL(i, f, v) (v * (1-(fabs(f - frequencies[i])/rbw/1000)))

float LEVEL(uint32_t i, uint32_t f, int v)
{
  float dv;
  float df = fabs((float)f - (float)i);
  if (df < rbw*1000)
    dv = df/(rbw*1000);
  else
    dv =  1 + 50*(df - rbw*1000)/(rbw*1000);
  return (v - dv - settingAttenuate);
}

float Simulated_SI4432_RSSI(uint32_t i, int s)
{
  SI4432_Sel = s;
  float v = -100 + log10(rbw)*10 + NOISE;
  if(s == 0) {
    v = fmax(LEVEL(i,10000000,-20),v);
    v = fmax(LEVEL(i,20000000,-40),v);
    v = fmax(LEVEL(i,30000000,-30),v);
    v = fmax(LEVEL(i,40000000,-90),v);
  } else {
    v = fmax(LEVEL(i,320000000,-20),v);
    v = fmax(LEVEL(i,340000000,-40),v);
    v = fmax(LEVEL(i,360000000,-30),v);
    v = fmax(LEVEL(i,380000000,-90),v);
  }
  return(v);
}

#endif
//--------------------- Frequency control -----------------------

int dirty = true;
int scandirty = true;

//---------------- menu system -----------------------

int settingAttenuate = 0;
// int settingGenerate = 0;
int settingBandwidth = 0;

//int settingLevelOffset = 0;

int settingRefer = 1;
int refferFreq[] = {30000000, 15000000, 10000000, 4000000, 3000000, 2000000, 1000000};
int settingSpur = 0;
int settingAverage = 0;
int settingShowStorage = 0;
int settingSubtractStorage = 0;
int settingMode = M_LOW;
int settingDrive=0; // 0-3 , 3=+20dBm
int settingAGC = true;
int settingLNA = false;
int extraVFO = false;
int settingModulation = MO_NONE;
int settingStepDelay = 0;
float rbw = 0;
float vbw = 0;

uint32_t minFreq = 0;
uint32_t maxFreq = 520000000;

void set_refer_output(int v)
{
  settingRefer = v;
  dirty = true;
}

int get_refer_output(void)
{
  return(settingRefer);
}

#if 0
void SetGenerate(int g)
{
  settingGenerate = g;
  dirty = true;
}
#endif

void SetDrive(int d)
{
  settingDrive = d;
  dirty = true;
}

void SetModulation(int m)
{
  settingModulation = m;
  dirty = true;
}
void SetIF(int f)
{
  frequency_IF = f;
  dirty = true;
}

int GetMode(void)
{
  return(settingMode);
  dirty = true;
}

void SetMode(int m)
{
  if (settingMode == m)
    return;
  settingMode = m;
  switch(m) {
  case M_LOW:
    set_sweep_frequency(ST_START, (int32_t) 0);
    set_sweep_frequency(ST_STOP, (int32_t) 300000000);
    goto min_max_low;
  case M_GENLOW:
    set_sweep_frequency(ST_CENTER, (int32_t) 10000000);
    set_sweep_frequency(ST_SPAN, 0);
  min_max_low:
    minFreq = 0;
    maxFreq = 520000000;
    break;
  case M_HIGH:
    set_sweep_frequency(ST_START, (int32_t) 300000000);
    set_sweep_frequency(ST_STOP, (int32_t) 960000000);
    goto min_max_high;
  case M_GENHIGH:
    set_sweep_frequency(ST_CENTER, (int32_t) 300000000);
    set_sweep_frequency(ST_SPAN, 0);
  min_max_high:
    minFreq = 240000000;
    maxFreq = 960000000;
    break;
  }
  dirty = true;
}


void SetAttenuation(int a)
{
  if (a<0)
    a = 0;
  if (a> 31)
    a=31;
  if (settingAttenuate == a)
    return;
  settingAttenuate = a;
  dirty = true;
}

void SetStorage(void)
{
  for (int i=0; i<POINTS_COUNT;i++)
    stored_t[i] = actual_t[i];
  settingShowStorage = true;
  trace[TRACE_STORED].enabled = true;
  dirty = true;
}

int GetStorage(void)
{
  return(settingShowStorage);
}

void SetClearStorage(void)
{
  settingShowStorage = false;
  settingSubtractStorage = false;
  trace[TRACE_STORED].enabled = false;
  dirty = true;
}

void SetSubtractStorage(void)
{
  if (!settingSubtractStorage) {
    if (!settingShowStorage)
      SetStorage();
    settingSubtractStorage = true;
  } else {
    settingSubtractStorage = false;
  }
  dirty = true;
}

int GetSubtractStorage(void)
{
  return(settingSubtractStorage);
}

extern float peakLevel;
void SetPowerLevel(int o)
{
  if (o != 100) {
    if (settingMode == M_HIGH)
      config.high_level_offset = o - peakLevel - settingAttenuate + settingLevelOffset();
    else if (settingMode == M_LOW)
      config.low_level_offset = o - peakLevel - settingAttenuate + settingLevelOffset();
  }
  else {
    config.low_level_offset = 0;
    config.high_level_offset = 0;
  }
  dirty = true;
}

int settingLevelOffset(void)
{
  if (settingMode == M_HIGH)
    return(config.high_level_offset);
  if (settingMode == M_LOW)
    return(config.high_level_offset);
  return(0);
}

void SetRBW(int v)
{
  settingBandwidth = v;
  update_rbw(frequencies[1] - frequencies[0]);
  dirty = true;
}

int GetRBW(void)
{
  return(settingBandwidth);
}

int GetActualRBW(void)
{
  return((int) rbw);
}
void SetSpur(int v)
{
  settingSpur = v;
  dirty = true;
}

void SetStepDelay(int d)
{
  settingStepDelay = d;
  dirty = true;
}

int GetSpur(void)
{
  return(settingSpur);
}

void SetAverage(int v)
{
  settingAverage = v;
  trace[TRACE_TEMP].enabled = (v != 0);
  dirty = true;
}

int GetAverage(void)
{
  return(settingAverage);
}

void ToggleLNA(void)
{
  settingLNA = !settingLNA;
  dirty = true;
}

int GetLNA(void)
{
  return(settingLNA);
}

void ToggleAGC(void)
{
  settingAGC = !settingAGC;
  dirty = true;
}

int GetAGC(void)
{
  return(settingAGC);
}

//------------------------------------------


float peakLevel;
uint32_t peakFreq;
int peakIndex;
float temppeakLevel;
int temppeakIndex;

#define BARSTART  24


int vbwSteps = 1;

#if 0
int inData = 0;
unsigned long  startFreq = 250000000;
unsigned long  stopFreq = 300000000;
unsigned long  lastFreq[6] = { 300000000, 300000000,0,0,0,0};
int lastParameter[10];
int parameter;
unsigned long reg = 0;
long offset=0;
long offset2=0;
static unsigned int spacing = 10000;
double delta=0.0;
int phase=0;
int deltaPhase;
int delaytime = 50;
#endif


#if 0
void displayHisto ()
{
  //  clearDisplay();
  //int settingMax = 0;
  //int settingMin = -120;

  if (old_settingMax != settingMax || old_settingMin != settingMin) {
    // Display levels at left of screen
    tft.fillRect(0, 0, oX-2, tft.height(), DISPLAY_BLACK);
    textWhite();
    tft.setCursor(0,oY);             // Start at top-left corner
    tft.println(settingMax);
    tft.setCursor(0,tft.height() - 16);
    tft.println(settingMin);
    //  tft.setCursor(0,tft.height()/2);
    //  tft.println("dB");
    old_settingMax = settingMax;
    old_settingMin = settingMin;
  }

  if (old_startFreq != startFreq || old_stopFreq != stopFreq) {
    // Dsiplay frequencies
    // Bottom of screen
    tft.fillRect(0, tft.height()-8, tft.width(), tft.height()-1, DISPLAY_BLACK);
    tft.setTextColor(DISPLAY_WHITE);        // Draw white text
    tft.setCursor(oX+2,tft.height()-8);             // Start at top-left corner
    double f = (((double)(startFreq - lastFreq[0]))/ 1000000.0);
    tft.print(f);
    tft.print("MHz");
    tft.setCursor(tft.width() - 58,tft.height()-8);
    f = (((double)(stopFreq - lastFreq[0]))/ 1000000.0);
    tft.print(f);
    tft.print("MHz");

    tft.setCursor(tft.width()/2 - 80 + oX,tft.height()-8);
    tft.print("center:");
    f = (double)((stopFreq/2 + startFreq/2 - lastFreq[0]) / 1000000.0);
    tft.print(f);
    tft.print("MHz");
    old_startFreq = startFreq;
    old_stopFreq = stopFreq;
  }

  // Top of screen

  if (old_settingAttenuate != settingAttenuate || old_settingPowerGrid != settingPowerGrid) {
    tft.fillRect(0, 0, 8*6, oY-2, DISPLAY_BLACK);
    tft.setCursor(0,0);             // Start at top-left corner
    tft.setTextColor(DISPLAY_WHITE);        // Draw white text
    tft.print("Atten:");
    tft.print(settingAttenuate);
    tft.setCursor(0,8);             // Start at top-left corner
    tft.print(settingPowerGrid);
    tft.print("dB/");
    old_settingAttenuate = settingAttenuate;
    old_settingPowerGrid = settingPowerGrid;
    old_rbw = -1;
  }  

  if (old_rbw != rbw || old_vbw != vbw) {
    tft.fillRect(56, 0, 99, oY-2, DISPLAY_BLACK);
    tft.setCursor(56,0);             // Start at top-left corner
    tft.setTextColor(DISPLAY_WHITE);        // Draw white text
    tft.print("RBW:");
    tft.print(rbw);
    tft.print("kHz");
    tft.setCursor(56,8);             // Start at top-left corner
    tft.print("VBW:");
    tft.print(vbw);
    tft.print("kHz");
    old_rbw = rbw;
    old_vbw = vbw;
  }  

  if (peakLevel > -150) {
    tft.fillRect(oX+100, 0, 100, 8-1, DISPLAY_BLACK);
    tft.setCursor(oX + 100,0);             // Start at top-left corner
    tft.setTextColor(DISPLAY_WHITE);        // Draw white text
    tft.print("Max=");
    tft.print((int)((peakLevel/ 2.0  - settingAttenuate) - 120.0)+settingLevelOffset);
    tft.print("dB, ");
    tft.print(peakFreq/ 1000000.0);
    tft.print("MHz");
  }

  if (old_settingAverage != settingAverage || abs(old_settingSpur) != abs(settingSpur)) {
    int x =  tft.width() - 60;
    tft.fillRect( x, 0, 60, oY-2, DISPLAY_BLACK);
    tft.setTextColor(DISPLAY_WHITE);        // Draw white text
    if (settingAverage) {
      tft.setCursor( x,0);             // Start at top-left corner
      tft.print("AVR:");
      tft.print(averageText[settingAverage]);
    }
    if (settingSpur) {
      tft.setCursor(x,8);             // Start at top-left corner
      tft.print("SPUR:");
      tft.print("ON");
    }
    old_settingAverage = settingAverage;
    old_settingSpur = settingSpur;
  }  



  /*
  for (int i=0; i<DISPLAY_POINTS - 1; i++) {
    int delta=settingMax - settingMin;
    DrawCheckerBoard(i);
    double f = ((actual_t[i] / 2.0  - settingAttenuate) - 120.0) + settingLevelOffset;
    f = (f - settingMin) * Y_GRID * dY / delta;
    if (f >= Y_GRID * dY) f = Y_GRID * dY-1;
    if (f < 0) f = 0;
    double f2 = ((actual_t[i+1] / 2.0  - settingAttenuate) - 120.0) + settingLevelOffset;
    f2 = (f2 - settingMin) * Y_GRID * dY / delta;
    if (f2 >= Y_GRID * dY) f2 = Y_GRID * dY-1;
    if (f2 < 0) f2 = 0;
  int x = i;
  int Y1 = Y_GRID * dY - 1 - (int)f;
  int Y2 = Y_GRID * dY - 1 - (int)f2;
  tft.drawLine(x+oX, oY+Y1, x+oX+1, oY+Y2, DISPLAY_YELLOW);
//  tft.drawLine(x+oX, oY+Y1+1, x+oX+1, oY+Y2, DISPLAY_YELLOW);
  }


   */
  sendDisplay();
}

void DisplayPoint(unsigned char *data, int i, int color)
{
  if (i == 0)
    return;
  int x = i-1;
  int delta=settingMax - settingMin;
  double f = ((data[x] / 2.0  - settingAttenuate) - 120.0) + settingLevelOffset;
  f = (f - settingMin) * Y_GRID * dY / delta;
  if (f >= Y_GRID * dY) f = Y_GRID * dY-1;
  if (f < 0) f = 0;
  double f2 = ((data[x+1] / 2.0  - settingAttenuate) - 120.0) + settingLevelOffset;
  f2 = (f2 - settingMin) * Y_GRID * dY / delta;
  if (f2 >= Y_GRID * dY) f2 = Y_GRID * dY-1;
  if (f2 < 0) f2 = 0;
  int Y1 = Y_GRID * dY - 1 - (int)f;
  int Y2 = Y_GRID * dY - 1 - (int)f2;
  DrawDirty(x,min(Y2,Y1));
  DrawDirty(x+1,min(Y2,Y1));
  tft.drawLine(x+oX, oY+Y1, x+oX+1, oY+Y2, color);
  //  tft.drawLine(x+oX, oY+Y1+1, x+oX+1, oY+Y2, DISPLAY_YELLOW);
  sendDisplay();
}

void DisplayPeakData(void)
{
  double f = ((((float)actual_t[peakIndex]) / 2.0  - settingAttenuate) - 120.0) + settingLevelOffset;
  int delta=settingMax - settingMin;
  f = (f - settingMin) * Y_GRID * dY / delta;
  if (f >= Y_GRID * dY) f = Y_GRID * dY-1;
  if (f < 0) f = 0;
  int Y1 = Y_GRID * dY - 1 - (int)f;
  tft.setCursor(oX+peakIndex+5,oY+Y1);             // Start at top-left corner
  tft.setTextColor(DISPLAY_WHITE);        // Draw white text
  tft.print(peakFreq/ 1000000.0);
  tft.setCursor(oX+peakIndex+5,oY+Y1+8);             // Start at top-left corner
  tft.print((int)((peakLevel/ 2.0  - settingAttenuate) - 120.0)+settingLevelOffset);
  tft.print("dB");
  for (int x=peakIndex+5;x<peakIndex+5+6*8;x++)
    DrawDirty(x,Y1);
}

#endif

void setupSA(void)
{
  SI4432_Init();
  PE4302_init();
  PE4302_Write_Byte(0);
}


void setFreq(int V, unsigned long freq)
{
  if (V>=0) {
    SI4432_Sel = V;
#ifdef USE_SI4463
    if (SI4432_Sel == 2) {
      freq = freq - 433000000;
      freq = freq / 10000;  //convert to 10kHz channel starting with 433MHz
      //      Serial.print("Set frequency Si4463 = ");
      //      Serial.println(freq);
      Si446x_RX ((uint8_t)freq);
    }
    else
#endif
      SI4432_Set_Frequency(freq);
  }
}

void SetSwitchTransmit(void) {
  SI4432_Write_Byte(0x0b, 0x1f);// Set switch to transmit
  SI4432_Write_Byte(0x0c, 0x1d);
}

void SetSwitchReceive(void) {
  SI4432_Write_Byte(0x0b, 0x1d);// Set switch to receive
  SI4432_Write_Byte(0x0c, 0x1f);
}

void SetAGCLNA(void) {
  unsigned char v = 0x40;
  if (settingAGC) v |= 0x20;
  if (settingLNA) v |= 0x10;
  SI4432_Write_Byte(0x69, v);
}

void SetRX(int m)
{
switch(m) {
case M_LOW:     // Mixed into 0
    SI4432_Sel = 0;
    SI4432_Receive();
    SetSwitchReceive();
    SetAGCLNA();

    SI4432_Sel = 1;
    SetSwitchReceive();
//    SI4432_Receive(); For noise testing only
    SI4432_Transmit(settingDrive);
    // SI4432_SetReference(settingRefer);
    break;
case M_HIGH:    // Direct into 1
    // SI4432_SetReference(-1); // Stop reference output
    SI4432_Sel = 0; // both as receiver to avoid spurs
    SetSwitchReceive();
    SI4432_Receive();

    SI4432_Sel = 1;
    SI4432_Receive();
    SetSwitchReceive();
    SetAGCLNA();

    break;
case M_GENLOW:  // Mixed output from 0
    SI4432_Sel = 0;
    SetSwitchTransmit();
    SI4432_Transmit(settingDrive);

    SI4432_Sel = 1;
    SetSwitchReceive();
    SI4432_Transmit(settingDrive);

    break;
case M_GENHIGH: // Direct output from 1
    SI4432_Sel = 0;
    SI4432_Receive();
    SetSwitchReceive();

    SI4432_Sel = 1;
    SetSwitchTransmit();
    SI4432_Transmit(settingDrive);

    break;
  }
}

void update_rbw(uint32_t delta_f)
{
  vbw = (delta_f)/1000.0;
  rbw = settingBandwidth;
//  float old_rbw = rbw;
  if (rbw == 0)
    rbw = 2*vbw;
  if (rbw < 2.6)
    rbw = 2.6;
  if (rbw > 600)
    rbw = 600;
  SI4432_Sel =  MODE_SELECT(settingMode);
  rbw = SI4432_SET_RBW(rbw);
  vbwSteps = ((int)(2 * vbw / rbw));
  if (vbwSteps < 1)
    vbwSteps = 1;
  dirty = true;
}

static int old_lf = -1;
static int modulation_counter = 0;

float perform(bool break_on_operation, int i, int32_t f, int extraV)
{
  long local_IF = (MODE_LOW(settingMode)?frequency_IF + (int)(rbw < 300.0?settingSpur * 1000 * rbw :0):0);
  if (i == 0 && dirty) {
    if (settingStepDelay == 0){
      if (rbw < 10.0)
        actualStepDelay = 2500;
      else if (rbw <30.0)
        actualStepDelay = 2000;
      else if (rbw <100.0)
        actualStepDelay = 1000;
      else
        actualStepDelay = 500;
    } else
      actualStepDelay = settingStepDelay;

//    setupSA();

    int p = settingAttenuate * 2;
    PE4302_Write_Byte(p);
    if (settingModulation == MO_NFM ) {
      SI4432_Sel = 1;
      SI4432_Write_Byte(0x7A, 1);  // Use frequency hopping channel width for FM modulation
    } else if (settingModulation == MO_WFM ) {
      SI4432_Sel = 1;
      SI4432_Write_Byte(0x7A, 10);  // Use frequency hopping channel width for FM modulation
    } else {
      SI4432_Sel = 1;
      SI4432_Write_Byte(0x79, 0);  // IF no FM back to channel 0
    }
    SetRX(settingMode);
    SI4432_SetReference(settingRefer);
    temppeakLevel = -150;
    if (local_IF)
      setFreq (0, local_IF);
//    if (dirty) {
      scandirty = true;
      dirty = false;
//    }
  }
  if (settingModulation == MO_AM) {
    int p = settingAttenuate * 2 + modulation_counter;
    PE4302_Write_Byte(p);
    if (modulation_counter == 3)
      modulation_counter = 0;
    else
      modulation_counter++;
    chThdSleepMicroseconds(250);
  } else if (settingModulation == MO_NFM || settingModulation == MO_WFM ) {
      SI4432_Sel = 1;
      SI4432_Write_Byte(0x79, modulation_counter);  // Use frequency hopping channel for FM modulation
      if (modulation_counter == 3)
        modulation_counter = 0;
      else
        modulation_counter++;
      chThdSleepMicroseconds(250);
  }
  volatile int subSteps = ((int)(2 * vbw / rbw));
  float RSSI = -150.0;
  int t = 0;
  do {
    int lf = (uint32_t)(f + (int)(t * 500 * rbw));
    if (extraV)
      setFreq (0, local_IF + lf - refferFreq[settingRefer]);    // Offset so fundamental of reffer is visible
    if (lf != old_lf)                                           // only set on change
      setFreq (1, local_IF + lf);
    old_lf = lf;
    if (MODE_OUTPUT(settingMode))
      return(0);
    float subRSSI = SI4432_RSSI(lf, MODE_SELECT(settingMode))+settingLevelOffset()+settingAttenuate;
    if (RSSI < subRSSI)
      RSSI = subRSSI;
    t++;
    if ((operation_requested && break_on_operation ) || (MODE_OUTPUT(settingMode))) // output modes do not step.
      subSteps = 0;         // abort
  } while (subSteps-- > 0);
  return(RSSI);
}

// main loop for measurement
static bool sweep(bool break_on_operation)
{
  float RSSI;
  palClearPad(GPIOC, GPIOC_LED);
  for (int i = 0; i < sweep_points; i++) {
again:
    RSSI = perform(break_on_operation, i, frequencies[i], extraVFO);
    if (settingSpur == 1)
      temp_t[i] = RSSI;
    else
    {
      if (settingSpur == -1)
        RSSI = ( RSSI < temp_t[i] ? RSSI : temp_t[i]);
      temp_t[i] = RSSI;
      if (settingSubtractStorage) {
        RSSI = RSSI - stored_t[i] ;
      }
      //   stored_t[i] = (SI4432_Read_Byte(0x69) & 0x0f) * 3.0 - 90.0; // Display the AGC value in thestored trace
      if (scandirty || settingAverage == AV_OFF)
        actual_t[i] = RSSI;
      else {
        switch(settingAverage) {
        case AV_MIN: if (actual_t[i] > RSSI) actual_t[i] = RSSI; break;
        case AV_MAX: if (actual_t[i] < RSSI) actual_t[i] = RSSI; break;
        case AV_2: actual_t[i] = (actual_t[i] + RSSI) / 2.0; break;
        case AV_4: actual_t[i] = (actual_t[i]*3 + RSSI) / 4.0; break;
        case AV_8: actual_t[i] = (actual_t[i]*7 + RSSI) / 8.0; break;
        }
      }
      if (frequencies[i] > 1000000) {
        if (temppeakLevel < actual_t[i]) {
          temppeakIndex = i;
          temppeakLevel = actual_t[i];
        }
      }
    }
    if (i == sweep_points -1) {
      if (settingSpur == 1) {
        settingSpur = -1;
        i = 0;
        goto again;
      }
      if (scandirty) {
        scandirty = false;
      }
      peakIndex = temppeakIndex;
      peakLevel = actual_t[peakIndex];
      peakFreq = frequencies[peakIndex];
      settingSpur = -settingSpur;
      int peak_marker = 0;
      markers[peak_marker].enabled = true;
      markers[peak_marker].index = peakIndex;
      markers[peak_marker].frequency = frequencies[markers[peak_marker].index];
      //    redraw_marker(peak_marker, FALSE);


    }
    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;
  }
  palSetPad(GPIOC, GPIOC_LED);
  return true;
}


#if 0
void PeakSearch()
{
#define PEAKSTACK   4
#define PEAKDISTANCE    10
  int level = 0;
  int searchLeft[PEAKSTACK];
  int peakIndex[PEAKSTACK];
  int peak_marker = 0;
  searchLeft[level] = true;
  peakIndex[level] = markers[peak_marker].index;
  level++;
  searchLeft[level] = true;
  int peakFrom;
  int peakTo;
  while (peak_marker < 4){
    if (searchLeft[level])
    {
      int fromLevel = level;
      while (fromLevel > 0 && searchLeft[fromLevel])
        fromLevel--
      if(fromLevel == 0) {
        peakFrom = PEAKDISTANCE;
      } else {
        peakFrom = peakIndex[fromLevel] + PEAKDISTANCE;
      }
      peakTo = peakIndex[level] - PEAKDISTANCE;
    } else {
      int toLevel = level;
      while (toLevel > 0 && !searchLeft[toLevel])
        toLevel--
      if(toLevel == 0) {
        peakTo = POINTS_COUNT - 1 - PEAKDISTANCE;
      } else {
        peakTo = peakIndex[fromLevel] - PEAKDISTANCE;
      }
      peakFrom = peakIndex[level] + PEAKDISTANCE;
    }
    float peakMax = actual_t[peakFrom];
    int peakIndex = peakFrom;
    for (int i = peakFrom; i < peakTo; i++) {
      if (peakMax < actual_t[i]) {
        peakMax = actual_t[i];
        peakIndex = i;
      }
    }


  peakIndex = temppeakIndex;
  peakLevel = actual_t[peakIndex];
  peakFreq = frequencies[peakIndex];
  settingSpur = -settingSpur;
  int peak_marker = 0;
  markers[peak_marker].enabled = true;
  markers[peak_marker].index = peakIndex;
  markers[peak_marker].frequency = frequencies[markers[peak_marker].index];
//    redraw_marker(peak_marker, FALSE);


}

}
#endif

char *averageText[] = { "OFF", "MIN", "MAX", "2", "4", "8"};
char *dBText[] = { "1dB/", "2dB/", "5dB/", "10dB/", "20dB/"};
int refMHz[] = { 30, 15, 10, 4, 3, 2, 1 };

void draw_cal_status(void)
{
#define BLEN    10
  char buf[BLEN];
#define YSTEP   8
  int x = 0;
  int y = OFFSETY;
  unsigned int color;

#define XSTEP   40

//  if (!sweep_enabled)
//    perform(true, 0, frequencies[0], false);

  ili9341_fill(x, y, OFFSETX, HEIGHT, 0x0000);
  ili9341_set_background(DEFAULT_BG_COLOR);

  int yMax = (NGRIDY - get_trace_refpos(0)) * get_trace_scale(0);
  plot_printf(buf, BLEN, "%ddB", yMax);
  buf[5]=0;
  ili9341_set_foreground(DEFAULT_FG_COLOR);
  ili9341_drawstring(buf, x, y);

  y += YSTEP*2;
  plot_printf(buf, BLEN, "%ddB/",(int)get_trace_scale(0));
  ili9341_drawstring(buf, x, y);

  if (settingAttenuate) {
    ili9341_set_foreground(BRIGHT_COLOR_GREEN);
    y += YSTEP*2;
    ili9341_drawstring("Attn:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "-%ddB", settingAttenuate);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  if (settingAverage>0) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP*2;
    ili9341_drawstring("Aver:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%s",averageText[settingAverage]);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  if (settingSpur) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP*2;
    ili9341_drawstring("Spur:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "ON");
    ili9341_drawstring(buf, x, y);
  }

  if (settingBandwidth)
    color = BRIGHT_COLOR_GREEN;
  else
    color = DEFAULT_FG_COLOR;
  ili9341_set_foreground(color);

  y += YSTEP*2;
  ili9341_drawstring("RBW:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%dkHz", (int)rbw);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  ili9341_set_foreground(DEFAULT_FG_COLOR);
  y += YSTEP*2;
  ili9341_drawstring("VBW:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%dkHz",(int)vbw);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  y += YSTEP*2;
  ili9341_drawstring("Scan:", x, y);

  y += YSTEP;
  int32_t t = (int)((2* vbwSteps * sweep_points * ( actualStepDelay / 100) )) /10 * (settingSpur ? 2 : 1); // in mS
  if (t>1000)
    plot_printf(buf, BLEN, "%dS",(t+500)/1000);
  else
    plot_printf(buf, BLEN, "%dmS",t);

  buf[5]=0;
  ili9341_drawstring(buf, x, y);


  if (settingRefer >= 0) {
    ili9341_set_foreground(BRIGHT_COLOR_RED);
    y += YSTEP*2;
    ili9341_drawstring("Ref:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%dMHz",refMHz[settingRefer]);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  y = HEIGHT-7 + OFFSETY;
  plot_printf(buf, BLEN, "%ddB", (int)(yMax - get_trace_scale(0) * NGRIDY));
  buf[5]=0;
  ili9341_set_foreground(DEFAULT_FG_COLOR);
  ili9341_drawstring(buf, x, y);

}

// -------------------- Self testing -------------------------------------------------

enum {
  TC_SIGNAL, TC_BELOW, TC_ABOVE, TC_FLAT
};

enum {
  TP_SILENT, TP_10MHZ, TP_10MHZEXTRA, TP_30MHZ
};

#define TEST_COUNT  7

static const struct {
  int kind;
  int setup;
  uint32_t center;      // In MHz
  int span;             // In MHz
  float pass;
  int width;
  float stop;
} test_case [TEST_COUNT] =
{// Condition   Preparation     Center  Span    Pass Width  Stop
 {TC_SIGNAL,    TP_10MHZ,       10,     7,      -30, 30,    -85 },
 {TC_SIGNAL,    TP_10MHZ,       20,     7,      -50, 30,    -90 },
 {TC_SIGNAL,    TP_10MHZ,       30,     7,      -40, 30,    -90 },
 {TC_BELOW,     TP_SILENT,      200,    100,    -80, 0,     0},
 {TC_SIGNAL,    TP_10MHZEXTRA,  10,     8,      -30, 50,    -80 },
 {TC_FLAT,      TP_10MHZEXTRA,  10,     4,      -35, 20,    -80},
 {TC_SIGNAL,    TP_30MHZ,       360,    18,     -70, 20,    -100 },
};

enum {
  TS_WAITING, TS_PASS, TS_FAIL, TS_CRITICAL
};
static const  char *(test_text [4]) =
{
 "Waiting", "Pass", "Fail", "Critical"
};
static const  char *(test_fail_cause [TEST_COUNT]);

static int test_status[TEST_COUNT];
static int show_test_info = FALSE;
static volatile int test_wait = false;

static void test_acquire(int i)
{
  pause_sweep();
  if (test_case[i].center < 300)
    settingMode = M_LOW;
  else
    settingMode = M_HIGH;
  set_sweep_frequency(ST_CENTER, (int32_t)test_case[i].center * 1000000);
  set_sweep_frequency(ST_SPAN, (int32_t)test_case[i].span * 1000000);
  sweep(false);
  plot_into_index(measured);
  redraw_request |= REDRAW_CELLS | REDRAW_FREQUENCY;
}

extern void cell_drawstring_5x7(int w, int h, char *str, int x, int y, uint16_t fg);
extern void cell_drawstring_7x13(int w, int h, char *str, int x, int y, uint16_t fg);
void cell_drawstring(char *str, int x, int y);

static char self_test_status_buf[35];
void cell_draw_test_info(int x0, int y0)
{
#define INFO_SPACING    13
//  char self_test_status_buf[35];
  if (!show_test_info)
    return;
  for (int i = -1; i < TEST_COUNT+1; i++) {
    int xpos = 25 - x0;
    int ypos = 40+i*INFO_SPACING - y0;
    unsigned int color = RGBHEX(0xFFFFFF);
    if (i == -1) {
        plot_printf(self_test_status_buf, sizeof self_test_status_buf, "Self test status:");
    } else if (i == TEST_COUNT) {
        if (test_wait)
          plot_printf(self_test_status_buf, sizeof self_test_status_buf, "Touch screen to continue");
        else
          self_test_status_buf[0] = 0;
      } else {
      plot_printf(self_test_status_buf, sizeof self_test_status_buf, "Test %d: %s%s", i+1, test_fail_cause[i], test_text[test_status[i]] );
      if (test_status[i] == TS_PASS)
        color = RGBHEX(0x00FF00);
      else if (test_status[i] == TS_CRITICAL)
        color = RGBHEX(0xFFFF00);
      else if (test_status[i] == TS_FAIL)
        color = RGBHEX(0xFF7F7F);
      else
        color = RGBHEX(0x0000FF);
    }
    ili9341_set_foreground(color);
    cell_drawstring(self_test_status_buf, xpos, ypos);
  }
}

#define fabs(X) ((X)<0?-(X):(X))

int validate_peak_within(int i, float margin)
{
  if (fabs(peakLevel-test_case[i].pass) > margin)
    return false;
  return(test_case[i].center * 1000000 - 100000 < peakFreq && peakFreq < test_case[i].center * 1000000 + 100000 );
}

int validate_peak_below(int i, float margin) {
  return(test_case[i].pass - peakLevel > margin);
}

int validate_below(void) {
  int status = TS_PASS;
  for (int j = 0; j < POINTS_COUNT; j++) {
    if (actual_t[j] > stored_t[j] - 5)
      status = TS_CRITICAL;
    else if (actual_t[j] > stored_t[j]) {
      status = TS_FAIL;
      break;
    }
  }
  return(status);
}

int validate_flatness(int i) {
  volatile int j;
  for (j = peakIndex; j < POINTS_COUNT; j++) {
    if (actual_t[j] < peakLevel - 3)    // Search right -3dB
      break;
  }
  if (j - peakIndex < test_case[i].width)
    return(TS_FAIL);
  for (j = peakIndex; j > 0; j--) {
    if (actual_t[j] < peakLevel - 3)    // Search left -3dB
      break;
  }
  if (peakIndex - j < test_case[i].width)
    return(TS_FAIL);
  return(TS_PASS);
}

int validate_above(void) {
  int status = TS_PASS;
  for (int j = 0; j < POINTS_COUNT; j++) {
    if (actual_t[j] < stored_t[j] + 5)
      status = TS_CRITICAL;
    else if (actual_t[j] < stored_t[j]) {
      status = TS_FAIL;
      break;
    }
  }
  return(status);
}


void test_validate(int i)
{
  if (test_case[i].kind == TC_SIGNAL) {         // Validate signal
    if (validate_peak_within(i, 5.0))                // Validate Peak
      test_status[i] = TS_PASS;
    else if (validate_peak_within(i, 10.0))
      test_status[i] = TS_CRITICAL;
    else
      test_status[i] = TS_FAIL;
    if (test_status[i] != TS_PASS)
      test_fail_cause[i] = "Peak ";
    if (test_status[i] == TS_PASS) {            // Validate noise floor
      for (int j = 0; j < POINTS_COUNT/2 - test_case[i].width; j++) {
        if (actual_t[j] > test_case[i].stop - 5)
          test_status[i] = TS_CRITICAL;
        else if (actual_t[j] > test_case[i].stop) {
          test_status[i] = TS_FAIL;
          break;
        }
      }
      for (int j = POINTS_COUNT/2 + test_case[i].width; j < POINTS_COUNT; j++) {
        if (actual_t[j] > test_case[i].stop - 5)
          test_status[i] = TS_CRITICAL;
        else if (actual_t[j] > test_case[i].stop) {
          test_status[i] = TS_FAIL;
          break;
        }
      }
      if (test_status[i] != TS_PASS)
        test_fail_cause[i] = "Stopband ";
    }

  } else if (test_case[i].kind == TC_ABOVE) {   // Validate signal above curve
    for (int j = 0; j < POINTS_COUNT; j++) {
      if (actual_t[j] < test_case[i].pass + 5)
        test_status[i] = TS_CRITICAL;
      else if (actual_t[j] < test_case[i].pass) {
        test_status[i] = TS_FAIL;
        break;
      }
    }
    if (test_status[i] != TS_PASS)
      test_fail_cause[i] = "Above ";

  } else if (test_case[i].kind == TC_BELOW) {   // Validate signal below curve
      if (validate_peak_below(i, 10.0))
        test_status[i] = TS_PASS;
      else if (validate_peak_below(i, 5.0))
        test_status[i] = TS_CRITICAL;
      else
        test_status[i] = TS_FAIL;
      if (test_status[i] != TS_PASS)
        test_fail_cause[i] = "Above ";
  } else if (test_case[i].kind == TC_FLAT) {   // Validate passband flatness
    test_status[i] = validate_flatness(i);
    if (test_status[i] != TS_PASS)
      test_fail_cause[i] = "Passband ";
  }

  // Report status

  if (test_status[i] != TS_PASS || i == TEST_COUNT - 1)
    test_wait = true;
  draw_all(TRUE);
  resume_sweep();
}

extern void menu_autosettings_cb(int item);
extern void touch_wait_release(void);

void self_test(void)
{
  menu_autosettings_cb(0);
  for (int i=0; i < TEST_COUNT; i++) {          // All test cases waiting
    test_status[i] = TS_WAITING;
    test_fail_cause[i] = "";
  }
  show_test_info = TRUE;
  for (int i=0; i < TEST_COUNT; i++) {
    extraVFO = false; //Default test setup
    switch(test_case[i].setup) {                // Prepare test conditions
    case TP_SILENT:                             // No input signal
      set_refer_output(-1);
      for (int j = 0; j < POINTS_COUNT; j++)
        stored_t[j] = test_case[i].pass;
      break;
    case TP_10MHZEXTRA:                         // Swept receiver
      extraVFO = true; //Sweep BPF
      goto common;
    case TP_10MHZ:                              // 10MHz input
      common:
      set_refer_output(2);
      int j;
      for (j = 0; j < POINTS_COUNT/2 - test_case[i].width; j++)
        stored_t[j] = test_case[i].stop;
      for (j = POINTS_COUNT/2 + test_case[i].width; j < POINTS_COUNT; j++)
        stored_t[j] = test_case[i].stop;
      for (j = POINTS_COUNT/2 - test_case[i].width; j < POINTS_COUNT/2 + test_case[i].width; j++)
        stored_t[j] = test_case[i].pass;
      break;
    case TP_30MHZ:
      set_refer_output(0);
      goto common;
    }
    trace[TRACE_STORED].enabled = true;
    set_trace_refpos(0, NGRIDY - (test_case[i].pass + 30) / get_trace_scale(0));
    set_trace_refpos(1, NGRIDY - (test_case[i].pass + 30) / get_trace_scale(0));
    set_trace_refpos(2, NGRIDY - (test_case[i].pass + 30) / get_trace_scale(0));
    draw_cal_status();
    test_acquire(i);                        // Acquire test
    test_validate(i);                       // Validate test
    chThdSleepMilliseconds(2000);
    if (test_status[i] != TS_PASS) {
      touch_wait_release();
    }
  }
  touch_wait_release();
    //  chThdSleepMilliseconds(2000);
  show_test_info = FALSE;
  trace[TRACE_STORED].enabled = false;
  set_trace_refpos(0, NGRIDY - (-10) / get_trace_scale(0));
  set_trace_refpos(1, NGRIDY - (-10) / get_trace_scale(0));
  set_trace_refpos(2, NGRIDY - (-10) / get_trace_scale(0));
  set_refer_output(0);
  settingMode = M_LOW;
  draw_cal_status();

  menu_autosettings_cb(0);
}
