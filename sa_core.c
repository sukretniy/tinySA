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

int settingRefer = -1;  // Off by default
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
int in_selftest = false;

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
    config.low_level_offset = 100;
    config.high_level_offset = 100;
  }
  dirty = true;
}

int settingLevelOffset(void)
{
  if (settingMode == M_HIGH) {
    if (config.high_level_offset == 100)
      return 0;
    return(config.high_level_offset);
  }
  if (settingMode == M_LOW) {
    if (config.low_level_offset == 100)
      return 0;
    return(config.low_level_offset);
  }
  return(0);
}

int level_is_calibrated(void)
{
  if (settingMode == M_HIGH && config.high_level_offset != 100)
    return 1;
  if (settingMode == M_LOW && config.low_level_offset != 100)
    return 1;
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

void SetRefpos(int level)
{
  set_trace_refpos(0, NGRIDY - level / get_trace_scale(0));
  set_trace_refpos(1, NGRIDY - level / get_trace_scale(0));
  set_trace_refpos(2, NGRIDY - level / get_trace_scale(0));
  dirty = true;
}

void SetScale(int s) {
  set_trace_scale(0, s);
  set_trace_scale(1, s);
  set_trace_scale(2, s);
}

void SetMode(int m)
{
  if (settingMode == m)
    return;
  settingMode = m;
  switch(m) {
  case M_LOW:
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_START, (int32_t) 0);
    set_sweep_frequency(ST_STOP, (int32_t) 300000000);
    SetRefpos(-10);
    settingSpur = 0;        // Not for output mode
    break;
  case M_GENLOW:
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_CENTER, (int32_t) 10000000);
    set_sweep_frequency(ST_SPAN, 0);
    settingSpur = 0;        // Not for output mode
    settingRefer = -1;      // No refer output in output mode
    break;
  case M_HIGH:
    minFreq = 240000000;
    maxFreq = 960000000;
    set_sweep_frequency(ST_START, (int32_t) 300000000);
    set_sweep_frequency(ST_STOP, (int32_t) 960000000);
    SetRefpos(-30);
    goto common_high;
  case M_GENHIGH:
    minFreq = 240000000;
    maxFreq = 960000000;
    set_sweep_frequency(ST_CENTER, (int32_t) 300000000);
    set_sweep_frequency(ST_SPAN, 0);
    settingRefer = -1;      // No refer output in output mode
  common_high:
    extraVFO = false;       // Not possible in high mode
    settingSpur = 0;        // Not possible in high mode
    break;
  }
  settingAttenuate = 0;
  SetRBW(0);
  SetScale(10);
  dirty = true;
}


//------------------------------------------


float peakLevel;
float min_level;
uint32_t peakFreq;
int peakIndex;
float temppeakLevel;
int temppeakIndex;

#define BARSTART  24


int vbwSteps = 1;

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

//static int spur_old_stepdelay = 0;
static int spur_IF =            433900000;
static int spur_alternate_IF =  433700000;
static const int spur_table[] =
{
   470000,
   780000,
   830000,
   880000,
   949000,
  1390000,
  1468000,
  1830000,
  1900000,
  2770000,
  2840000,
  2880000,
  4710000,
  4780000,
  4800000,
  4880000,
  6510000,
  6750000,
  6790000,
  6860000,
  7340000,
  8100000,
  8200000,
  8880000,
//  9970000,    10MHz!!!!!!
 10870000,
 11420000,
 14880000,
 16820000,
};

int avoid_spur(int f)
{
  int window = ((int)rbw ) * 1000*2;
  if (window < 50000)
    window = 50000;
  if (! settingMode == M_LOW)
    return false ;
  if (frequency_IF != spur_IF)
    return false;
  if (rbw > 300.0)
    return(false);
//  if (spur_old_stepdelay != 0 && actualStepDelay != spur_old_stepdelay)  // restore stepdelay
//    actualStepDelay = spur_old_stepdelay;
  for (int i = 0; i < (sizeof spur_table)/sizeof(int); i++) {
    if (f/window == spur_table[i]/window) {
//      spur_old_stepdelay = actualStepDelay;
//      actualStepDelay += 4000;
      return true;
    }
  }
  return false;
}

static int old_lf = -1;
static int modulation_counter = 0;
static int old_local_IF = -1;

float perform(bool break_on_operation, int i, int32_t f, int extraV)
{
//  long local_IF = (MODE_LOW(settingMode)?frequency_IF + (int)(rbw < 300.0?settingSpur * 1000 * rbw :0):0);
  long local_IF;
  if (MODE_HIGH(settingMode))
    local_IF = 0;
  else if (avoid_spur(f))
    local_IF = spur_alternate_IF;
  else
    local_IF = frequency_IF;

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

//    if (dirty) {
      scandirty = true;
      dirty = false;
//    }
  }
//  if (i == 0 && ( scandirty || settingSpur) && local_IF)
  if (local_IF && old_local_IF != local_IF) {
    setFreq (0, local_IF);
    old_local_IF = local_IF;
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
  temppeakLevel = -150;
  float temp_min_level = 100;
  //  spur_old_stepdelay = 0;
again:
  for (int i = 0; i < sweep_points; i++) {
    RSSI = perform(break_on_operation, i, frequencies[i], extraVFO);
    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;

    if (settingSpur == 1) {                           // First pass
      temp_t[i] = RSSI;
      continue;                                       // Skip all other processing
    }
    if (settingSpur == -1)                            // Second pass
      RSSI = ( RSSI < temp_t[i] ? RSSI : temp_t[i]);  // Minimum of two passes
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
    if (temp_min_level > actual_t[i])
      temp_min_level = actual_t[i];
  }
  if (settingSpur == 1) {
    settingSpur = -1;
    goto again;
  } else if (settingSpur == -1)
    settingSpur = 1;

  if (scandirty) {
    scandirty = false;
    draw_cal_status();
  }
  peakIndex = temppeakIndex;
  peakLevel = actual_t[peakIndex];
  peakFreq = frequencies[peakIndex];
  min_level = temp_min_level;
#if 0                           // Auto ref level setting
  int scale = get_trace_scale(2);
  int rp = (NGRIDY - get_trace_refpos(2)) * scale;
  if (scale > 0 && peakLevel > rp && peakLevel - min_level < 8 * scale ) {
    SetRefpos((((int)(peakLevel/scale)) + 1) * scale);
  }
  if (scale > 0 && min_level < rp - 9*scale && peakLevel - min_level < 8 * scale ) {
    int new_rp = (((int)((min_level + 9*scale)/scale)) - 1) * scale;
    if (new_rp < rp)
      SetRefpos(new_rp);
  }

#endif
  int peak_marker = 0;
  markers[peak_marker].enabled = true;
  markers[peak_marker].index = peakIndex;
  markers[peak_marker].frequency = frequencies[markers[peak_marker].index];
  //    redraw_marker(peak_marker, FALSE);
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

  ili9341_fill(x, y, OFFSETX, HEIGHT, 0x0000);

  if (MODE_OUTPUT(settingMode))     // No cal status during output
    return;
  if (current_menu_is_form() && !in_selftest)
    return;

  ili9341_set_background(DEFAULT_BG_COLOR);

  int yMax = (NGRIDY - get_trace_refpos(0)) * get_trace_scale(0);
  plot_printf(buf, BLEN, "%ddB", yMax);
  buf[5]=0;
  if (level_is_calibrated())
    color = DEFAULT_FG_COLOR;
  else
    color = BRIGHT_COLOR_RED;
  ili9341_set_foreground(color);
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

  if (dirty)
    ili9341_set_foreground(BRIGHT_COLOR_RED);

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
  if (level_is_calibrated())
    color = DEFAULT_FG_COLOR;
  else
    color = BRIGHT_COLOR_RED;
  ili9341_set_foreground(color);
  ili9341_drawstring(buf, x, y);

}

// -------------------- Self testing -------------------------------------------------

enum {
  TC_SIGNAL, TC_BELOW, TC_ABOVE, TC_FLAT, TC_MEASURE, TC_SET,
};

enum {
  TP_SILENT, TPH_SILENT, TP_10MHZ, TP_10MHZEXTRA, TP_30MHZ, TPH_30MHZ
};

#define TEST_COUNT  13

static const struct {
  int kind;
  int setup;
  float center;      // In MHz
  float span;        // In MHz
  float pass;
  int width;
  float stop;
} test_case [TEST_COUNT] =
{// Condition   Preparation     Center  Span    Pass Width  Stop
 {TC_BELOW,     TP_SILENT,      0.001,  0.0005,  -10,0,     0},         // 1 Zero Hz leakage
 {TC_BELOW,     TP_SILENT,      0.01,  0.01,  -40,   0,     0},         // 2 Phase noise of zero Hz
 {TC_MEASURE,   TP_30MHZ,       30,     7,      -25, 30,    -85 },      // 3 Measure power level and noise
 {TC_SET,       TP_30MHZ,       30,     7,      -25, 0,     0 },        // 4 Calibrate power low mode
 {TC_MEASURE,   TP_30MHZ,       270,    4,      -50, 30,    -85 },      // 5 Measure powerlevel and noise
 {TC_SET,       TPH_30MHZ,      270,    4,      -50,  0,     0 },       // 6 Calibrate power high mode
 {TC_SIGNAL,    TP_10MHZ,       20,     7,      -40, 30,    -90 },      // 7
 {TC_SIGNAL,    TP_10MHZ,       30,     7,      -30, 30,    -90 },      // 8
 {TC_BELOW,     TP_SILENT,      200,    100,    -75, 0,     0},         // 9  Wide band noise floor low mode
 {TC_BELOW,     TPH_SILENT,     600,    720,    -75, 0,     0},         // 10 Wide band noise floor high mode
 {TC_SIGNAL,    TP_10MHZEXTRA,  10,     8,      -20, 50,    -70 },      // 11 BPF loss and stop band
 {TC_FLAT,      TP_10MHZEXTRA,  10,     4,      -25, 20,    -70},       // 12 BPF pass band flatness
 {TC_BELOW,     TP_30MHZ,       430,    60,     -75, 0,     -85},       // 13 LPF cutoff
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
static float test_value;

static void test_acquire(int i)
{
  pause_sweep();
#if 0
  if (test_case[i].center < 300)
    settingMode = M_LOW;
  else
    settingMode = M_HIGH;
#endif
  set_sweep_frequency(ST_CENTER, (int32_t)(test_case[i].center * 1000000));
  set_sweep_frequency(ST_SPAN, (int32_t)(test_case[i].span * 1000000));
  SetAverage(4);
  sweep(false);
  sweep(false);
  sweep(false);
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
//  draw_all(TRUE);
  switch (test_case[i].kind) {
  case TC_SET:
    if (test_case[i].pass == 0) {
      if (test_value != 0)
        SetPowerLevel(test_value);
    } else
      SetPowerLevel(test_case[i].pass);
    goto common;
  case TC_MEASURE:
    case TC_SIGNAL:           // Validate signal
 common:
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
    if (test_status[i] == TS_PASS && test_case[i].kind == TC_MEASURE)
      test_value = peakLevel;
    else
      test_value = 0;           //   Not valid
    break;
  case TC_ABOVE:   // Validate signal above curve
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
    break;
  case TC_BELOW:   // Validate signal below curve
      test_status[i] = validate_below();
      if (test_status[i] != TS_PASS)
        test_fail_cause[i] = "Above ";
      break;
  case TC_FLAT:   // Validate passband flatness
    test_status[i] = validate_flatness(i);
    if (test_status[i] != TS_PASS)
      test_fail_cause[i] = "Passband ";
    break;

  }

  // Report status

  if (test_status[i] != TS_PASS || i == TEST_COUNT - 1)
    test_wait = true;
//  draw_frequencies();
//  draw_cal_status();
  draw_all(TRUE);
  resume_sweep();
}

extern void menu_autosettings_cb(int item);
extern void touch_wait_release(void);

void self_test(void)
{
  in_selftest = true;
  menu_autosettings_cb(0);
  for (int i=0; i < TEST_COUNT; i++) {          // All test cases waiting
    test_status[i] = TS_WAITING;
    test_fail_cause[i] = "";
  }
  show_test_info = TRUE;
  for (int i=0; i < TEST_COUNT; i++) {
    extraVFO = false; //Default test setup
    switch(test_case[i].setup) {                // Prepare test conditions
    case TPH_SILENT:                             // No input signal
      SetMode(M_HIGH);
      goto common_silent;
    case TP_SILENT:                             // No input signal
      SetMode(M_LOW);
common_silent:
      set_refer_output(-1);
      for (int j = 0; j < POINTS_COUNT; j++)
        stored_t[j] = test_case[i].pass;
      break;
    case TP_10MHZEXTRA:                         // Swept receiver
      SetMode(M_LOW);
      extraVFO = true; //Sweep BPF
      set_refer_output(2);
      goto common;
    case TP_10MHZ:                              // 10MHz input
      SetMode(M_LOW);
      set_refer_output(2);
   common:

      for (int j = 0; j < POINTS_COUNT/2 - test_case[i].width; j++)
        stored_t[j] = test_case[i].stop;
      for (int j = POINTS_COUNT/2 + test_case[i].width; j < POINTS_COUNT; j++)
        stored_t[j] = test_case[i].stop;
      for (int j = POINTS_COUNT/2 - test_case[i].width; j < POINTS_COUNT/2 + test_case[i].width; j++)
        stored_t[j] = test_case[i].pass;
      break;
    case TP_30MHZ:
      SetMode(M_LOW);
      set_refer_output(0);
      goto common;
    case TPH_30MHZ:
      SetMode(M_HIGH);
      set_refer_output(0);
      goto common;
    }
    trace[TRACE_STORED].enabled = true;
    SetRefpos(test_case[i].pass+10);
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

  in_selftest = false;
  menu_autosettings_cb(0);
}
