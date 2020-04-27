#include "SI4432.h"		// comment out for simulation

int dirty = true;
int scandirty = true;
#if 0
int setting.mode = -1;      // To force initialzation
int setting.attenuate = 0;
int setting.auto_attenuation;
int setting.step_atten;
int setting.rbw = 0;
int setting.below_IF;
int setting.average = 0;
int setting.show_stored = 0;
int setting.subtract_stored = 0;
int setting.drive; // 0-7 , 7=+20dBm, 3dB steps
int setting.agc = true;
int setting.lna = false;
int setting.auto_reflevel;
int setting.reflevel;
int setting.scale;
int setting.tracking = false;
int setting.modulation = MO_NONE;
int setting.step_delay = 0;
int setting.frequency_step;
int setting.test;
int setting.harmonic;
int setting.decay;
int setting.noise;
float setting.vbw = 0;
int  setting.tracking_output;
int setting.repeat;
uint32_t setting.frequency0;
uint32_t setting.frequency1;
uint32_t setting.setting.frequency_IF;
int setting.measurement;
#endif

extern int actualStepDelay;

setting_t setting;
uint32_t frequencies[POINTS_COUNT];

float actual_rbw = 0;


int vbwSteps = 1;
#ifdef __ULTRA__
int setting.spur = 0;
#endif
float minFreq = 0;
float maxFreq = 520000000;

//int setting.refer = -1;  // Off by default
const int reffer_freq[] = {30000000, 15000000, 10000000, 4000000, 3000000, 2000000, 1000000};

int in_selftest = false;

void reset_settings(int m)
{
  setting.mode = m;
  SetScale(10);
  SetReflevel(-10);
  setting.attenuate = 0;
  setting.rbw = 0;
  setting.average = 0;
  setting.harmonic = 0;
  setting.show_stored = 0;
  setting.auto_attenuation = true;
  setting.subtract_stored = 0;
  setting.drive=13;
  setting.step_atten = 0;       // Only used in low output mode
  setting.agc = true;
  setting.lna = false;
  setting.tracking = false;
  setting.modulation = MO_NONE;
  setting.step_delay = 0;
  setting.vbw = 0;
  setting.auto_reflevel = true;     // Must be after SetReflevel
  setting.decay=20;
  setting.noise=5;
  setting.below_IF = false;
  setting.repeat = 1;
  setting.tracking_output = false;
  setting.measurement = M_OFF;
  setting.frequency_IF = 433800000;
  trace[TRACE_STORED].enabled = false;
  trace[TRACE_TEMP].enabled = false;
  #ifdef __ULTRA__
  setting.spur = 0;
#endif
  switch(m) {
  case -1:
    setting.refer = -1; // Clear cal signal when no autostart
    break;
  case M_LOW:
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_START, (uint32_t) 0);
    set_sweep_frequency(ST_STOP, (uint32_t) 350000000);
    setting.attenuate = 30;
    break;
#ifdef __ULTRA__
  case M_ULTRA:
    minFreq = 870000000;
    if (setting.harmonic * 240000000 >  870000000)
      minFreq = setting.harmonic * 240000000;
    if (setting.harmonic == 0)
      maxFreq = 4360000000;
    else
      maxFreq = 960000000 * setting.harmonic;
    set_sweep_frequency(ST_START, (uint32_t) minFreq);
    set_sweep_frequency(ST_STOP, (uint32_t) maxFreq);
    setting.attenuate = 0;
    break;
#endif
  case M_GENLOW:
    setting.drive=8;
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_CENTER, (int32_t) 10000000);
    set_sweep_frequency(ST_SPAN, 0);
    break;
  case M_HIGH:
#ifdef __ULTRA_SA__
    minFreq = 00000000;
    maxFreq = 2000000000;
#else
    minFreq = 240000000;
    maxFreq = 960000000;
#endif
    set_sweep_frequency(ST_START, (int32_t) minFreq);
    set_sweep_frequency(ST_STOP, (int32_t) maxFreq);
    break;
  case M_GENHIGH:
    setting.drive=8;
    minFreq = 240000000;
    maxFreq = 960000000;
    set_sweep_frequency(ST_CENTER, (int32_t) 300000000);
    set_sweep_frequency(ST_SPAN, 0);
    break;
  }
  for (int i = 0; i< MARKERS_MAX; i++) {
    markers[i].enabled = M_DISABLED;
    markers[i].mtype = M_NORMAL;
  }
  markers[0].mtype = M_REFERENCE | M_TRACKING;
  markers[0].enabled = M_ENABLED;

  dirty = true;
}

void set_refer_output(int v)
{
  setting.refer = v;
  dirty = true;
}

int get_refer_output(void)
{
  return(setting.refer);
}

void set_decay(int d)
{
  if (d < 0 || d > 200)
    return;
  setting.decay = d;
  dirty = true;
}

void set_noise(int d)
{
  if (d < 2 || d > 50)
    return;
  setting.noise = d;
  dirty = true;
}

void set_measurement(int m)
{
  setting.measurement = m;
  dirty = true;
}
void SetDrive(int d)
{
  setting.drive = d;
  dirty = true;
}

void set_tracking_output(int t)
{
  setting.tracking_output = t;
  dirty = true;
}

void toggle_tracking_output(void)
{
  setting.tracking_output = !setting.tracking_output;
  dirty = true;
}
void toggle_below_IF(void)
{
  setting.below_IF = !setting.below_IF;
  dirty = true;
}

void SetModulation(int m)
{
  setting.modulation = m;
  dirty = true;
}

void set_repeat(int r)
{
  if (r > 0 && r < 50) {
    setting.repeat = r;
    dirty = true;
  }
}

void SetIF(int f)
{
  setting.frequency_IF = f;
  dirty = true;
}

int GetMode(void)
{
  return(setting.mode);
  dirty = true;
}


#define POWER_STEP  0           // Should be 5 dB but appearently it is lower
#define POWER_OFFSET    20
#define SWITCH_ATTENUATION  29

int GetAttenuation(void)
{
  if (setting.mode == M_GENLOW) {
    if (setting.step_atten)
      return ( -(POWER_OFFSET + setting.attenuate - (setting.step_atten-1)*POWER_STEP + SWITCH_ATTENUATION));
    else
      return ( -POWER_OFFSET - setting.attenuate + (setting.drive & 7) * 3);
  }
  return(setting.attenuate);
}

void set_auto_attenuation(void)
{
  setting.auto_attenuation = true;
  setting.attenuate = 30;
}

void set_auto_reflevel(void)
{
  setting.auto_reflevel = true;
}

void SetAttenuation(int a)
{
  if (setting.mode == M_GENLOW) {
    setting.drive = 8;              // Start at lowest drive level;
    a = a + POWER_OFFSET;
    if (a > 0) {
      setting.drive++;
      a = a - 3;
    }
    if (a > 0) {
      setting.drive++;
      a = a - 3;
    }
    if (a > 0) {
      setting.drive++;
      a = a - 3;
    }
    if (a > 0)
      a = 0;
    if( a >  - SWITCH_ATTENUATION) {
      setting.step_atten = 0;
    } else {
      a = a + SWITCH_ATTENUATION;
      setting.step_atten = 1;
    }
    a = -a;
  } else {
    setting.step_atten = 0;
  }
  if (a<0)
      a = 0;
  if (a> 31)
    a=31;
//  if (setting.attenuate == a)
//    return;
  setting.attenuate = a;
  dirty = true;
}

void SetStorage(void)
{
  for (int i=0; i<POINTS_COUNT;i++)
    stored_t[i] = actual_t[i];
  setting.show_stored = true;
  trace[TRACE_STORED].enabled = true;
  dirty = true;
}

int GetStorage(void)
{
  return(setting.show_stored);
}

void SetClearStorage(void)
{
  setting.show_stored = false;
  setting.subtract_stored = false;
  trace[TRACE_STORED].enabled = false;
  dirty = true;
}

void SetSubtractStorage(void)
{
  if (!setting.subtract_stored) {
    if (!setting.show_stored)
      SetStorage();
    setting.subtract_stored = true;
  } else {
    setting.subtract_stored = false;
  }
  dirty = true;
}

int GetSubtractStorage(void)
{
  return(setting.subtract_stored);
}

extern float peakLevel;
void SetPowerLevel(int o)
{
  float new_offset = o - peakLevel - setting.attenuate + settingLevelOffset();
  if (o != 100) {
    if (setting.mode == M_HIGH)
      config.high_level_offset = new_offset;
    else if (setting.mode == M_LOW)
      config.low_level_offset = new_offset;
#ifdef __ULTRA__
    else if (setting.mode == M_ULTRA)
      config.low_level_offset = new_offset;
#endif
  }
  else {
    config.low_level_offset = 100;
    config.high_level_offset = 100;
  }
  dirty = true;
}

int settingLevelOffset(void)
{
  if (setting.mode == M_HIGH) {
    if (config.high_level_offset == 100)
      return 0;
    return(config.high_level_offset);
  }
  if (setting.mode == M_LOW) {
    if (config.low_level_offset == 100)
      return 0;
    return(config.low_level_offset);
  }
  return(0);
}

int level_is_calibrated(void)
{
  if (setting.mode == M_HIGH && config.high_level_offset != 100)
    return 1;
  if (setting.mode == M_LOW && config.low_level_offset != 100)
    return 1;
  return(0);
}

void SetRBW(int v)
{
  setting.rbw = v;
  update_rbw();
  dirty = true;
}

int GetRBW(void)
{
  return(setting.rbw);
}

int GetActualRBW(void)
{
  return((int) actual_rbw);
}

#ifdef __ULTRA__
void SetSpur(int v)
{
  setting.spur = v;
  if (setting.spur && actual_rbw > 360)
    SetRBW(300);
  dirty = true;
}
#endif

void set_harmonic(int h)
{
  setting.harmonic = h;
  minFreq = 870000000;
  if (setting.harmonic * 240000000 >  870000000)
    minFreq = setting.harmonic * 240000000;
  maxFreq = 4360000000;
  if (setting.harmonic != 0 && 960000000.0 * setting.harmonic < 4360000000.0)
    maxFreq = ((uint32_t)960000000) * (uint32_t)setting.harmonic;
  set_sweep_frequency(ST_START, (uint32_t) minFreq);
  set_sweep_frequency(ST_STOP, (uint32_t) maxFreq);
}

void SetStepDelay(int d)
{
  setting.step_delay = d;
  dirty = true;
}

void SetAverage(int v)
{
  setting.average = v;
  trace[TRACE_TEMP].enabled = (v != 0);
  dirty = true;
}

int GetAverage(void)
{
  return(setting.average);
}

void ToggleLNA(void)
{
  setting.lna = !setting.lna;
  dirty = true;
}

void toggle_tracking(void)
{
  setting.tracking = !setting.tracking;
  dirty = true;
}

int GetExtraVFO(void)
{
  return(setting.tracking);
}

int GetLNA(void)
{
  return(setting.lna);
}

void ToggleAGC(void)
{
  setting.agc = !setting.agc;
  dirty = true;
}

int GetAGC(void)
{
  return(setting.agc);
}

void SetReflevel(int level)
{
  setting.reflevel = (level / setting.scale) * setting.scale;
  set_trace_refpos(0, NGRIDY - level / get_trace_scale(0));
  set_trace_refpos(1, NGRIDY - level / get_trace_scale(0));
  set_trace_refpos(2, NGRIDY - level / get_trace_scale(0));
  dirty = true;
}

//int GetRefpos(void) {
//  return (NGRIDY - get_trace_refpos(2)) * get_trace_scale(2);
//}

void SetScale(int s) {
  setting.scale = s;
  set_trace_scale(0, s);
  set_trace_scale(1, s);
  set_trace_scale(2, s);
}

//int GetScale(void) {
//  return get_trace_refpos(2);
//}
void SetMode(int m)
{
#ifdef __ULTRA__
  if (m == 6)
    m = M_ULTRA;
#endif
  if (setting.mode == m)
    return;
  reset_settings(m);
}

void apply_settings(void)
{
  if (setting.mode == M_HIGH)
    PE4302_Write_Byte(40);
  else
    PE4302_Write_Byte(setting.attenuate * 2);
#if 0
  if (setting.modulation == MO_NFM ) {
    SI4432_Sel = 1;
    SI4432_Write_Byte(0x7A, 1);  // Use frequency hopping channel width for FM modulation
  } else if (setting.modulation == MO_WFM ) {
    SI4432_Sel = 1;
    SI4432_Write_Byte(0x7A, 10);  // Use frequency hopping channel width for FM modulation
  } else {
    SI4432_Sel = 1;
    SI4432_Write_Byte(0x79, 0);  // IF no FM back to channel 0
  }
#endif
  SetRX(setting.mode);
  SI4432_SetReference(setting.refer);
  update_rbw();
  if (setting.step_delay == 0){
      if (actual_rbw > 90.0)         actualStepDelay =  400;
      else if (actual_rbw > 75.0)    actualStepDelay =  550;
      else if (actual_rbw > 56.0)    actualStepDelay =  650;
      else if (actual_rbw > 37.0)    actualStepDelay =  700;
      else if (actual_rbw > 18.0)    actualStepDelay = 1100;
      else if (actual_rbw >  9.0)    actualStepDelay = 2000;
      else if (actual_rbw >  5.0)    actualStepDelay = 3500;
      else                           actualStepDelay = 6000;
  } else
    actualStepDelay = setting.step_delay;
}

//------------------------------------------


float peakLevel;
float min_level;
uint32_t peakFreq;
int peakIndex;
float temppeakLevel;
int temppeakIndex;

void setupSA(void)
{
  SI4432_Init();
  PE4302_init();
  PE4302_Write_Byte(0);
}

static unsigned long old_freq[4] = { 0, 0, 0, 0 };

void setFreq(int V, unsigned long freq)
{
  if (old_freq[V] != freq) {
    if (V <= 1) {
      SI4432_Sel = V;
      SI4432_Set_Frequency(freq);
#ifdef __ULTRA_SA__
    } else {
      ADF4351_set_frequency(V-2,freq,3);
#endif
    }
    old_freq[V] = freq;
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

void SetSwitchOff(void) {
  SI4432_Write_Byte(0x0b, 0x1d);// Set both switch off
  SI4432_Write_Byte(0x0c, 0x1f);
}

void SetAGCLNA(void) {
  unsigned char v = 0x40;
  if (setting.agc) v |= 0x20;
  if (setting.lna) v |= 0x10;
  SI4432_Write_Byte(0x69, v);
}

void SetRX(int m)
{
switch(m) {
case M_LOW:     // Mixed into 0
#ifdef __ULTRA__
case M_ULTRA:
#endif
    SI4432_Sel = 0;
    SI4432_Receive();
    if (setting.step_atten) {
      SetSwitchTransmit();
    } else {
      SetSwitchReceive();
    }
    SetAGCLNA();

    SI4432_Sel = 1;
    if (setting.tracking_output)
      SetSwitchTransmit();
    else
      SetSwitchOff();
//    SI4432_Receive(); For noise testing only
    SI4432_Transmit(setting.drive);
    // SI4432_SetReference(setting.refer);
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
    if (setting.step_atten) {
      SetSwitchOff();
    } else {
      SetSwitchTransmit();
    }
    SI4432_Transmit(setting.drive);

    SI4432_Sel = 1;
    if (setting.modulation == MO_EXTERNAL) {
      SetSwitchTransmit();  // High input for external LO scuh as tracking output of other tinySA
      SI4432_Receive();
    } else {
      SetSwitchOff();
      SI4432_Transmit(12);                 // Fix LO drive a 10dBm
    }
    break;
case M_GENHIGH: // Direct output from 1
    SI4432_Sel = 0;
    SI4432_Receive();
    SetSwitchReceive();

    SI4432_Sel = 1;
    if (setting.drive < 8) {
      SetSwitchOff();
    } else {
      SetSwitchTransmit();
    }
    SI4432_Transmit(setting.drive);

    break;
  }
  SI4432_Sel = 1;
  SI4432_Write_Byte(0x73, 0);  // Back to nominal offset
  SI4432_Write_Byte(0x74, 0);

}

void update_rbw(void)
{
  setting.vbw = (setting.frequency_step)/1000.0;
  actual_rbw = setting.rbw;
//  float old_rbw = actual_rbw;
  if (actual_rbw == 0)
    actual_rbw = 2*setting.vbw;
  if (actual_rbw < 2.6)
    actual_rbw = 2.6;
  if (actual_rbw > 600)
    actual_rbw = 600;

  SI4432_Sel =  MODE_SELECT(setting.mode);
  actual_rbw = SI4432_SET_RBW(actual_rbw);

  vbwSteps = ((int)(2 * setting.vbw / actual_rbw));

  if (vbwSteps < 1)
    vbwSteps = 1;
  dirty = true;
}

int binary_search_frequency(int f)
{
  int L = 0;
  int R =  (sizeof frequencies)/sizeof(int) - 1;
  int fmin =  f - ((int)actual_rbw ) * 1000;
  int fplus = f + ((int)actual_rbw ) * 1000;
  while (L <= R) {
    int m = (L + R) / 2;
    if ((int)frequencies[m] < fmin)
      L = m + 1;
    else if ((int)frequencies[m] > fplus)
      R = m - 1;
    else
       return m; // index is m
  }
  return -1;
}


#define MAX_MAX 4
int
search_maximum(int m, int center, int span)
{
  center = binary_search_frequency(center);
  if (center < 0)
    return false;
  int from = center - span/2;
  int found = false;
  int to = center + span/2;
  int cur_max = 0;          // Always at least one maximum
  int max_index[4];
  if (from<0)
    from = 0;
  if (to > POINTS_COUNT-1)
    to = POINTS_COUNT-1;
  temppeakIndex = 0;
  temppeakLevel = actual_t[from];
  max_index[cur_max] = from;
  int downslope = true;

  for (int i = from; i <= to; i++) {
    if (downslope) {
      if (temppeakLevel > actual_t[i]) {    // Follow down
        temppeakIndex = i;                  // Latest minimum
        temppeakLevel = actual_t[i];
      } else if (temppeakLevel + setting.noise < actual_t[i]) {    // Local minimum found
        temppeakIndex = i;                          // This is now the latest maximum
        temppeakLevel = actual_t[i];
        downslope = false;
      }
    } else {
      if (temppeakLevel < actual_t[i]) {    // Follow up
        temppeakIndex = i;
        temppeakLevel = actual_t[i];
      } else if (temppeakLevel - setting.noise > actual_t[i]) {    // Local max found

        found = true;
        int j = 0;                                            // Insertion index
        while (j<cur_max && actual_t[max_index[j]] >= temppeakLevel)   // Find where to insert
          j++;
        if (j < MAX_MAX) {                                    // Larger then one of the previous found
          int k = MAX_MAX-1;
          while (k > j) {                                      // Shift to make room for max
            max_index[k] = max_index[k-1];
            //              maxlevel_index[k] = maxlevel_index[k-1];        // Only for debugging
            k--;
          }
          max_index[j] = temppeakIndex;
          //            maxlevel_index[j] = actual_t[temppeakIndex];      // Only for debugging
          if (cur_max < MAX_MAX) {
            cur_max++;
          }
          //STOP_PROFILE
        }
        temppeakIndex = i;            // Latest minimum
        temppeakLevel = actual_t[i];

        downslope = true;
      }
    }
  }
  markers[m].index = max_index[0];
  return found;
}

//static int spur_old_stepdelay = 0;
static const unsigned int spur_IF =            433800000;
static const unsigned int spur_alternate_IF =  434000000;
static const int spur_table[] =
{
 580000,
 961000,
 1600000,
 1837000,           // Real signal
 2755000,           // Real signal
 2760000,
 2961000,
 4933000,
 4960000,
 6961000,
 6980000,
 8267000,
 8961000,
 10000000,
 10960000,
 11600000,
 16960000,
 22960000,
 28960000,

 /*
0.52
6.96
1.84
2.77



 4934
 4960
 8928
 7371



 870000,
   970000,
  1460000,
  1610000,
  1840000,
  2840000,
  2890000,
  2970000,
  4780000,
  4810000,
  4850000,
  4880000,
  8100000,
  8140000,
  10870000,
  14880000,
*/
#ifdef IF_AT_4339
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
#endif
};

int binary_search(int f)
{
  int L = 0;
  int R =  (sizeof spur_table)/sizeof(int) - 1;
  int fmin =  f - ((int)actual_rbw ) * 1000;
  int fplus = f + ((int)actual_rbw ) * 1000;
  while (L <= R) {
    int m = (L + R) / 2;
    if (spur_table[m] < fmin)
      L = m + 1;
    else if (spur_table[m] > fplus)
      R = m - 1;
    else
       return true; // index is m
  }
  return false;
}


int avoid_spur(int f)
{
//  int window = ((int)actual_rbw ) * 1000*2;
//  if (window < 50000)
//    window = 50000;
  if (! setting.mode == M_LOW || setting.frequency_IF != spur_IF || actual_rbw > 300.0)
    return(false);
  return binary_search(f);
#if 0
  f = f + window/2;
  for (unsigned int i = 0; i < (sizeof spur_table)/sizeof(int); i++) {
    if (f/window == (spur_table[i] + window/2)/window) {
//      spur_old_stepdelay = actualStepDelay;
//      actualStepDelay += 4000;
      binary_search(f);
      return true;
    }
  }
  return false;
#endif
}

static int modulation_counter = 0;

char age[POINTS_COUNT];

float perform(bool break_on_operation, int i, uint32_t f, int tracking)
{
  long local_IF;
  if (MODE_HIGH(setting.mode))
    local_IF = 0;
  else
    local_IF = setting.frequency_IF;

  if (i == 0 && dirty) {
    apply_settings();
    scandirty = true;
    dirty = false;
  }
  if (MODE_OUTPUT(setting.mode) && setting.modulation == MO_AM) {
    int p = setting.attenuate * 2 + modulation_counter;
    PE4302_Write_Byte(p);
    if (modulation_counter == 3)
      modulation_counter = 0;
    else
      modulation_counter++;
    chThdSleepMicroseconds(250);
  } else if (MODE_OUTPUT(setting.mode) && (setting.modulation == MO_NFM || setting.modulation == MO_WFM )) {
      SI4432_Sel = 1;
      int offset;
      if (setting.modulation == MO_NFM ) {
        offset = modulation_counter ;
        SI4432_Write_Byte(0x73, (offset & 0xff ));  // Use frequency hopping channel for FM modulation
        SI4432_Write_Byte(0x74, ((offset >> 8) & 0x03 ));  // Use frequency hopping channel for FM modulation
      }
      else {
        offset = modulation_counter * 100;
        SI4432_Write_Byte(0x73, (offset & 0xff ));  // Use frequency hopping channel for FM modulation
        SI4432_Write_Byte(0x74, ((offset >> 8) & 0x03 ));  // Use frequency hopping channel for FM modulation
      }
      if (modulation_counter == 2)
        modulation_counter = -2;
      else
        modulation_counter++;
      chThdSleepMicroseconds(250);
  }
  float RSSI = -150.0;
  int t = 0;
  do {
    int offs = (int)((t * 500  - vbwSteps * 250)  * actual_rbw);
//    if (-offs > (uint32_t)f)         // Ensure lf >0 0
//      offs = -(uint32_t)(f + offs);
    uint32_t lf = (uint32_t)(f + offs);
#ifdef __ULTRA__
    float spur_RSSI = 0;
again:
#endif
    if (setting.mode == M_LOW && tracking) {
      setFreq (0, setting.frequency_IF + lf - reffer_freq[setting.refer]);    // Offset so fundamental of reffer is visible
      local_IF = setting.frequency_IF ;
    } else if (MODE_LOW(setting.mode)) {
      if (setting.mode == M_LOW && !in_selftest && avoid_spur(f)) {
        local_IF = spur_alternate_IF;
      } else {
//        local_IF = setting.frequency_IF ;
      }
      if (setting.mode == M_GENLOW && setting.modulation == MO_EXTERNAL)
        local_IF += lf;
      setFreq (0, local_IF);
#ifdef __ULTRA__
    } else if (setting.mode == M_ULTRA) {
      local_IF  = setting.frequency_IF + (int)(actual_rbw < 350.0 ? setting.spur*300000 : 0 );
      setFreq (0, local_IF);
 //     local_IF  = setting.frequency_IF + (int)(actual_rbw < 300.0?setting.spur * 1000 * actual_rbw:0);
#endif
    } else
      local_IF= 0;
#if 0
    if (lf >11000000 || lf < 9000000) {
      lf = lf;
      break;
    }
#endif
#ifdef __ULTRA__
    if (setting.mode == M_ULTRA) {
//      if (lf > 3406000000 )
//        setFreq (1, local_IF/5 + lf/5);
//      else
      if (lf > 2446000000 )
        setFreq (1, local_IF/5 + lf/5);
      else
//        if (lf > 1486000000)
        setFreq (1, local_IF/3 + lf/3);
//      else
//        setFreq (1, local_IF/2 + lf/2);
    } else
#endif
    {
#ifdef __ULTRA_SA__
//#define IF_1    2550000000
#define IF_2    2025000000

       setFreq (3, IF_2 - 433800000);
       setFreq (2, IF_2 + lf);
       setFreq (1, 433800000);
#else
       if (setting.mode == M_LOW && !setting.tracking && setting.below_IF)
         setFreq (1, local_IF-lf);
       else
         setFreq (1, local_IF+lf);
#endif
    }
    if (MODE_OUTPUT(setting.mode))              // No substepping in output mode
      return(0);
    float signal_path_loss;
#ifdef __ULTRA__
    if (setting.mode == M_ULTRA)
      signal_path_loss = -15;      // Loss in dB, -9.5 for v0.1, -12.5 for v0.2
    else
#endif
      if (setting.mode == M_LOW)
        signal_path_loss = -5.5;      // Loss in dB, -9.5 for v0.1, -12.5 for v0.2
      else
      signal_path_loss = +7;         // Loss in dB (+ is gain)
    float subRSSI = SI4432_RSSI(lf, MODE_SELECT(setting.mode))+settingLevelOffset()+ setting.attenuate - signal_path_loss;
#ifdef __ULTRA__
    if (setting.spur == 1) {                           // First pass
      spur_RSSI = subRSSI;
      setting.spur = -1;
      goto again;                     // Skip all other processing
    } else if (setting.spur == -1) {                            // Second pass
      subRSSI = ( subRSSI < spur_RSSI ? subRSSI : spur_RSSI);  // Minimum of two passes
      setting.spur = 1;
    }
#endif

    if (RSSI < subRSSI)
      RSSI = subRSSI;
    t++;
    if (operation_requested && break_on_operation) // output modes do not step.
      break;         // abort
  } while (t < vbwSteps);
  return(RSSI);
}

#define MAX_MAX 4
int16_t max_index[MAX_MAX];
int16_t cur_max = 0;

// main loop for measurement
static bool sweep(bool break_on_operation)
{
  float RSSI;
  int16_t downslope = true;
  palClearPad(GPIOB, GPIOB_LED);
  temppeakLevel = -150;
  float temp_min_level = 100;
  //  spur_old_stepdelay = 0;
  for (int i = 0; i < sweep_points; i++) {
    RSSI = perform(break_on_operation, i, frequencies[i], setting.tracking);

    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;
    if (MODE_OUTPUT(setting.mode) && setting.modulation == MO_NONE) {
      osalThreadSleepMilliseconds(10);
    }

    if (MODE_INPUT(setting.mode)) {

      temp_t[i] = RSSI;
      if (setting.subtract_stored) {
        RSSI = RSSI - stored_t[i] ;
      }
      //   stored_t[i] = (SI4432_Read_Byte(0x69) & 0x0f) * 3.0 - 90.0; // Display the AGC value in thestored trace
      if (scandirty || setting.average == AV_OFF) {
        actual_t[i] = RSSI;
        age[i] = 0;
      } else {
        switch(setting.average) {
        case AV_MIN:      if (actual_t[i] > RSSI) actual_t[i] = RSSI; break;
        case AV_MAX_HOLD: if (actual_t[i] < RSSI) actual_t[i] = RSSI; break;
        case AV_MAX_DECAY:
          if (actual_t[i] < RSSI) {
            actual_t[i] = RSSI;
            age[i] = 0;
          } else {
            if (age[i] > setting.decay)
              actual_t[i] -= 0.5;
            else
              age[i] += 1;
          }
          break;
        case AV_4:  actual_t[i] = (actual_t[i]*3 + RSSI) / 4.0; break;
        case AV_16: actual_t[i] = (actual_t[i]*15 + RSSI) / 16.0; break;
        }
      }
#if 1
      // START_PROFILE
      if (i == 0) {
        cur_max = 0;          // Always at least one maximum
        temppeakIndex = 0;
        temppeakLevel = actual_t[i];
        max_index[0] = 0;
        downslope = true;
      }
      if (downslope) {
        if (temppeakLevel > actual_t[i]) {    // Follow down
          temppeakIndex = i;                  // Latest minimum
          temppeakLevel = actual_t[i];
        } else if (temppeakLevel + setting.noise < actual_t[i] ) {    // Local minimum found
          temppeakIndex = i;                          // This is now the latest maximum
          temppeakLevel = actual_t[i];
          downslope = false;
        }
      } else {
        if (temppeakLevel < actual_t[i]) {    // Follow up
          temppeakIndex = i;
          temppeakLevel = actual_t[i];
        } else if (actual_t[i] < temppeakLevel - setting.noise) {    // Local max found

          int j = 0;                                            // Insertion index
          while (j<cur_max && actual_t[max_index[j]] >= temppeakLevel)   // Find where to insert
            j++;
          if (j < MAX_MAX) {                                    // Larger then one of the previous found
            int k = MAX_MAX-1;
            while (k > j) {                                      // Shift to make room for max
              max_index[k] = max_index[k-1];
              //              maxlevel_index[k] = maxlevel_index[k-1];        // Only for debugging
              k--;
            }
            max_index[j] = temppeakIndex;
            //            maxlevel_index[j] = actual_t[temppeakIndex];      // Only for debugging
            if (cur_max < MAX_MAX) {
              cur_max++;
            }
            //STOP_PROFILE
          }
          temppeakIndex = i;            // Latest minimum
          temppeakLevel = actual_t[i];

          downslope = true;
        }
      }
    }
#else
    if (frequencies[i] > 1000000) {
      if (temppeakLevel < actual_t[i]) {
        temppeakIndex = i;
        temppeakLevel = actual_t[i];
      }
    }
#endif
    if (temp_min_level > actual_t[i])
      temp_min_level = actual_t[i];

  }
  if (scandirty) {
    scandirty = false;
    draw_cal_status();
  }
  if (!in_selftest && setting.mode == M_LOW && setting.auto_attenuation && max_index[0] > 0) {
    if (actual_t[max_index[0]] - setting.attenuate < - 3*setting.scale && setting.attenuate >= setting.scale) {
      setting.attenuate -= setting.scale;
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    } else if (actual_t[max_index[0]] - setting.attenuate > - 1.5*setting.scale && setting.attenuate <= 30 - setting.scale) {
      setting.attenuate += setting.scale;
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    }
  }
  if (!in_selftest && MODE_INPUT(setting.mode) && setting.auto_reflevel && max_index[0] > 0) {
    if (actual_t[max_index[0]] > setting.reflevel - setting.scale/2) {
      SetReflevel(setting.reflevel + setting.scale);
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    } else if (temp_min_level < setting.reflevel - 9 * setting.scale - 2 && actual_t[max_index[0]] < setting.reflevel -  setting.scale * 3 / 2) {
      SetReflevel(setting.reflevel - setting.scale);
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    } else if (temp_min_level > setting.reflevel - 9 * setting.scale + setting.scale + 2) {
      SetReflevel(setting.reflevel + setting.scale);
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    }
  }
#if 1
  if (MODE_INPUT(setting.mode)) {
    int i = 0;
    int m = 0;
    while (i < cur_max) {                                 // For all maxima found
      while (m < MARKERS_MAX) {
        if (markers[m].enabled && markers[m].mtype & M_TRACKING) {   // Available marker found
          markers[m].index = max_index[i];
          markers[m].frequency = frequencies[markers[m].index];
          m++;
          break;                          // Next maximum
        }
        m++;                              // Try next marker
      }
      i++;
    }
    while (m < MARKERS_MAX) {
      if (markers[m].enabled && markers[m].mtype & M_TRACKING) {    // More available markers found
        markers[m].index = 0;                             // Enabled but no max
        markers[m].frequency = frequencies[markers[m].index];
      }
      m++;                              // Try next marker
    }
#ifdef __MEASURE__
    if (setting.measurement == M_IMD && markers[0].index > 10) {
      markers[1].enabled = search_maximum(1, frequencies[markers[0].index]*2, 8);
      markers[2].enabled = search_maximum(2, frequencies[markers[0].index]*3, 12);
      markers[3].enabled = search_maximum(3, frequencies[markers[0].index]*4, 16);
    } else if (setting.measurement == M_OIP3  && markers[0].index > 10 && markers[1].index > 10) {
      int l = markers[0].index;
      int r = markers[1].index;
      if (r < l) {
        l = markers[1].index;
        r = markers[0].index;
        markers[0].index = l;
        markers[1].index = r;
      }
      uint32_t lf = frequencies[l];
      uint32_t rf = frequencies[r];
      markers[2].enabled = search_maximum(2, lf - (rf - lf), 12);
      markers[3].enabled = search_maximum(3, rf + (rf - lf), 12);
    } else if (setting.measurement == M_PHASE_NOISE  && markers[0].index > 10) {
      markers[1].index =  markers[0].index + (setting.mode == M_LOW ? 290/4 : -290/4);  // Position phase noise marker at requested offset
    } else if (setting.measurement == M_STOP_BAND  && markers[0].index > 10) {
      markers[1].index =  marker_search_left_min(markers[0].index);
      if (markers[1].index < 0) markers[1].index = 0;
      markers[2].index =  marker_search_right_min(markers[0].index);
      if (markers[2].index < 0) markers[1].index = POINTS_COUNT - 1;
    } else if (setting.measurement == M_PASS_BAND  && markers[0].index > 10) {
      int t = markers[0].index;
      float v = actual_t[t];
      while (t > 0 && actual_t[t] > v - 3.0)
        t --;
      if (t > 0)
        markers[1].index = t;
      t = markers[0].index;
      while (t < POINTS_COUNT - 1 && actual_t[t] > v - 3.0)
        t ++;
      if (t < POINTS_COUNT - 1 )
        markers[2].index = t;
    }
#endif
    peakIndex = max_index[0];
    peakLevel = actual_t[peakIndex];
    peakFreq = frequencies[peakIndex];
#else
    int peak_marker = 0;
    markers[peak_marker].enabled = true;
    markers[peak_marker].index = peakIndex;
    markers[peak_marker].frequency = frequencies[markers[peak_marker].index];
#endif
    min_level = temp_min_level;
#if 0                           // Auto ref level setting
    int scale = setting.scale;
    int rp = GetRepos();
    if (scale > 0 && peakLevel > rp && peakLevel - min_level < 8 * scale ) {
      SetReflevel((((int)(peakLevel/scale)) + 1) * scale);
    }
    if (scale > 0 && min_level < rp - 9*scale && peakLevel - min_level < 8 * scale ) {
      int new_rp = (((int)((min_level + 9*scale)/scale)) - 1) * scale;
      if (new_rp < rp)
        SetReflevel(new_rp);
    }

#endif
  }
  //    redraw_marker(peak_marker, FALSE);
  palSetPad(GPIOB, GPIOB_LED);
  return true;
}

//------------------------------- SEARCH ---------------------------------------------

int
marker_search_left_max(int from)
{
  int i;
  int found = -1;
  if (uistat.current_trace == -1)
    return -1;

  int value = actual_t[from];
  for (i = from - 1; i >= 0; i--) {
    int new_value = actual_t[i];
    if (new_value < value) {
      value = new_value;
      found = i;
    } else if (new_value > value + setting.noise )
      break;
  }

  for (; i >= 0; i--) {
    int new_value = actual_t[i];
    if (new_value > value) {
      value = new_value;
      found = i;
    } else if (new_value < value  - setting.noise )
      break;
  }
  return found;
}

int
marker_search_right_max(int from)
{
  int i;
  int found = -1;

  if (uistat.current_trace == -1)
    return -1;
  int value = actual_t[from];
  for (i = from + 1; i < sweep_points; i++) {
    int new_value = actual_t[i];
    if (new_value < value) {    // follow down
      value = new_value;
      found = i;
    } else if (new_value > value + setting.noise) // larger then lowest value + noise
      break;    //  past the minimum
  }
  for (; i < sweep_points; i++) {
    int new_value = actual_t[i];
    if (new_value > value) {    // follow up
      value = new_value;
      found = i;
    } else if (new_value < value - setting.noise)
      break;
  }
  return found;
}

#define MINMAX_DELTA 10


int
marker_search_left_min(int from)
{
  int i;
  int found = from;
  if (uistat.current_trace == -1)
    return -1;

  int value = actual_t[from];
  for (i = from - 1; i >= 0; i--) {
    int new_value = actual_t[i];
    if (new_value > value) {
      value = new_value;        // follow up
//      found = i;
    } else if (new_value < value - MINMAX_DELTA )
      break;  // past the maximum
  }

  for (; i >= 0; i--) {
    int new_value = actual_t[i];
    if (new_value < value) {
      value = new_value;        // follow down
      found = i;
    } else if (new_value > value  + MINMAX_DELTA )
      break;
  }
  return found;
}

int
marker_search_right_min(int from)
{
  int i;
  int found = from;

  if (uistat.current_trace == -1)
    return -1;
  int value = actual_t[from];
  for (i = from + 1; i < sweep_points; i++) {
    int new_value = actual_t[i];
    if (new_value > value) {    // follow up
      value = new_value;
//      found = i;
    } else if (new_value < value - MINMAX_DELTA) // less then largest value - noise
      break;    // past the maximum
  }
  for (; i < sweep_points; i++) {
    int new_value = actual_t[i];
    if (new_value < value) {    // follow down
      value = new_value;
      found = i;
    } else if (new_value > value + MINMAX_DELTA) // larger then smallest value + noise
      break;
  }
  return found;
}





// -------------------------- CAL STATUS ---------------------------------------------
const char *averageText[] = { "OFF", "MIN", "MAX", "MAXD", " A 4", "A 16"};
const char *dBText[] = { "1dB/", "2dB/", "5dB/", "10dB/", "20dB/"};
const int refMHz[] = { 30, 15, 10, 4, 3, 2, 1 };

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

  if (MODE_OUTPUT(setting.mode))     // No cal status during output
    return;
  if (current_menu_is_form() && !in_selftest)
    return;

  ili9341_set_background(DEFAULT_BG_COLOR);

  int yMax = setting.reflevel;
  plot_printf(buf, BLEN, "%ddB", yMax);
  buf[5]=0;
  if (level_is_calibrated()) {
    if (setting.auto_reflevel)
      color = DEFAULT_FG_COLOR;
    else
      color = BRIGHT_COLOR_GREEN;
  }
  else
    color = BRIGHT_COLOR_RED;
  ili9341_set_foreground(color);
  ili9341_drawstring(buf, x, y);

  color = DEFAULT_FG_COLOR;
  ili9341_set_foreground(color);
  y += YSTEP*2;
  plot_printf(buf, BLEN, "%ddB/",(int)setting.scale);
  ili9341_drawstring(buf, x, y);

  if (setting.auto_attenuation)
    color = DEFAULT_FG_COLOR;
  else
    color = BRIGHT_COLOR_GREEN;
  ili9341_set_foreground(color);
  y += YSTEP*2;
  ili9341_drawstring("Attn:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%ddB", setting.attenuate);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  if (setting.average>0) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP*2;
    ili9341_drawstring("Calc:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%s",averageText[setting.average]);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }
#ifdef __ULTRA__
  if (setting.spur) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP*2;
    ili9341_drawstring("Spur:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "ON");
    ili9341_drawstring(buf, x, y);
  }
#endif

  if (setting.rbw)
    color = BRIGHT_COLOR_GREEN;
  else
    color = DEFAULT_FG_COLOR;
  ili9341_set_foreground(color);

  y += YSTEP*2;
  ili9341_drawstring("RBW:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%dkHz", (int)actual_rbw);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  ili9341_set_foreground(DEFAULT_FG_COLOR);
  y += YSTEP*2;
  ili9341_drawstring("VBW:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%dkHz",(int)setting.vbw);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  if (dirty)
    color = BRIGHT_COLOR_RED;
  else if (setting.step_delay)
    color = BRIGHT_COLOR_GREEN;
  else
    color = DEFAULT_FG_COLOR;

    ili9341_set_foreground(color);

  y += YSTEP*2;
  ili9341_drawstring("Scan:", x, y);

  y += YSTEP;
  int32_t t = (int)((2* vbwSteps * sweep_points * ( actualStepDelay / 100) )) /10
#ifdef __ULTRA__
      * (setting.spur ? 2 : 1)
#endif
      ; // in mS
  if (t>1000)
    plot_printf(buf, BLEN, "%dS",(t+500)/1000);
  else
    plot_printf(buf, BLEN, "%dmS",t);

  buf[5]=0;
  ili9341_drawstring(buf, x, y);


  if (setting.refer >= 0) {
    ili9341_set_foreground(BRIGHT_COLOR_RED);
    y += YSTEP*2;
    ili9341_drawstring("Ref:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%dMHz",reffer_freq[setting.refer]/1000000);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  ili9341_set_foreground(BRIGHT_COLOR_GREEN);
  y += YSTEP*2;
  if (MODE_LOW(setting.mode))
      ili9341_drawstring_7x13("M:L", x, y);
  else
    ili9341_drawstring_7x13("M:H", x, y);


  y = HEIGHT-7 + OFFSETY;
  plot_printf(buf, BLEN, "%ddB", (int)(yMax - setting.scale * NGRIDY));
  buf[5]=0;
  if (level_is_calibrated())
    if (setting.auto_reflevel)
      color = DEFAULT_FG_COLOR;
    else
      color = BRIGHT_COLOR_GREEN;
  else
    color = BRIGHT_COLOR_RED;
  ili9341_set_foreground(color);
  ili9341_drawstring(buf, x, y);

}

// -------------------- Self testing -------------------------------------------------

enum {
  TC_SIGNAL, TC_BELOW, TC_ABOVE, TC_FLAT, TC_MEASURE, TC_SET, TC_END,
};

enum {
  TP_SILENT, TPH_SILENT, TP_10MHZ, TP_10MHZEXTRA, TP_10MHZ_SWITCH, TP_30MHZ, TPH_30MHZ
};

#define TEST_COUNT  17

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
 {TC_BELOW,     TP_SILENT,      0.005,  0.01,  0,0,     0},         // 1 Zero Hz leakage
 {TC_BELOW,     TP_SILENT,      0.01,   0.01,  -30,   0,     0},         // 2 Phase noise of zero Hz
 {TC_SIGNAL,    TP_10MHZ,       20,     7,      -37, 30,    -80 },      // 3
 {TC_SIGNAL,    TP_10MHZ,       30,     7,      -32, 30,    -80 },      // 4
 {TC_BELOW,     TP_SILENT,      200,    100,    -70, 0,     0},         // 5  Wide band noise floor low mode
 {TC_BELOW,     TPH_SILENT,     600,    720,    -65, 0,     0},         // 6 Wide band noise floor high mode
 {TC_SIGNAL,    TP_10MHZEXTRA,  10,     8,      -20, 80,    -60 },      // 7 BPF loss and stop band
 {TC_FLAT,      TP_10MHZEXTRA,  10,     4,      -18, 20,    -60},       // 8 BPF pass band flatness
 {TC_BELOW,     TP_30MHZ,       430,    60,     -65, 0,     -75},       // 9 LPF cutoff
 {TC_SIGNAL,    TP_10MHZ_SWITCH,20,     7,      -58, 30,    -90 },      // 10 Switch isolation
 {TC_END,       0,              0,      0,      0,   0,     0},
 {TC_MEASURE,   TP_30MHZ,       30,     7,      -22.5, 30,  -70 },      // 12 Measure power level and noise
 {TC_MEASURE,   TP_30MHZ,       270,    4,      -50, 30,    -75 },       // 13 Measure powerlevel and noise
 {TC_MEASURE,   TPH_30MHZ,      270,    4,      -40, 30,    -65 },       // 14 Calibrate power high mode
 {TC_END,       0,              0,      0,      0,   0,     0},
 {TC_MEASURE,   TP_30MHZ,       30,     1,      -20, 30,    -70 },      // 16 Measure RBW step time
 {TC_END,       0,              0,      0,      0,   0,     0},
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
  (void)i;
  pause_sweep();
#if 0
  if (test_case[i].center < 300)
    setting.mode = M_LOW;
  else
    setting.mode = M_HIGH;
#endif
//  SetAverage(4);
  sweep(false);
//  sweep(false);
//  sweep(false);
//  sweep(false);
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
  int i = -2;
  do {
    i++;
    int xpos = 25 - x0;
    int ypos = 50+i*INFO_SPACING - y0;
    unsigned int color = RGBHEX(0xFFFFFF);
    if (i == -1) {
        plot_printf(self_test_status_buf, sizeof self_test_status_buf, "Self test status:");
    } else if (test_case[i].kind == TC_END) {
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
  } while (test_case[i].kind != TC_END);
}

#define fabs(X) ((X)<0?-(X):(X))

int validate_signal_within(int i, float margin)
{
  test_fail_cause[i] = "Signal level ";
  if (fabs(peakLevel-test_case[i].pass) > 2*margin) {
    return TS_FAIL;
  }
  if (fabs(peakLevel-test_case[i].pass) > margin) {
    return TS_CRITICAL;
  }
  test_fail_cause[i] = "Frequency ";
  if (peakFreq < test_case[i].center * 1000000 - 100000 || test_case[i].center * 1000000 + 100000 < peakFreq )
    return TS_FAIL;
  test_fail_cause[i] = "";
  return TS_PASS;
}

int validate_peak_below(int i, float margin) {
  return(test_case[i].pass - peakLevel > margin);
}

int validate_below(int tc, int from, int to) {
  int status = TS_PASS;
  for (int j = from; j < to; j++) {
    if (actual_t[j] > stored_t[j] - 5)
      status = TS_CRITICAL;
    else if (actual_t[j] > stored_t[j]) {
      status = TS_FAIL;
      break;
    }
  }
  if (status != TS_PASS)
    test_fail_cause[tc] = "Above ";
  return(status);
}

int validate_flatness(int i) {
  volatile int j;
  test_fail_cause[i] = "Passband ";
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
  test_fail_cause[i] = "";
  return(TS_PASS);
}

int validate_above(int tc) {
  int status = TS_PASS;
  for (int j = 0; j < POINTS_COUNT; j++) {
    if (actual_t[j] < stored_t[j] + 5)
      status = TS_CRITICAL;
    else if (actual_t[j] < stored_t[j]) {
      status = TS_FAIL;
      break;
    }
  }
  if (status != TS_PASS)
    test_fail_cause[tc] = "Below ";
  return(status);
}


int test_validate(int i)
{
//  draw_all(TRUE);
  int current_test_status = TS_PASS;
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
  common: current_test_status = validate_signal_within(i, 5.0);
    if (current_test_status == TS_PASS) {            // Validate noise floor
      current_test_status = validate_below(i, 0, POINTS_COUNT/2 - test_case[i].width);
      if (current_test_status == TS_PASS) {
        current_test_status = validate_below(i, POINTS_COUNT/2 + test_case[i].width, POINTS_COUNT);
      }
      if (current_test_status != TS_PASS)
        test_fail_cause[i] = "Stopband ";
    }
    if (current_test_status == TS_PASS && test_case[i].kind == TC_MEASURE)
      test_value = peakLevel;
    else
      test_value = 0;           //   Not valid
  break;
  case TC_ABOVE:   // Validate signal above curve
    current_test_status = validate_above(i);
    break;
  case TC_BELOW:   // Validate signal below curve
    current_test_status = validate_below(i, 0, POINTS_COUNT);
    break;
  case TC_FLAT:   // Validate passband flatness
    current_test_status = validate_flatness(i);
    break;

  }

  // Report status

  if (current_test_status != TS_PASS || test_case[i+1].kind == TC_END)
    test_wait = true;
  test_status[i] = current_test_status;     // Must be set before draw_all() !!!!!!!!
  //  draw_frequencies();
//  draw_cal_status();
  draw_all(TRUE);
  resume_sweep();
  return current_test_status;
}

void test_prepare(int i)
{
  setting.tracking = false; //Default test setup
  setting.step_atten = false;
  SetAttenuation(0);
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
  case TP_10MHZ_SWITCH:
    SetMode(M_LOW);
    set_refer_output(2);
    setting.step_atten = true;
    goto common;
  case TP_10MHZEXTRA:                         // Swept receiver
    SetMode(M_LOW);
    setting.tracking = true; //Sweep BPF
    setting.frequency_IF = 434000000;                // Center on SAW filters
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
  setting.auto_attenuation = false;
  setting.attenuate = 0;
  trace[TRACE_STORED].enabled = true;
  SetReflevel(test_case[i].pass+10);
  set_sweep_frequency(ST_CENTER, (int32_t)(test_case[i].center * 1000000));
  set_sweep_frequency(ST_SPAN, (int32_t)(test_case[i].span * 1000000));
  draw_cal_status();
}

extern void menu_autosettings_cb(int item);
extern float SI4432_force_RBW(int i);

int last_spur = 0;
int add_spur(int f)
{
  for (int i = 0; i < last_spur; i++) {
    if (temp_t[i] == f) {
      stored_t[i] += 1;
      return stored_t[i];
    }
  }
  if (last_spur < 290) {
    temp_t[last_spur] = f;
    stored_t[last_spur++] = 1;
  }
  return 1;
}


void self_test(int test)
{
  if (test ==1) {
    in_selftest = true;               // Spur search
    reset_settings(M_LOW);
    test_prepare(4);
    int f = 400000;           // Start search at 400kHz
    //  int i = 0;                     // Index in spur table (temp_t)
    float p2, p1, p;

#define FREQ_STEP   3000

    SetRBW(FREQ_STEP/1000);
    last_spur = 0;
    for (int j = 0; j < 10; j++) {

      p2 = perform(false, 0, f, false);
      vbwSteps = 1;
      f += FREQ_STEP;
      p1 = perform(false, 1, f, false);
      f += FREQ_STEP;
      shell_printf("\n\rStarting with %4.2f, %4.2f and IF at %d\n\r", p2, p1, setting.frequency_IF);

      f = 400000;
      while (f < 100000000) {
        p = perform(false, 1, f, false);
#define SPUR_DELTA  6
        if ( p2 < p1 - SPUR_DELTA  && p < p1 - SPUR_DELTA) {
          //        temp_t[i++] = f - FREQ_STEP;
          shell_printf("Spur of %4.2f at %d with count %d\n\r", p1,(f - FREQ_STEP)/1000, add_spur(f - FREQ_STEP));
        }
        //    else
        //      shell_printf("%f at %d\n\r", p1,f - FREQ_STEP);
        p2 = p1;
        p1 = p;
        f += FREQ_STEP;
      }
    }
    shell_printf("\n\rTable for IF at %d\n\r", setting.frequency_IF);
    for (int j = 0; j < last_spur; j++) {
      if ((int)stored_t[j] > 1)
        shell_printf("%d, %d\n\r", ((int)temp_t[j])/1000, (int)stored_t[j]);
    }
  } else if (test == 2) {
    // Attenuator test
    in_selftest = true;
    reset_settings(M_LOW);
    int i = 15;       // calibrate attenuator at 30 MHz;
    float reference_peak_level = 0;
    test_prepare(i);
    for (int j= 0; j < 32; j++ ) {
      test_prepare(i);
      SetAttenuation(j);
      float summed_peak_level = 0;
      for (int k=0; k<10; k++) {
        test_acquire(i);                        // Acquire test
        test_validate(i);                       // Validate test
        summed_peak_level += peakLevel;
      }
      peakLevel = summed_peak_level / 10;
      if (j == 0)
        reference_peak_level = peakLevel;
      shell_printf("Target %d, actual %f, delta %f\n\r",j, peakLevel, peakLevel - reference_peak_level);
    }
    return;
  } else if (test == 3) {
    // RBW step time search
    in_selftest = true;
    reset_settings(M_LOW);
    int i = 15;       // calibrate low mode power on 30 MHz;
    test_prepare(i);
    setting.step_delay = 6000;
    for (int j= 0; j < 57; j++ ) {
      setting.step_delay = setting.step_delay * 4/3;
      setting.rbw = SI4432_force_RBW(j);
      shell_printf("RBW = %d, ",setting.rbw);
      test_prepare(i);
      test_acquire(i);                        // Acquire test
      test_validate(i);                       // Validate test
      float saved_peakLevel = peakLevel;
      if (peakLevel < -30) {
        shell_printf("Peak level too low, abort\n\r");
        return;
      }

      shell_printf("Start level = %f, ",peakLevel);
      while (setting.step_delay > 100 && peakLevel > saved_peakLevel - 1) {
        setting.step_delay = setting.step_delay * 3 / 4;
        test_prepare(i);
        //      shell_printf("\n\rRBW = %f",SI4432_force_RBW(j));
        test_acquire(i);                        // Acquire test
        test_validate(i);                       // Validate test
        //      shell_printf(" Step %f, %d",peakLevel, setting.step_delay);
      }
      setting.step_delay = setting.step_delay * 4 / 3;
      shell_printf("End level = %f, step time = %d\n\r",peakLevel, setting.step_delay);
    }
  } else if (test == 0) {
    int old_IF = setting.frequency_IF;
    in_selftest = true;
    menu_autosettings_cb(0);
    for (int i=0; i < TEST_COUNT; i++) {          // All test cases waiting
      if (test_case[i].kind == TC_END)
        break;
      test_status[i] = TS_WAITING;
      test_fail_cause[i] = "";
    }
    show_test_info = TRUE;
    int i=0;
    while (test_case[i].kind != TC_END) {
      setting.frequency_IF = old_IF;
      test_prepare(i);
      test_acquire(i);                        // Acquire test
      test_status[i] = test_validate(i);                       // Validate test
      if (test_status[i] != TS_PASS) {
        wait_user();
      }
      i++;
    }
    ili9341_set_foreground(BRIGHT_COLOR_GREEN);
    ili9341_drawstring_7x13("Self test complete", 50, 200);
    ili9341_drawstring_7x13("Touch screen to continue", 50, 215);
    wait_user();
    ili9341_clear_screen();

    sweep_mode = SWEEP_ENABLE;
    show_test_info = FALSE;
    set_refer_output(0);
    reset_settings(M_LOW);
    in_selftest = false;
  }
}

void reset_calibration(void)
{
  SetPowerLevel(100);
}

#define CALIBRATE_RBWS  1
const int power_rbw [5] = { 100, 300, 30, 10, 3 };

void calibrate(void)
{
#ifdef __CALIBRATE__
  int local_test_status;
  float last_peak_level;
  in_selftest = true;
  SetPowerLevel(100);
  reset_settings(M_LOW);
  int i = 11;       // calibrate low mode power on 30 MHz;
  for (int j= 0; j < CALIBRATE_RBWS; j++ ) {
    SetRBW(power_rbw[j]);
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
//    chThdSleepMilliseconds(1000);
    if (local_test_status != TS_PASS) {
      ili9341_set_foreground(BRIGHT_COLOR_RED);
      ili9341_drawstring_7x13("Calibration failed", 30, 120);
      goto quit;
    } else {
      SetPowerLevel(-22);           // Should be -22.5dBm
      chThdSleepMilliseconds(1000);
    }
  }
  i = 12;           // Measure 270MHz in low mode
  SetRBW(100);
  test_prepare(i);
  test_acquire(i);                        // Acquire test
  last_peak_level = peakLevel;
  local_test_status = test_validate(i);                       // Validate test
  chThdSleepMilliseconds(1000);

  config.high_level_offset = 0;           /// Preliminary setting

  i = 13;           // Calibrate 270MHz in high mode
  for (int j = 0; j < CALIBRATE_RBWS; j++) {
    SetRBW(power_rbw[j]);
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
//    if (local_test_status != TS_PASS) {                       // Do not validate due to variations in SI4432
//      ili9341_set_foreground(BRIGHT_COLOR_RED);
//      ili9341_drawstring_7x13("Calibration failed", 30, 120);
//      goto quit;
//    } else
      SetPowerLevel(last_peak_level);
      chThdSleepMilliseconds(1000);
  }
  ili9341_set_foreground(BRIGHT_COLOR_GREEN);
  ili9341_drawstring_7x13("Calibration complete", 30, 120);
quit:
  ili9341_drawstring_7x13("Touch screen to continue", 30, 140);
  wait_user();
  ili9341_clear_screen();

  in_selftest = false;
  sweep_mode = SWEEP_ENABLE;
  set_refer_output(0);
  SetMode(M_LOW);
#endif
}


