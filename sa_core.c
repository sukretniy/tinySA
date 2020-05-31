#include "SI4432.h"		// comment out for simulation

int dirty = true;
int scandirty = true;

extern int actualStepDelay;

setting_t setting;
uint32_t frequencies[POINTS_COUNT];

float actual_rbw = 0;
int vbwSteps = 1;
float minFreq = 0;
float maxFreq = 520000000;

//int setting.refer = -1;  // Off by default
const int reffer_freq[] = {30000000, 15000000, 10000000, 4000000, 3000000, 2000000, 1000000};

int in_selftest = false;

void reset_settings(int m)
{
  setting.mode = m;
  set_scale(10);
  set_reflevel(-10);
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
  setting.offset = 0.0;
  setting.trigger = T_AUTO;
  setting.trigger_level = -150.0;
  setting.linearity_step = 0;
  trace[TRACE_STORED].enabled = false;
  trace[TRACE_TEMP].enabled = false;
  setting.refer = -1;
#ifdef __SPUR__
  setting.spur = 0;
#endif
  switch(m) {
  case M_LOW:
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_START, (uint32_t) 0);
    set_sweep_frequency(ST_STOP, (uint32_t) 350000000);
    setting.attenuate = 30;
    break;
#ifdef __ULTRA__
  case M_ULTRA:
    minFreq = 674000000;
    maxFreq = 4300000000;
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
  if (m == M_LINEARITY) {
    trace[TRACE_STORED].enabled = true;
    for (int j = 0; j < setting._sweep_points; j++)
      stored_t[j] = -150;
    setting.linearity_step = 0;
    setting.attenuate = 29;
    setting.auto_attenuation = false;
  }
  dirty = true;
}
void set_drive(int d)
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

void set_modulation(int m)
{
  setting.modulation = m;
  dirty = true;
}

void set_repeat(int r)
{
  if (r > 0 && r <= 1000) {
    setting.repeat = r;
    dirty = true;
  }
}

void set_IF(int f)
{
  setting.frequency_IF = f;
  dirty = true;
}

void set_unit(int u)
{
  if (UNIT_IS_LINEAR(setting.unit) && !UNIT_IS_LINEAR(u)) {
    set_scale(10);
  }
  setting.unit = u;
  dirty = true;
}

int GetMode(void)
{
  return(setting.mode);
  dirty = true;
}


#define POWER_STEP  0           // Should be 5 dB but appearently it is lower
#define POWER_OFFSET    15
#define SWITCH_ATTENUATION  30


void set_auto_attenuation(void)
{
  setting.auto_attenuation = true;
  setting.attenuate = 30;
}

void set_auto_reflevel(int v)
{
  setting.auto_reflevel = v;
}

int get_attenuation(void)
{
  if (setting.mode == M_GENLOW) {
    if (setting.step_atten)
      return ( -(POWER_OFFSET + setting.attenuate - (setting.step_atten-1)*POWER_STEP + SWITCH_ATTENUATION));
    else
      return ( -POWER_OFFSET - setting.attenuate + (setting.drive & 7) * 3);
  }
  return(setting.attenuate);
}

void set_attenuation(int a)
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
    setting.auto_attenuation = false;
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

void set_storage(void)
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

void set_clear_storage(void)
{
  setting.show_stored = false;
  setting.subtract_stored = false;
  trace[TRACE_STORED].enabled = false;
  dirty = true;
}

void set_subtract_storage(void)
{
  if (!setting.subtract_stored) {
    if (!setting.show_stored)
      set_storage();
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
void set_actual_power(float o)
{
  float new_offset = o - peakLevel + get_level_offset();
  if (o == 100) new_offset = 0;
  if (setting.mode == M_HIGH) {
    config.high_level_offset = new_offset;
  } else if (setting.mode == M_LOW) {
    config.low_level_offset = new_offset;
#ifdef __ULTRA__
  } else if (setting.mode == M_ULTRA) {
    config.low_level_offset = new_offset;
#endif
  }
  dirty = true;
}

int get_level_offset(void)
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

void set_RBW(int v)
{
  setting.rbw = v;
  update_rbw();
  dirty = true;
}

int GetRBW(void)
{
  return(setting.rbw);
}

int get_actual_RBW(void)
{
  return((int) actual_rbw);
}

#ifdef __SPUR__
void SetSpur(int v)
{
  setting.spur = v;
  if (setting.spur && actual_rbw > 360)
    set_RBW(300);
  dirty = true;
}
#endif

void set_harmonic(int h)
{
  setting.harmonic = h;
  minFreq = 684000000.0;
  if (setting.harmonic * 240000000+434000000 >  minFreq)
    minFreq = setting.harmonic * 240000000.0+434000000.0;
  maxFreq = 4360000000;
  if (setting.harmonic != 0 && (960000000.0 * setting.harmonic + 434000000.0 )< 4360000000.0)
    maxFreq = (960000000.0 * setting.harmonic + 434000000.0 );
  set_sweep_frequency(ST_START, (uint32_t) minFreq);
  set_sweep_frequency(ST_STOP, (uint32_t) maxFreq);
}

void set_step_delay(int d)
{
  setting.step_delay = d;
  dirty = true;
}

void set_average(int v)
{
  setting.average = v;
  trace[TRACE_TEMP].enabled = (v != 0);
  dirty = true;
}

int GetAverage(void)
{
  return(setting.average);
}

void toggle_LNA(void)
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

void toggle_AGC(void)
{
  setting.agc = !setting.agc;
  dirty = true;
}

int GetAGC(void)
{
  return(setting.agc);
}

void set_reflevel(float level)
{
  setting.reflevel = (level / setting.scale) * setting.scale;
  set_trace_refpos(0, NGRIDY - level / get_trace_scale(0));
  set_trace_refpos(1, NGRIDY - level / get_trace_scale(0));
  set_trace_refpos(2, NGRIDY - level / get_trace_scale(0));
  dirty = true;
}


void set_offset(float offset)
{
  setting.offset = offset;
  dirty = true;
}

void set_trigger_level(float trigger_level)
{
  setting.trigger_level = trigger_level;
  if (setting.trigger != T_AUTO) {
    for (int j = 0; j < setting._sweep_points; j++)
      stored_t[j] = trigger_level;
  }
  dirty = true;
}

void set_trigger(int trigger)
{
  setting.trigger = trigger;
  if (trigger == T_AUTO) {
    trace[TRACE_STORED].enabled = false;
  } else {
    for (int j = 0; j < setting._sweep_points; j++)
      stored_t[j] = setting.trigger_level;
    trace[TRACE_STORED].enabled = true;
  }
  sweep_mode = SWEEP_ENABLE;
  dirty = true;
}


//int GetRefpos(void) {
//  return (NGRIDY - get_trace_refpos(2)) * get_trace_scale(2);
//}

void set_scale(float s) {
  setting.scale = s;
  if (UNIT_IS_LINEAR(setting.unit)) {   // Bottom always at zero
    set_reflevel(NGRIDY * s);
  }
  set_trace_scale(0, s);
  set_trace_scale(1, s);
  set_trace_scale(2, s);
}

//int GetScale(void) {
//  return get_trace_refpos(2);
//}
void set_mode(int m)
{
#ifdef __ULTRA__
  if (m == 6)
    m = M_ULTRA;
#endif
  dirty = true;
  if (setting.mode == m)
    return;
  reset_settings(m);
}

void apply_settings(void)
{
  if (setting.mode == M_HIGH)
    PE4302_Write_Byte(40);  // Ensure defined input impedance of low port when using high input mode (power calibration)
  else
    PE4302_Write_Byte(setting.attenuate * 2);
  if (setting.modulation == MO_NONE) {
    SI4432_Write_Byte(0x73, 0);  // Back to nominal offset
    SI4432_Write_Byte(0x74, 0);
  }
  set_switches(setting.mode);
  SI4432_SetReference(setting.refer);
  update_rbw();
  if (setting.frequency_step == 0.0) {
    if (setting.step_delay <= 1)
      actualStepDelay = 0;
    else
      actualStepDelay = setting.step_delay;
  } else if (setting.step_delay < 2){
    if (actual_rbw >= 191.0)        actualStepDelay =  280;
    else if (actual_rbw >= 142.0)   actualStepDelay =  350;
    else if (actual_rbw >= 75.0)    actualStepDelay =  450;
    else if (actual_rbw >= 56.0)    actualStepDelay =  650;
    else if (actual_rbw >= 37.0)    actualStepDelay =  700;
    else if (actual_rbw >= 18.0)    actualStepDelay = 1100;
    else if (actual_rbw >=  9.0)    actualStepDelay = 1700;
    else if (actual_rbw >=  5.0)    actualStepDelay = 3300;
    else                           actualStepDelay = 6400;
    if (setting.step_delay == 1)
      actualStepDelay *= 2;
  } else
    actualStepDelay = setting.step_delay;
}

//------------------------------------------
#if 0
#define CORRECTION_POINTS  10

static const uint32_t correction_frequency[CORRECTION_POINTS] =
{ 100000, 200000, 400000, 1000000, 2000000, 50000000, 100000000, 200000000, 300000000, 350000000 };

static const float correction_value[CORRECTION_POINTS] =
{ +4.0, +2.0, +1.5, +0.5, 0.0, 0.0, +1.0, +1.0, +2.5, +5.0 };
#endif

float get_frequency_correction(uint32_t f)
{
  if (!(setting.mode == M_LOW))
    return(0.0);
  int i = 0;
  while (f > config.correction_frequency[i] && i < CORRECTION_POINTS)
    i++;
  if (i >= CORRECTION_POINTS)
    return(config.correction_value[CORRECTION_POINTS-1]);
  if (i == 0)
    return(config.correction_value[0]);
  f = f - config.correction_frequency[i-1];
  uint32_t m = config.correction_frequency[i] - config.correction_frequency[i-1] ;
  float cv = config.correction_value[i-1] + (config.correction_value[i] - config.correction_value[i-1]) * (float)f / (float)m;
  return(cv);
}


float peakLevel;
float min_level;
uint32_t peakFreq;
int peakIndex;
float temppeakLevel;
int temppeakIndex;

void setupSA(void)
{
  SI4432_Init();
  SI4432_Sel = 1;
  SI4432_Receive();

  SI4432_Transmit(0);
  PE4302_init();
  PE4302_Write_Byte(0);
}

static unsigned long old_freq[4] = { 0, 0, 0, 0 };

void set_freq(int V, unsigned long freq)
{
  if (old_freq[V] != freq) {
    if (V <= 1) {
      SI4432_Sel = V;
      if (freq < 240000000 || freq > 960000000) {
        old_freq[V] = freq + 1;
        return;
      }
      SI4432_Set_Frequency(freq);
#ifdef __ULTRA_SA__
    } else {
      ADF4351_set_frequency(V-2,freq,3);
#endif
    }
    old_freq[V] = freq;
  }
}

void set_switch_transmit(void) {
  SI4432_Write_Byte(0x0b, 0x1f);// Set switch to transmit
  SI4432_Write_Byte(0x0c, 0x1d);
}

void set_switch_receive(void) {
  SI4432_Write_Byte(0x0b, 0x1d);// Set switch to receive
  SI4432_Write_Byte(0x0c, 0x1f);
}

void set_switch_off(void) {
  SI4432_Write_Byte(0x0b, 0x1d);// Set both switch off
  SI4432_Write_Byte(0x0c, 0x1f);
}

void set_AGC_LNA(void) {
  unsigned char v = 0x40;
  if (setting.agc) v |= 0x20;
  if (setting.lna) v |= 0x10;
  SI4432_Write_Byte(0x69, v);
}

void set_switches(int m)
{
switch(m) {
case M_LOW:     // Mixed into 0
#ifdef __ULTRA__
case M_ULTRA:
#endif
    SI4432_Sel = 0;
    SI4432_Receive();
    if (setting.step_atten) {
      set_switch_transmit();
    } else {
      set_switch_receive();
    }
    set_AGC_LNA();

    SI4432_Sel = 1;
    if (setting.tracking_output)
      set_switch_transmit();
    else
      set_switch_off();
//    SI4432_Receive(); For noise testing only
    SI4432_Transmit(setting.drive);
    // SI4432_SetReference(setting.refer);
    break;
case M_HIGH:    // Direct into 1
    // SI4432_SetReference(-1); // Stop reference output
    SI4432_Sel = 0; // both as receiver to avoid spurs
    set_switch_receive();
    SI4432_Receive();

    SI4432_Sel = 1;
    SI4432_Receive();
    set_switch_receive();
    set_AGC_LNA();

    break;
case M_GENLOW:  // Mixed output from 0
    SI4432_Sel = 0;
    if (setting.step_atten) {
      set_switch_off();
    } else {
      set_switch_transmit();
    }
    SI4432_Transmit(setting.drive);

    SI4432_Sel = 1;
    if (setting.modulation == MO_EXTERNAL) {
      set_switch_transmit();  // High input for external LO scuh as tracking output of other tinySA
      SI4432_Receive();
    } else {
      set_switch_off();
      SI4432_Transmit(12);                 // Fix LO drive a 10dBm
    }
    break;
case M_GENHIGH: // Direct output from 1
    SI4432_Sel = 0;
    SI4432_Receive();
    set_switch_receive();

    SI4432_Sel = 1;
    if (setting.drive < 8) {
      set_switch_off();
    } else {
      set_switch_transmit();
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
  if (setting.frequency_step > 0) {
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
  } else {
    actual_rbw = setting.rbw;
    if (actual_rbw == 0)
      actual_rbw = 600;
    if (actual_rbw < 2.6)
      actual_rbw = 2.6;
    if (actual_rbw > 600)
      actual_rbw = 600;

    SI4432_Sel =  MODE_SELECT(setting.mode);
    actual_rbw = SI4432_SET_RBW(actual_rbw);
    setting.vbw = actual_rbw;
    vbwSteps = 1;
  }
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
  if (to > setting._sweep_points-1)
    to = setting._sweep_points-1;
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
static const unsigned int spur_alternate_IF =  433900000;
static const int spur_table[] =
{
 580000,            // 433.8 MHz table
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
 29800000,
 38105000,
 49500000,
#ifdef IF_AT_4339
  780000,           // 433.9MHz table
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
}

static int modulation_counter = 0;

static const int am_modulation[5] =  { 4,0,1,5,7 };
static const int nfm_modulation[5] = { 0, 2, 1, -1, -2};
static const int wfm_modulation[5] = { 0, 190, 118, -118, -190 };

char age[POINTS_COUNT];

float perform(bool break_on_operation, int i, uint32_t f, int tracking)
{
  long local_IF;
  if (MODE_HIGH(setting.mode))
    local_IF = 0;
  else
    local_IF = setting.frequency_IF;

  if (i == 0 && dirty) {                                                        // SCan initiation
    apply_settings();
    scandirty = true;
    dirty = false;
  }

  if (MODE_OUTPUT(setting.mode) && setting.modulation == MO_AM) {               // AM modulation
    int p = setting.attenuate * 2 + am_modulation[modulation_counter];
    PE4302_Write_Byte(p);
    if (modulation_counter == 4) {  // 3dB modulation depth
      modulation_counter = 0;
    } else {
      modulation_counter++;
    }
    my_microsecond_delay(200);
//    chThdSleepMicroseconds(200);

  } else if (MODE_OUTPUT(setting.mode) && (setting.modulation == MO_NFM || setting.modulation == MO_WFM )) { //FM modulation
      SI4432_Sel = 1;
      int offset;
      if (setting.modulation == MO_NFM ) {
        offset = nfm_modulation[modulation_counter] ;
        SI4432_Write_Byte(0x73, (offset & 0xff ));  // Use frequency hopping channel for FM modulation
        SI4432_Write_Byte(0x74, ((offset >> 8) & 0x03 ));  // Use frequency hopping channel for FM modulation
      }
      else {
        offset = wfm_modulation[modulation_counter] ;
        SI4432_Write_Byte(0x73, (offset & 0xff ));  // Use frequency hopping channel for FM modulation
        SI4432_Write_Byte(0x74, ((offset >> 8) & 0x03 ));  // Use frequency hopping channel for FM modulation
      }
      if (modulation_counter == 4)
        modulation_counter = 0;
      else
        modulation_counter++;
      my_microsecond_delay(200);
//      chThdSleepMicroseconds(200);
  }

  float RSSI = -150.0;
  int t = 0;
  do {           // ------------- Acquisition loop ----------
    int offs;
    if (vbwSteps & 1) { // Uneven steps, center
      offs = (t - (vbwSteps >> 1)) * 500;
    } else {            // Even, shift half step
      offs = (t - (vbwSteps >> 1)) * 500 + 250;
    }
    offs = (int)(offs * actual_rbw);
    uint32_t lf = (uint32_t)(f + offs);
#ifdef __SPUR__
    float spur_RSSI = 0;
again:
#endif
    if (setting.mode == M_LOW && tracking) {                                // Measure BPF
      set_freq (0, setting.frequency_IF + lf - reffer_freq[setting.refer]);    // Offset so fundamental of reffer is visible
      local_IF = setting.frequency_IF ;
    } else if (MODE_LOW(setting.mode)) {
      if (setting.mode == M_LOW && !in_selftest && avoid_spur(f)) {
        local_IF = spur_alternate_IF;
#ifdef __SPUR__
      } else if (setting.mode== M_LOW && setting.spur){
        if (lf > 150000000) // if above 150MHz use IF shift
          local_IF  = setting.frequency_IF + (int)(actual_rbw < 350.0 ? setting.spur*300000 : 0 );
        else {              // else low/above IF
          local_IF = setting.frequency_IF;
          if (setting.spur == 1)
            setting.below_IF = true;
          else
            setting.below_IF = false;
        }
#endif
      } else {
//        local_IF = setting.frequency_IF ;
      }
      if (setting.mode == M_GENLOW && setting.modulation == MO_EXTERNAL)    // LO input via high port
        local_IF += lf;
      set_freq (0, local_IF);
#ifdef __ULTRA__
    } else if (setting.mode == M_ULTRA) {               // No above/below IF mode in Ultra
      local_IF  = setting.frequency_IF + (int)(actual_rbw < 350.0 ? setting.spur*300000 : 0 );
      set_freq (0, local_IF);
 //     local_IF  = setting.frequency_IF + (int)(actual_rbw < 300.0?setting.spur * 1000 * actual_rbw:0);
#endif
    } else          // This must be high mode
      local_IF= 0;
#ifdef __ULTRA__
    if (setting.mode == M_ULTRA) {      // Set LO to correct harmonic in Ultra mode
//      if (lf > 3406000000 )
//        setFreq (1, local_IF/5 + lf/5);
//      else
      if (setting.spur != 1) {  // Left of tables
        if (lf > 3250000000 )
          set_freq (1, lf/5 - local_IF/5);
        if (lf > 1250000000 )
          set_freq (1, lf/3 - local_IF/3);
        else
          set_freq (1,  lf - local_IF);

      } else {              // Right of tables
        if (lf >= 2350000000)
          set_freq (1,  lf/5 + local_IF/5);
        else
          set_freq (1, lf/3 + local_IF/3);
      }
    } else
#endif
    {                                           // Else set LO ('s)
#ifdef __ULTRA_SA__
//#define IF_1    2550000000
#define IF_2    2025000000                      // First IF in Ultra SA mode

       set_freq (2, IF_2 + lf);                 // Scanning LO up to IF2
       set_freq (3, IF_2 - 433800000);          // Down from IF2 to fixed second IF in Ultra SA mode
       set_freq (1, 433800000);                 // Second IF fixe in Ultra SA mode
#else
       if (setting.mode == M_LOW && !setting.tracking && setting.below_IF)
         set_freq (1, local_IF-lf);
       else
         set_freq (1, local_IF+lf);
#endif
    }
    if (MODE_OUTPUT(setting.mode))              // No substepping and no RSSI in output mode
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

    int wait_for_trigger = false;
    int old_actual_step_delay = actualStepDelay;
    if (i == 0 && setting.frequency_step == 0 && setting.trigger != T_AUTO) { // prepare for wait for trigger to happen
      wait_for_trigger = true;
      actualStepDelay = 0;      // fastest possible in zero span trigger mode
    }
    float subRSSI;
    float correct_RSSI = get_level_offset()+ setting.attenuate - signal_path_loss - setting.offset + get_frequency_correction(f);
   wait:
    subRSSI = SI4432_RSSI(lf, MODE_SELECT(setting.mode)) + correct_RSSI ;
    if (wait_for_trigger) { // wait for trigger to happen
      if (operation_requested && break_on_operation)
        break;         // abort
      if (subRSSI < setting.trigger_level)
        goto wait;
      actualStepDelay = old_actual_step_delay; // Trigger happened, restore step delay
      if (setting.trigger == T_SINGLE)
        pause_sweep();                    // Trigger once so pause after this sweep has completed!!!!!!!
    }

#ifdef __SPUR__
    if (setting.spur == 1) {                                     // If first spur pass
      spur_RSSI = subRSSI;
      setting.spur = -1;
      goto again;                                                // Skip all other processing
    } else if (setting.spur == -1) {                            // If second  spur pass
      subRSSI = ( subRSSI < spur_RSSI ? subRSSI : spur_RSSI);  // Take minimum of two
      setting.spur = 1;
    }
#endif

    if (RSSI < subRSSI)                                     // Take max during subscanning
      RSSI = subRSSI;
    t++;
    if (operation_requested && break_on_operation)       // break subscanning if requested
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
  int16_t downslope;
//  if (setting.mode== -1)
//    return;
//  START_PROFILE;
again:
  downslope = true;
  palClearPad(GPIOB, GPIOB_LED);
  temppeakLevel = -150;
  float temp_min_level = 100;
  //  spur_old_stepdelay = 0;
  int repeats = 1;
  if (MODE_OUTPUT(setting.mode) && setting.modulation != MO_NONE) {
    repeats = 1000; // to avoid interrupting the tone during UI processing
    modulation_counter = 0;
  }
  while (repeats--) {
  for (int i = 0; i < sweep_points; i++) {

    RSSI = perform(break_on_operation, i, frequencies[i], setting.tracking);

    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;
    if (MODE_OUTPUT(setting.mode)) {
      if (setting.modulation == MO_NONE) {
        osalThreadSleepMilliseconds(10);              // Slow down sweep in output mode
      }
      continue;             // Skip all other processing
    }
    if (MODE_INPUT(setting.mode)) {

      temp_t[i] = RSSI;
      if (setting.subtract_stored) {
        RSSI = RSSI - stored_t[i] ;
      }
//         stored_t[i] = (SI4432_Read_Byte(0x69) & 0x0f) * 3.0 - 90.0; // Display the AGC value in the stored trace
      if (scandirty || setting.average == AV_OFF) {             // Level calculations
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
      if (i == 0) {                                          // Prepare peak finding
        cur_max = 0;          // Always at least one maximum
        temppeakIndex = 0;
        temppeakLevel = actual_t[i];
        max_index[0] = 0;
        downslope = true;
      }
      if (downslope) {                               // If in down slope peak finding
        if (temppeakLevel > actual_t[i]) {           // Follow down
          temppeakIndex = i;                         // Latest minimum
          temppeakLevel = actual_t[i];
        } else if (temppeakLevel + setting.noise < actual_t[i] ) {    // Local minimum found
          temppeakIndex = i;                         // This is now the latest maximum
          temppeakLevel = actual_t[i];
          downslope = false;
        }
      } else {                                      // up slope peak finding
        if (temppeakLevel < actual_t[i]) {    // Follow up
          temppeakIndex = i;
          temppeakLevel = actual_t[i];
        } else if (actual_t[i] < temppeakLevel - setting.noise) {    // Local max found


          int j = 0;                                            // Insert max in sorted table
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
                                                              // Insert done
          temppeakIndex = i;            // Latest minimum
          temppeakLevel = actual_t[i];

          downslope = true;
        }
      }
    }                   // end of peak finding
#else
    if (frequencies[i] > 1000000) {
      if (temppeakLevel < actual_t[i]) {
        temppeakIndex = i;
        temppeakLevel = actual_t[i];
      }
    }
#endif
    if (temp_min_level > actual_t[i])   // Remember minimum
      temp_min_level = actual_t[i];

  }

  if (setting.trigger != T_AUTO && setting.frequency_step > 0) {    // Trigger active
    if (actual_t[max_index[0]] < setting.trigger_level) {
      goto again;
    } else {
      if (setting.trigger == T_SINGLE)
        pause_sweep();                    // Stop scanning after completing this sweep if above trigger
    }
    scandirty = true;                // To show trigger happened
  }

  if (scandirty) {
    scandirty = false;
    draw_cal_status();
  }

  if (!in_selftest && setting.mode == M_LOW && setting.auto_attenuation && max_index[0] > 0) {  // Auto attenuate
    if (actual_t[max_index[0]] - setting.attenuate < - 30 && setting.attenuate >= 10) {
      setting.attenuate -= 10;
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    } else if (actual_t[max_index[0]] - setting.attenuate > - 15 && setting.attenuate <= 20) {
      setting.attenuate += 10;
      redraw_request |= REDRAW_CAL_STATUS;
      dirty = true;                               // Must be  above if(scandirty!!!!!)
    }
  }
  if (!in_selftest && MODE_INPUT(setting.mode) && setting.auto_reflevel && max_index[0] > 0) {  // Auto reflevel
    if (UNIT_IS_LINEAR(setting.unit)) {            // Linear scales can not have negative values
      float t = value(actual_t[max_index[0]]);
      if (t < setting.reflevel / 2 || t> setting.reflevel) {
        float m = 1;
        t = t * 1.2;
        while (t > 10) { m *= 10; t/=10; }
        while (t < 1)  { m /= 10; t*=10; }
        t = round(t);
        set_scale(t*m / NGRIDY);
        set_reflevel(t*m);
      }
    } else {
      if (value(actual_t[max_index[0]]) > setting.reflevel - setting.scale/2) {
        set_reflevel(setting.reflevel + setting.scale);
        redraw_request |= REDRAW_CAL_STATUS;
        dirty = true;                               // Must be  above if(scandirty!!!!!)
      } else if (temp_min_level < setting.reflevel - 10.1 * setting.scale && value(actual_t[max_index[0]]) < setting.reflevel -  setting.scale * 1.5) {
        set_reflevel(setting.reflevel - setting.scale);
        redraw_request |= REDRAW_CAL_STATUS;
        dirty = true;                               // Must be  above if(scandirty!!!!!)
      } else if (temp_min_level > setting.reflevel - 8.8 * setting.scale) {
        set_reflevel(setting.reflevel + setting.scale);
        redraw_request |= REDRAW_CAL_STATUS;
        dirty = true;                               // Must be  above if(scandirty!!!!!)
      }
    }
  }
#if 1
  if (MODE_INPUT(setting.mode)) {               // Assign maxima found to tracking markers
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
    while (m < MARKERS_MAX) {                  // Insufficient maxima found
      if (markers[m].enabled && markers[m].mtype & M_TRACKING) {    // More available markers found
        markers[m].index = 0;                             // Enabled but no max so set to left most frequency
        markers[m].frequency = frequencies[markers[m].index];
      }
      m++;                              // Try next marker
    }
#ifdef __MEASURE__
    if (setting.measurement == M_IMD && markers[0].index > 10) {                    // IMD measurement
      markers[1].enabled = search_maximum(1, frequencies[markers[0].index]*2, 8);
      markers[2].enabled = search_maximum(2, frequencies[markers[0].index]*3, 12);
      markers[3].enabled = search_maximum(3, frequencies[markers[0].index]*4, 16);
    } else if (setting.measurement == M_OIP3  && markers[0].index > 10 && markers[1].index > 10) { // IOP measurement
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
    } else if (setting.measurement == M_PHASE_NOISE  && markers[0].index > 10) {    // Phase noise measurement
      markers[1].index =  markers[0].index + (setting.mode == M_LOW ? 290/4 : -290/4);  // Position phase noise marker at requested offset
    } else if (setting.measurement == M_STOP_BAND  && markers[0].index > 10) {      // Stop band measurement
      markers[1].index =  marker_search_left_min(markers[0].index);
      if (markers[1].index < 0) markers[1].index = 0;
      markers[2].index =  marker_search_right_min(markers[0].index);
      if (markers[2].index < 0) markers[1].index = setting._sweep_points - 1;
    } else if (setting.measurement == M_PASS_BAND  && markers[0].index > 10) {      // Pass band measurement
      int t = markers[0].index;
      float v = actual_t[t];
      while (t > 0 && actual_t[t] > v - 3.0)                                        // Find left -3dB point
        t --;
      if (t > 0)
        markers[1].index = t;
      t = markers[0].index;
      while (t < setting._sweep_points - 1 && actual_t[t] > v - 3.0)                // find right -3dB point
        t ++;
      if (t < setting._sweep_points - 1 )
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
  }
  }
  if (setting.measurement == M_LINEARITY && setting.linearity_step < setting._sweep_points) {
    setting.attenuate = 29 - setting.linearity_step * 30 / 290;
    dirty = true;
    stored_t[setting.linearity_step] = peakLevel;
    setting.linearity_step++;
  }

  //    redraw_marker(peak_marker, FALSE);
//  STOP_PROFILE;
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

float my_round(float v)
{
  float m = 1;
  int sign = 1;
  if (v < 0) {
    sign = -1;
    v = -v;
  }
  while (v < 100) {
    v = v * 10;
    m = m / 10;
  }
  while (v > 1000) {
    v = v / 10;
    m = m * 10;
  }
  v = (int)(v+0.5);
  v = v * m;
  if (sign == -1) {
    v = -v;
  }
  return v;
}

const char *unit_string[] = { "dBm", "dBmV", "dBuV", "mV", "uV", "mW", "uW" };

void draw_cal_status(void)
{
#define BLEN    10
  char buf[BLEN];
#define YSTEP   8
  int x = 0;
  int y = OFFSETY;
  unsigned int color;
  int rounding = false;
  if (!UNIT_IS_LINEAR(setting.unit))
    rounding  = true;
  const char *unit = unit_string[setting.unit];


#define XSTEP   40

  ili9341_fill(x, y, OFFSETX, HEIGHT, 0x0000);

  if (MODE_OUTPUT(setting.mode))     // No cal status during output
    return;
//  if (current_menu_is_form() && !in_selftest)
//    return;

  ili9341_set_background(DEFAULT_BG_COLOR);

  float yMax = setting.reflevel;
  if (rounding)
    plot_printf(buf, BLEN, "%d", (int)yMax);
  else
    plot_printf(buf, BLEN, "%.2f", yMax);
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
  y += YSTEP + YSTEP/2 ;
  plot_printf(buf, BLEN, "%s",unit);
  ili9341_drawstring(buf, x, y);

  color = DEFAULT_FG_COLOR;
  ili9341_set_foreground(color);
  y += YSTEP + YSTEP/2 ;
  if (rounding)
    plot_printf(buf, BLEN, "%d/",(int)setting.scale);
  else
    plot_printf(buf, BLEN, "%.2f/",setting.scale);
  ili9341_drawstring(buf, x, y);

  if (setting.auto_attenuation)
    color = DEFAULT_FG_COLOR;
  else
    color = BRIGHT_COLOR_GREEN;
  ili9341_set_foreground(color);
  y += YSTEP + YSTEP/2 ;
  ili9341_drawstring("Attn:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%ddB", setting.attenuate);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  if (setting.average>0) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP + YSTEP/2 ;
    ili9341_drawstring("Calc:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%s",averageText[setting.average]);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }
#ifdef __SPUR__
  if (setting.spur) {
    ili9341_set_foreground(BRIGHT_COLOR_GREEN);
    y += YSTEP + YSTEP/2 ;
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

  y += YSTEP + YSTEP/2 ;
  ili9341_drawstring("RBW:", x, y);

  y += YSTEP;
  plot_printf(buf, BLEN, "%dkHz", (int)actual_rbw);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  if (setting.frequency_step > 0) {
    ili9341_set_foreground(DEFAULT_FG_COLOR);
    y += YSTEP + YSTEP/2 ;
    ili9341_drawstring("VBW:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%dkHz",(int)setting.vbw);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }
  if (dirty)
    color = BRIGHT_COLOR_RED;
  else if (setting.step_delay)
    color = BRIGHT_COLOR_GREEN;
  else
    color = DEFAULT_FG_COLOR;

  ili9341_set_foreground(color);

  y += YSTEP + YSTEP/2 ;
  ili9341_drawstring("Scan:", x, y);

  y += YSTEP;
  int32_t t = (int)((2* vbwSteps * sweep_points * ( actualStepDelay / 100) )) /10
#ifdef __SPUR__
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
    y += YSTEP + YSTEP/2 ;
    ili9341_drawstring("Ref:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%dMHz",reffer_freq[setting.refer]/1000000);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  if (setting.offset != 0.0) {
    ili9341_set_foreground(BRIGHT_COLOR_RED);
    y += YSTEP + YSTEP/2 ;
    ili9341_drawstring("Amp:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%.1fdB",setting.offset);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  if (setting.trigger != T_AUTO) {
    if (is_paused()) {
      ili9341_set_foreground(BRIGHT_COLOR_GREEN);
    } else {
      ili9341_set_foreground(BRIGHT_COLOR_RED);
    }
    y += YSTEP + YSTEP/2 ;
    ili9341_drawstring("TRIG:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%ddBm",(int)setting.trigger_level);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  if (level_is_calibrated())
    color = BRIGHT_COLOR_GREEN;
  else
    color = BRIGHT_COLOR_RED;
  ili9341_set_foreground(color);
  y += YSTEP + YSTEP/2 ;
  if (MODE_LOW(setting.mode))
      ili9341_drawstring_7x13("M:L", x, y);
  else
    ili9341_drawstring_7x13("M:H", x, y);


  y = HEIGHT-7 + OFFSETY;
  if (rounding)
    plot_printf(buf, BLEN, "%d", (int)(yMax - setting.scale * NGRIDY));
  else
    plot_printf(buf, BLEN, "%.2f", (yMax - setting.scale * NGRIDY));
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
 {TC_SIGNAL,    TP_10MHZ,       20,     7,      -37, 30,    -90 },      // 3
 {TC_SIGNAL,    TP_10MHZ,       30,     7,      -32, 30,    -90 },      // 4
 {TC_BELOW,     TP_SILENT,      200,    100,    -75, 0,     0},         // 5  Wide band noise floor low mode
 {TC_BELOW,     TPH_SILENT,     600,    720,    -75, 0,     0},         // 6 Wide band noise floor high mode
 {TC_SIGNAL,    TP_10MHZEXTRA,  10,     8,      -20, 80,    -80 },      // 7 BPF loss and stop band
 {TC_FLAT,      TP_10MHZEXTRA,  10,     4,      -18, 20,    -60},       // 8 BPF pass band flatness
 {TC_BELOW,     TP_30MHZ,       430,    60,     -80, 0,     -80},       // 9 LPF cutoff
 {TC_SIGNAL,    TP_10MHZ_SWITCH,20,     7,      -58, 30,    -95 },      // 10 Switch isolation
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
  float threshold=stored_t[from];
  float sum = 0;
  int sum_count = 0;
  for (int j = from; j < to; j++) {
    sum += actual_t[j];
    sum_count++;
    if (actual_t[j] > threshold) {
      status = TS_FAIL;
      break;
    }
  }
  sum = sum / sum_count;
  if (sum > threshold - 5)
    status = TS_CRITICAL;
  if (status != TS_PASS)
    test_fail_cause[tc] = "Above ";
  return(status);
}

int validate_flatness(int i) {
  volatile int j;
  test_fail_cause[i] = "Passband ";
  for (j = peakIndex; j < setting._sweep_points; j++) {
    if (actual_t[j] < peakLevel - 6)    // Search right -3dB
      break;
  }
  //shell_printf("\n\rRight width %d\n\r", j - peakIndex );
  if (j - peakIndex < test_case[i].width)
    return(TS_FAIL);
  for (j = peakIndex; j > 0; j--) {
    if (actual_t[j] < peakLevel - 6)    // Search left -3dB
      break;
  }
  //shell_printf("Left width %d\n\r", j - peakIndex );
  if (peakIndex - j < test_case[i].width)
    return(TS_FAIL);
  test_fail_cause[i] = "";
  return(TS_PASS);
}

int validate_above(int tc) {
  int status = TS_PASS;
  for (int j = 0; j < setting._sweep_points; j++) {
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
        set_actual_power(test_value);
    } else
      set_actual_power(test_case[i].pass);
    goto common;
  case TC_MEASURE:
  case TC_SIGNAL:           // Validate signal
  common: current_test_status = validate_signal_within(i, 5.0);
    if (current_test_status == TS_PASS) {            // Validate noise floor
      current_test_status = validate_below(i, 0, setting._sweep_points/2 - test_case[i].width);
      if (current_test_status == TS_PASS) {
        current_test_status = validate_below(i, setting._sweep_points/2 + test_case[i].width, setting._sweep_points);
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
    current_test_status = validate_below(i, 0, setting._sweep_points);
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
  set_attenuation(0);
  switch(test_case[i].setup) {                // Prepare test conditions
  case TPH_SILENT:                             // No input signal
    set_mode(M_HIGH);
    goto common_silent;
  case TP_SILENT:                             // No input signal
    set_mode(M_LOW);
common_silent:
    set_refer_output(-1);
    for (int j = 0; j < setting._sweep_points; j++)
      stored_t[j] = test_case[i].pass;
    break;
  case TP_10MHZ_SWITCH:
    set_mode(M_LOW);
    set_refer_output(2);
    setting.step_atten = true;
    goto common;
  case TP_10MHZEXTRA:                         // Swept receiver
    set_mode(M_LOW);
    setting.tracking = true; //Sweep BPF
    setting.frequency_IF = 434000000;                // Center on SAW filters
    set_refer_output(2);
    goto common;
  case TP_10MHZ:                              // 10MHz input
    set_mode(M_LOW);
    set_refer_output(2);
    set_step_delay(1);                      // Precise scanning speed
#ifdef __SPUR__
    setting.spur = 1;
#endif
 common:

    for (int j = 0; j < setting._sweep_points/2 - test_case[i].width; j++)
      stored_t[j] = test_case[i].stop;
    for (int j = setting._sweep_points/2 + test_case[i].width; j < setting._sweep_points; j++)
      stored_t[j] = test_case[i].stop;
    for (int j = setting._sweep_points/2 - test_case[i].width; j < setting._sweep_points/2 + test_case[i].width; j++)
      stored_t[j] = test_case[i].pass;
    break;
  case TP_30MHZ:
    set_mode(M_LOW);
    set_refer_output(0);
 //   set_step_delay(1);                      // Do not set !!!!!
#ifdef __SPUR__
    setting.spur = 1;
#endif
    goto common;
  case TPH_30MHZ:
    set_mode(M_HIGH);
    set_refer_output(0);
    goto common;
  }
  setting.auto_attenuation = false;
  setting.attenuate = 0;
  trace[TRACE_STORED].enabled = true;
  set_reflevel(test_case[i].pass+10);
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

    set_RBW(FREQ_STEP/1000);
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
      set_attenuation(j);
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
    ui_mode_normal();
    int i = 15;       // calibrate low mode power on 30 MHz;
    test_prepare(i);
    setting.step_delay = 8000;
    for (int j= 0; j < 57; j++ ) {
      test_prepare(i);
      setting.step_delay = setting.step_delay * 5 / 4;
      setting.rbw = SI4432_force_RBW(j);
      shell_printf("RBW = %d, ",setting.rbw);
      set_sweep_frequency(ST_SPAN, (int32_t)(setting.rbw * 10000));
      setting.repeat = 10;
      test_acquire(i);                        // Acquire test
      test_validate(i);                       // Validate test
      float saved_peakLevel = peakLevel;
      if (peakLevel < -35) {
        shell_printf("Peak level too low, abort\n\r");
        return;
      }

      shell_printf("Start level = %f, ",peakLevel);
      while (setting.step_delay > 10 && peakLevel > saved_peakLevel - 1) {
        test_prepare(i);
        setting.step_delay = setting.step_delay * 4 / 5;
        //      shell_printf("\n\rRBW = %f",SI4432_force_RBW(j));
        set_sweep_frequency(ST_SPAN, (int32_t)(setting.rbw * 10000));
        setting.repeat = 10;
        test_acquire(i);                        // Acquire test
        test_validate(i);                       // Validate test
        //      shell_printf(" Step %f, %d",peakLevel, setting.step_delay);
      }
      setting.step_delay = setting.step_delay * 5 / 4;
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
    if (setting.test_argument > 0)
      i=setting.test_argument-1;
    do {
      setting.frequency_IF = old_IF;
      test_prepare(i);
      test_acquire(i);                        // Acquire test
      test_status[i] = test_validate(i);                       // Validate test
      if (test_status[i] != TS_PASS) {
        wait_user();
      }
      i++;
    } while (test_case[i].kind != TC_END && setting.test_argument == 0 );
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
  config.high_level_offset = 100;
  config.low_level_offset = 100;
}

#define CALIBRATE_RBWS  1
const int power_rbw [5] = { 100, 300, 30, 10, 3 };

void calibrate(void)
{
#ifdef __CALIBRATE__
  int local_test_status;
  float last_peak_level;
  in_selftest = true;
  reset_calibration();
  reset_settings(M_LOW);
  int i = 11;       // calibrate low mode power on 30 MHz;
  for (int j= 0; j < CALIBRATE_RBWS; j++ ) {
    set_RBW(power_rbw[j]);
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
//    chThdSleepMilliseconds(1000);
    if (local_test_status != TS_PASS) {
      ili9341_set_foreground(BRIGHT_COLOR_RED);
      ili9341_drawstring_7x13("Calibration failed", 30, 120);
      goto quit;
    } else {
      set_actual_power(-22.5);           // Should be -22.5dBm
      chThdSleepMilliseconds(1000);
    }
  }
  i = 12;           // Measure 270MHz in low mode
  set_RBW(100);
  test_prepare(i);
  test_acquire(i);                        // Acquire test
  last_peak_level = peakLevel;
  local_test_status = test_validate(i);                       // Validate test
  chThdSleepMilliseconds(1000);

  config.high_level_offset = 0;           /// Preliminary setting

  i = 13;           // Calibrate 270MHz in high mode
  for (int j = 0; j < CALIBRATE_RBWS; j++) {
    set_RBW(power_rbw[j]);
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
//    if (local_test_status != TS_PASS) {                       // Do not validate due to variations in SI4432
//      ili9341_set_foreground(BRIGHT_COLOR_RED);
//      ili9341_drawstring_7x13("Calibration failed", 30, 120);
//      goto quit;
//    } else
      set_actual_power(last_peak_level);
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
  set_mode(M_LOW);
#endif
}


