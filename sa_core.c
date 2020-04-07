// ---------------------------------------------------

#include "SI4432.h"		// comment out for simulation

int setting_mode = M_LOW;


int dirty = true;
int scandirty = true;
int setting_attenuate = 0;
int setting_step_atten;
int setting_rbw = 0;
int setting_average = 0;
int setting_show_stored = 0;
int setting_subtract_stored = 0;
int setting_drive=13; // 0-7 , 7=+20dBm, 3dB steps
int setting_agc = true;
int setting_lna = false;
int setting_tracking = false;
int setting_modulation = MO_NONE;
int setting_step_delay = 0;
int setting_frequency_step;
int setting_decay;
int setting_noise;
float actual_rbw = 0;
float setting_vbw = 0;

int setting_measurement;

int vbwSteps = 1;

//int setting_spur = 0;
uint32_t minFreq = 0;
uint32_t maxFreq = 520000000;

int setting_refer = -1;  // Off by default
const int reffer_freq[] = {30000000, 15000000, 10000000, 4000000, 3000000, 2000000, 1000000};

int in_selftest = false;

void reset_settings(int m)
{
  setting_mode = m;
  setting_attenuate = 0;
  setting_rbw = 0;
  setting_average = 0;
  setting_show_stored = 0;
  setting_subtract_stored = 0;
  setting_drive=13;
  setting_step_atten = 0;       // Only used in low output mode
  setting_agc = true;
  setting_lna = false;
  setting_tracking = false;
  setting_modulation = MO_NONE;
  setting_step_delay = 0;
  setting_vbw = 0;
  setting_decay=20;
  setting_noise=20;
  trace[TRACE_STORED].enabled = false;
  trace[TRACE_TEMP].enabled = false;

  setting_measurement = M_OFF;
//  setting_spur = 0;
  switch(m) {
  case M_LOW:
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_START, (int32_t) 0);
    set_sweep_frequency(ST_STOP, (int32_t) 350000000);
    SetRefpos(-10);
    break;
  case M_GENLOW:
    setting_drive=8;
    minFreq = 0;
    maxFreq = 520000000;
    set_sweep_frequency(ST_CENTER, (int32_t) 10000000);
    set_sweep_frequency(ST_SPAN, 0);
    break;
  case M_HIGH:
    minFreq = 240000000;
    maxFreq = 960000000;
    set_sweep_frequency(ST_START, (int32_t) minFreq);
    set_sweep_frequency(ST_STOP, (int32_t) maxFreq);
    SetRefpos(-30);
    break;
  case M_GENHIGH:
    setting_drive=8;
    minFreq = 240000000;
    maxFreq = 960000000;
    set_sweep_frequency(ST_CENTER, (int32_t) 300000000);
    set_sweep_frequency(ST_SPAN, 0);
    break;
  }
  SetScale(10);
  dirty = true;
}

void set_refer_output(int v)
{
  setting_refer = v;
  dirty = true;
}

int get_refer_output(void)
{
  return(setting_refer);
}

void set_decay(int d)
{
  if (d < 0 || d > 200)
    return;
  setting_decay = d;
  dirty = true;
}

void set_noise(int d)
{
  if (d < 5 || d > 200)
    return;
  setting_noise = d;
  dirty = true;
}

void set_measurement(int m)
{
  setting_measurement = m;
  dirty = true;
}
void SetDrive(int d)
{
  setting_drive = d;
  dirty = true;
}

void SetModulation(int m)
{
  setting_modulation = m;
  dirty = true;
}
void SetIF(int f)
{
  frequency_IF = f;
  dirty = true;
}

int GetMode(void)
{
  return(setting_mode);
  dirty = true;
}


#define POWER_STEP  0           // Should be 5 dB but appearently it is lower
#define POWER_OFFSET    12
#define SWITCH_ATTENUATION  29

int GetAttenuation(void)
{
  if (setting_mode == M_GENLOW) {
    if (setting_step_atten)
      return ( -(POWER_OFFSET + setting_attenuate - (setting_step_atten-1)*POWER_STEP + SWITCH_ATTENUATION));
    else
      return ( -POWER_OFFSET - setting_attenuate + (setting_drive & 7) * 3);
  }
  return(setting_attenuate);
}


void SetAttenuation(int a)
{
  if (setting_mode == M_GENLOW) {
    setting_drive = 0;
    a = a + POWER_OFFSET;
    if (a > 0) {
      setting_drive++;
      a = a - 3;
    }
    if (a > 0) {
      setting_drive++;
      a = a - 3;
    }
    if (a > 0) {
      setting_drive++;
      a = a - 3;
    }
    if (a > 0)
      a = 0;
    if( a >  - SWITCH_ATTENUATION) {
      setting_step_atten = 0;
    } else {
      a = a + SWITCH_ATTENUATION;
      setting_step_atten = 1;
    }
    a = -a;
  } else {
    setting_step_atten = 0;
  }
  if (a<0)
      a = 0;
  if (a> 31)
    a=31;
//  if (setting_attenuate == a)
//    return;
  setting_attenuate = a;
  dirty = true;
}

void SetStorage(void)
{
  for (int i=0; i<POINTS_COUNT;i++)
    stored_t[i] = actual_t[i];
  setting_show_stored = true;
  trace[TRACE_STORED].enabled = true;
  dirty = true;
}

int GetStorage(void)
{
  return(setting_show_stored);
}

void SetClearStorage(void)
{
  setting_show_stored = false;
  setting_subtract_stored = false;
  trace[TRACE_STORED].enabled = false;
  dirty = true;
}

void SetSubtractStorage(void)
{
  if (!setting_subtract_stored) {
    if (!setting_show_stored)
      SetStorage();
    setting_subtract_stored = true;
  } else {
    setting_subtract_stored = false;
  }
  dirty = true;
}

int GetSubtractStorage(void)
{
  return(setting_subtract_stored);
}

extern float peakLevel;
void SetPowerLevel(int o)
{
  float new_offset = o - peakLevel - setting_attenuate + settingLevelOffset();
  if (o != 100) {
    if (setting_mode == M_HIGH)
      config.high_level_offset = new_offset;
    else if (setting_mode == M_LOW)
      config.low_level_offset = new_offset;
  }
  else {
    config.low_level_offset = 100;
    config.high_level_offset = 100;
  }
  dirty = true;
}

int settingLevelOffset(void)
{
  if (setting_mode == M_HIGH) {
    if (config.high_level_offset == 100)
      return 0;
    return(config.high_level_offset);
  }
  if (setting_mode == M_LOW) {
    if (config.low_level_offset == 100)
      return 0;
    return(config.low_level_offset);
  }
  return(0);
}

int level_is_calibrated(void)
{
  if (setting_mode == M_HIGH && config.high_level_offset != 100)
    return 1;
  if (setting_mode == M_LOW && config.low_level_offset != 100)
    return 1;
  return(0);
}

void SetRBW(int v)
{
  setting_rbw = v;
  update_rbw();
  dirty = true;
}

int GetRBW(void)
{
  return(setting_rbw);
}

int GetActualRBW(void)
{
  return((int) actual_rbw);
}
#if 0
void SetSpur(int v)
{
//  setting_spur = v;
  dirty = true;
}
#endif

void SetStepDelay(int d)
{
  setting_step_delay = d;
  dirty = true;
}

void SetAverage(int v)
{
  setting_average = v;
  trace[TRACE_TEMP].enabled = (v != 0);
  dirty = true;
}

int GetAverage(void)
{
  return(setting_average);
}

void ToggleLNA(void)
{
  setting_lna = !setting_lna;
  dirty = true;
}

void toggle_tracking(void)
{
  setting_tracking = !setting_tracking;
  dirty = true;
}

int GetExtraVFO(void)
{
  return(setting_tracking);
}

int GetLNA(void)
{
  return(setting_lna);
}

void ToggleAGC(void)
{
  setting_agc = !setting_agc;
  dirty = true;
}

int GetAGC(void)
{
  return(setting_agc);
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
  if (setting_mode == m)
    return;
  reset_settings(m);
}

void apply_settings(void)
{
  if (setting_step_delay == 0){
      if      (actual_rbw >142.0)    actualStepDelay =  350;
      else if (actual_rbw > 75.0)    actualStepDelay =  450;
      else if (actual_rbw > 56.0)    actualStepDelay =  600;
      else if (actual_rbw > 37.0)    actualStepDelay =  800;
      else if (actual_rbw > 18.0)    actualStepDelay = 1100;
      else if (actual_rbw >  9.0)    actualStepDelay = 2000;
      else if (actual_rbw >  5.0)    actualStepDelay = 3500;
      else                           actualStepDelay = 6000;
  } else
    actualStepDelay = setting_step_delay;
  PE4302_Write_Byte(setting_attenuate * 2);
  if (setting_modulation == MO_NFM ) {
    SI4432_Sel = 1;
    SI4432_Write_Byte(0x7A, 1);  // Use frequency hopping channel width for FM modulation
  } else if (setting_modulation == MO_WFM ) {
    SI4432_Sel = 1;
    SI4432_Write_Byte(0x7A, 10);  // Use frequency hopping channel width for FM modulation
  } else {
    SI4432_Sel = 1;
    SI4432_Write_Byte(0x79, 0);  // IF no FM back to channel 0
  }
  SetRX(setting_mode);
  SI4432_SetReference(setting_refer);
  update_rbw();
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

static unsigned long old_freq[2] = { 0, 0 };

void setFreq(int V, unsigned long freq)
{
  SI4432_Sel = V;
  if (old_freq[V] != freq) {
    if (V == 0) {
      V = -V;
      V = -V;
    }
    SI4432_Set_Frequency(freq);
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

void SetAGCLNA(void) {
  unsigned char v = 0x40;
  if (setting_agc) v |= 0x20;
  if (setting_lna) v |= 0x10;
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
    SI4432_Transmit(setting_drive);
    // SI4432_SetReference(setting_refer);
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
    if (setting_step_atten) {
      SetSwitchReceive();
    } else {
      SetSwitchTransmit();
    }
    SI4432_Transmit(setting_drive);        // Not to overdrive the mixer

    SI4432_Sel = 1;
    SetSwitchReceive();
    SI4432_Transmit(13);                 // Fix LO drive a 10dBm

    break;
case M_GENHIGH: // Direct output from 1
    SI4432_Sel = 0;
    SI4432_Receive();
    SetSwitchReceive();

    SI4432_Sel = 1;
    if (setting_drive < 8) {
      SetSwitchReceive();
    } else {
      SetSwitchTransmit();
    }
    SI4432_Transmit(setting_drive);

    break;
  }
}

void update_rbw(void)
{
  setting_vbw = (setting_frequency_step)/1000.0;
  actual_rbw = setting_rbw;
//  float old_rbw = actual_rbw;
  if (actual_rbw == 0)
    actual_rbw = 2*setting_vbw;
  if (actual_rbw < 2.6)
    actual_rbw = 2.6;
  if (actual_rbw > 600)
    actual_rbw = 600;

  SI4432_Sel =  MODE_SELECT(setting_mode);
  actual_rbw = SI4432_SET_RBW(actual_rbw);

  vbwSteps = ((int)(2 * setting_vbw / actual_rbw));

  if (vbwSteps < 1)
    vbwSteps = 1;
  dirty = true;
}
#define MAX_MAX 4
int
search_maximum(int m, int center, int span)
{
  int from = center - span/2;
  int found = false;
  int to = center + span/2;
  int cur_max = 0;          // Always at least one maximum
  int max_index[4];
  temppeakIndex = 0;
  temppeakLevel = actual_t[from];
  max_index[cur_max] = from;
  int downslope = true;

  for (int i = from; i <= to; i++) {
    if (downslope) {
      if (temppeakLevel > actual_t[i]) {    // Follow down
        temppeakIndex = i;                  // Latest minimum
        temppeakLevel = actual_t[i];
      } else if (temppeakLevel + setting_noise < actual_t[i]) {    // Local minimum found
        temppeakIndex = i;                          // This is now the latest maximum
        temppeakLevel = actual_t[i];
        downslope = false;
      }
    } else {
      if (temppeakLevel < actual_t[i]) {    // Follow up
        temppeakIndex = i;
        temppeakLevel = actual_t[i];
      } else if (temppeakLevel - setting_noise > actual_t[i]) {    // Local max found

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
 420000,
 490000,
 510000,
 1600000,
 1840000,
 2960000,
/*
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

int avoid_spur(int f)
{
  int window = ((int)actual_rbw ) * 1000*2;
//  if (window < 50000)
//    window = 50000;
  if (! setting_mode == M_LOW || frequency_IF != spur_IF || actual_rbw > 300.0)
    return(false);
  for (unsigned int i = 0; i < (sizeof spur_table)/sizeof(int); i++) {
    if (f/window == spur_table[i]/window) {
//      spur_old_stepdelay = actualStepDelay;
//      actualStepDelay += 4000;
      return true;
    }
  }
  return false;
}

static int modulation_counter = 0;

char age[POINTS_COUNT];

float perform(bool break_on_operation, int i, int32_t f, int tracking)
{
//  long local_IF = (MODE_LOW(setting_mode)?frequency_IF + (int)(actual_rbw < 300.0?setting_spur * 1000 * actual_rbw :0):0);
  long local_IF;
  if (MODE_HIGH(setting_mode))
    local_IF = 0;
  else if (setting_mode == M_LOW && avoid_spur(f))
    local_IF = spur_alternate_IF;
  else
    local_IF = frequency_IF;

  if (i == 0 && dirty) {
    apply_settings();
    scandirty = true;
    dirty = false;
  }
  if (local_IF) {
    setFreq (0, local_IF);
  }
  if (MODE_OUTPUT(setting_mode) && setting_modulation == MO_AM) {
    int p = setting_attenuate * 2 + modulation_counter;
    PE4302_Write_Byte(p);
    if (modulation_counter == 3)
      modulation_counter = 0;
    else
      modulation_counter++;
    chThdSleepMicroseconds(250);
  } else if (MODE_OUTPUT(setting_mode) && (setting_modulation == MO_NFM || setting_modulation == MO_WFM )) {
      SI4432_Sel = 1;
      int offset;
      if (setting_modulation == MO_NFM ) {
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
    int lf = (uint32_t)(f + (int)(t * 500 * actual_rbw));
    if (MODE_INPUT(setting_mode) && tracking)
      setFreq (0, local_IF + lf - reffer_freq[setting_refer]);    // Offset so fundamental of reffer is visible
#if 0
    if (lf >11000000 || lf < 9000000) {
      lf = lf;
      break;
    }
#endif
    setFreq (1, local_IF + lf);
    if (MODE_OUTPUT(setting_mode))              // No substepping in output mode
      return(0);
    float subRSSI = SI4432_RSSI(lf, MODE_SELECT(setting_mode))+settingLevelOffset()+setting_attenuate;
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
  //again:
  for (int i = 0; i < sweep_points; i++) {
    RSSI = perform(break_on_operation, i, frequencies[i], setting_tracking);

    // back to toplevel to handle ui operation
    if (operation_requested && break_on_operation)
      return false;
    if (MODE_OUTPUT(setting_mode) && setting_modulation == MO_NONE) {
      osalThreadSleepMilliseconds(10);
    }

    if (MODE_INPUT(setting_mode)) {
      //    if (setting_spur == 1) {                           // First pass
      //      temp_t[i] = RSSI;
      //      continue;                                       // Skip all other processing
      //    }
      //    if (setting_spur == -1)                            // Second pass
      //      RSSI = ( RSSI < temp_t[i] ? RSSI : temp_t[i]);  // Minimum of two passes
      temp_t[i] = RSSI;
      if (setting_subtract_stored) {
        RSSI = RSSI - stored_t[i] ;
      }
      //   stored_t[i] = (SI4432_Read_Byte(0x69) & 0x0f) * 3.0 - 90.0; // Display the AGC value in thestored trace
      if (scandirty || setting_average == AV_OFF) {
        actual_t[i] = RSSI;
        age[i] = 0;
      } else {
        switch(setting_average) {
        case AV_MIN:      if (actual_t[i] > RSSI) actual_t[i] = RSSI; break;
        case AV_MAX_HOLD: if (actual_t[i] < RSSI) actual_t[i] = RSSI; break;
        case AV_MAX_DECAY:
          if (actual_t[i] < RSSI) {
            actual_t[i] = RSSI;
            age[i] = 0;
          } else {
            if (age[i] > setting_decay)
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
        } else if (temppeakLevel + setting_noise < actual_t[i]) {    // Local minimum found
          temppeakIndex = i;                          // This is now the latest maximum
          temppeakLevel = actual_t[i];
          downslope = false;
        }
      } else {
        if (temppeakLevel < actual_t[i]) {    // Follow up
          temppeakIndex = i;
          temppeakLevel = actual_t[i];
        } else if (temppeakLevel - setting_noise > actual_t[i]) {    // Local max found

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
    if (temp_min_level > actual_t[i])
      temp_min_level = actual_t[i];
#endif
  }
  //  if (setting_spur == 1) {
  //    setting_spur = -1;
  //    goto again;
  //  } else if (setting_spur == -1)
  //    setting_spur = 1;

  if (scandirty) {
    scandirty = false;
    draw_cal_status();
  }
#if 1
  if (MODE_INPUT(setting_mode)) {
    int i = 0;
    int m = 0;
    while (i < cur_max) {                                 // For all maxima found
      while (m < MARKERS_MAX) {
        if (markers[m].enabled == M_TRACKING_ENABLED) {   // Available marker found
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
      if (markers[m].enabled == M_TRACKING_ENABLED ) {    // More available markers found
        markers[m].index = 0;                             // Enabled but no max
        markers[m].frequency = frequencies[markers[m].index];
      }
      m++;                              // Try next marker
    }
    if (setting_measurement == M_IMD && markers[0].index > 10) {
      markers[1].enabled = search_maximum(1, markers[0].index*2, 8);
      markers[2].enabled = search_maximum(2, markers[0].index*3, 12);
      markers[3].enabled = search_maximum(3, markers[0].index*4, 16);
    } else if (setting_measurement == M_OIP3  && markers[0].index > 10 && markers[1].index > 10) {
      int l = markers[0].index;
      int r = markers[1].index;
      if (r < l) {
        l = markers[1].index;
        r = markers[0].index;
      }
      markers[2].enabled = search_maximum(2, l - (r-l), 10);
      markers[3].enabled = search_maximum(3, r + (r-l), 10);
    }
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
  }
  //    redraw_marker(peak_marker, FALSE);
  palSetPad(GPIOB, GPIOB_LED);
  return true;
}


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

  if (MODE_OUTPUT(setting_mode))     // No cal status during output
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

  if (setting_attenuate) {
    ili9341_set_foreground(BRIGHT_COLOR_GREEN);
    y += YSTEP*2;
    ili9341_drawstring("Attn:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "-%ddB", setting_attenuate);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  if (setting_average>0) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP*2;
    ili9341_drawstring("Calc:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%s",averageText[setting_average]);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }
#if 0
  if (setting_spur) {
    ili9341_set_foreground(BRIGHT_COLOR_BLUE);
    y += YSTEP*2;
    ili9341_drawstring("Spur:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "ON");
    ili9341_drawstring(buf, x, y);
  }
#endif

  if (setting_rbw)
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
  plot_printf(buf, BLEN, "%dkHz",(int)setting_vbw);
  buf[5]=0;
  ili9341_drawstring(buf, x, y);

  if (dirty)
    ili9341_set_foreground(BRIGHT_COLOR_RED);

  y += YSTEP*2;
  ili9341_drawstring("Scan:", x, y);

  y += YSTEP;
  int32_t t = (int)((2* vbwSteps * sweep_points * ( actualStepDelay / 100) )) /10  /* * (setting_spur ? 2 : 1) */; // in mS
  if (t>1000)
    plot_printf(buf, BLEN, "%dS",(t+500)/1000);
  else
    plot_printf(buf, BLEN, "%dmS",t);

  buf[5]=0;
  ili9341_drawstring(buf, x, y);


  if (setting_refer >= 0) {
    ili9341_set_foreground(BRIGHT_COLOR_RED);
    y += YSTEP*2;
    ili9341_drawstring("Ref:", x, y);

    y += YSTEP;
    plot_printf(buf, BLEN, "%dMHz",reffer_freq[setting_refer]/1000000);
    buf[5]=0;
    ili9341_drawstring(buf, x, y);
  }

  ili9341_set_foreground(BRIGHT_COLOR_GREEN);
  y += YSTEP*2;
  if (MODE_LOW(setting_mode))
      ili9341_drawstring_7x13("M:L", x, y);
  else
    ili9341_drawstring_7x13("M:H", x, y);


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
  TC_SIGNAL, TC_BELOW, TC_ABOVE, TC_FLAT, TC_MEASURE, TC_SET, TC_END,
};

enum {
  TP_SILENT, TPH_SILENT, TP_10MHZ, TP_10MHZEXTRA, TP_30MHZ, TPH_30MHZ
};

#define TEST_COUNT  16

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
 {TC_SIGNAL,    TP_10MHZ,       20,     7,      -40, 30,    -90 },      // 3
 {TC_SIGNAL,    TP_10MHZ,       30,     7,      -30, 30,    -90 },      // 4
 {TC_BELOW,     TP_SILENT,      200,    100,    -75, 0,     0},         // 5  Wide band noise floor low mode
 {TC_BELOW,     TPH_SILENT,     600,    720,    -75, 0,     0},         // 6 Wide band noise floor high mode
 {TC_SIGNAL,    TP_10MHZEXTRA,  10,     8,      -20, 50,    -70 },      // 7 BPF loss and stop band
 {TC_FLAT,      TP_10MHZEXTRA,  10,     4,      -25, 20,    -70},       // 8 BPF pass band flatness
 {TC_BELOW,     TP_30MHZ,       430,    60,     -75, 0,     -85},       // 9 LPF cutoff
 {TC_END,       0,              0,      0,      0,   0,     0},
 {TC_MEASURE,   TP_30MHZ,       30,     7,      -25, 30,    -80 },      // 11 Measure power level and noise
 {TC_MEASURE,   TP_30MHZ,       270,    4,      -50, 30,    -85 },       // 12 Measure powerlevel and noise
 {TC_MEASURE,   TPH_30MHZ,      270,    4,      -35, 30,    -50 },       // 13 Calibrate power high mode
 {TC_END,       0,              0,      0,      0,   0,     0},
 {TC_MEASURE,   TP_30MHZ,       30,     1,      -25, 30,    -80 },      // 15 Measure RBW step time
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
    setting_mode = M_LOW;
  else
    setting_mode = M_HIGH;
#endif
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
 common:
    if (validate_peak_within(i, 5.0))                // Validate Peak
      current_test_status = TS_PASS;
    else if (validate_peak_within(i, 10.0))
      current_test_status = TS_CRITICAL;
    else
      current_test_status = TS_FAIL;
    if (current_test_status != TS_PASS)
      test_fail_cause[i] = "Peak ";
    if (current_test_status == TS_PASS) {            // Validate noise floor
      for (int j = 0; j < POINTS_COUNT/2 - test_case[i].width; j++) {
        if (actual_t[j] > test_case[i].stop - 5)
          current_test_status = TS_CRITICAL;
        else if (actual_t[j] > test_case[i].stop) {
          current_test_status = TS_FAIL;
          break;
        }
      }
      for (int j = POINTS_COUNT/2 + test_case[i].width; j < POINTS_COUNT; j++) {
        if (actual_t[j] > test_case[i].stop - 5)
          current_test_status = TS_CRITICAL;
        else if (actual_t[j] > test_case[i].stop) {
          current_test_status = TS_FAIL;
          break;
        }
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
    for (int j = 0; j < POINTS_COUNT; j++) {
      if (actual_t[j] < test_case[i].pass + 5)
        current_test_status = TS_CRITICAL;
      else if (actual_t[j] < test_case[i].pass) {
        current_test_status = TS_FAIL;
        break;
      }
    }
    if (current_test_status != TS_PASS)
      test_fail_cause[i] = "Above ";
    break;
  case TC_BELOW:   // Validate signal below curve
      current_test_status = validate_below();
      if (current_test_status != TS_PASS)
        test_fail_cause[i] = "Above ";
      break;
  case TC_FLAT:   // Validate passband flatness
    current_test_status = validate_flatness(i);
    if (current_test_status != TS_PASS)
      test_fail_cause[i] = "Passband ";
    break;

  }

  // Report status

  if (current_test_status != TS_PASS || test_case[i+1].kind == TC_END)
    test_wait = true;
//  draw_frequencies();
//  draw_cal_status();
  draw_all(TRUE);
  resume_sweep();
  return current_test_status;
}

void test_prepare(int i)
{
  setting_tracking = false; //Default test setup
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
    setting_tracking = true; //Sweep BPF
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
  set_sweep_frequency(ST_CENTER, (int32_t)(test_case[i].center * 1000000));
  set_sweep_frequency(ST_SPAN, (int32_t)(test_case[i].span * 1000000));
  draw_cal_status();
}

extern void menu_autosettings_cb(int item);
extern float SI4432_force_RBW(int i);

void self_test(void)
{
#if 0                 // RAttenuator test
  int local_test_status;
  in_selftest = true;
  reset_settings(M_LOW);
  int i = 14;       // calibrate low mode power on 30 MHz;
  test_prepare(i);
  for (int j= 0; j < 32; j++ ) {
    test_prepare(i);
    SetAttenuation(j);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
    shell_printf("Target %d, actual %f\n\r",j, peakLevel);
  }
  return;
#else

#if 0                   // RBW step time search
  int local_test_status;
  in_selftest = true;
  reset_settings(M_LOW);
  int i = 14;       // calibrate low mode power on 30 MHz;
  test_prepare(i);
  setting_step_delay = 6000;
  for (int j= 0; j < 57; j++ ) {
    setting_step_delay = setting_step_delay * 4/3;
    setting_rbw = SI4432_force_RBW(j);
    shell_printf("RBW = %d, ",setting_rbw);
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
    float saved_peakLevel = peakLevel;
    if (peakLevel < -30) {
      shell_printf("Peak level too low, abort\n\r");
      return;
    }

    shell_printf("Start level = %f, ",peakLevel);
    while (setting_step_delay > 100 && peakLevel > saved_peakLevel - 1) {
      setting_step_delay = setting_step_delay * 3 / 4;
//      test_prepare(i);
//      shell_printf("RBW = %f\n\r",SI4432_force_RBW(j));
      test_acquire(i);                        // Acquire test
      local_test_status = test_validate(i);                       // Validate test
 //     shell_printf("Step %f, %d",peakLevel, setting_step_delay);
    }
    setting_step_delay = setting_step_delay * 4 / 3;
    shell_printf("End level = %f, step time = %d\n\r",peakLevel, setting_step_delay);
  }
  return;
#else

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
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    test_status[i] = test_validate(i);                       // Validate test
    if (test_status[i] != TS_PASS) {
      wait_user();
    }
    i++;
  }
  ili9341_set_foreground(BRIGHT_COLOR_GREEN);
  ili9341_drawstring_7x13("Self test complete", 30, 120);
  ili9341_drawstring_7x13("Touch screen to continue", 30, 140);
  wait_user();
  ili9341_clear_screen();

  sweep_mode = SWEEP_ENABLE;
  show_test_info = FALSE;
  set_refer_output(0);
  reset_settings(M_LOW);
  in_selftest = false;
#endif
#endif
}

void reset_calibration(void)
{
  SetPowerLevel(100);
}

#define CALIBRATE_RBWS  1
const int power_rbw [5] = { 100, 300, 30, 10, 3 };

void calibrate(void)
{
  int local_test_status;
  float last_peak_level;
  in_selftest = true;
  SetPowerLevel(100);
  reset_settings(M_LOW);
  int i = 10;       // calibrate low mode power on 30 MHz;
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
  i = 11;           // Measure 270MHz in low mode
  SetRBW(100);
  test_prepare(i);
  test_acquire(i);                        // Acquire test
  last_peak_level = peakLevel;
  local_test_status = test_validate(i);                       // Validate test
  chThdSleepMilliseconds(1000);

  config.high_level_offset = 0;           /// Preliminary setting

  i = 12;           // Calibrate 270MHz in high mode
  for (int j = 0; j < CALIBRATE_RBWS; j++) {
    SetRBW(power_rbw[j]);
    test_prepare(i);
    test_acquire(i);                        // Acquire test
    local_test_status = test_validate(i);                       // Validate test
    if (local_test_status != TS_PASS) {
      ili9341_set_foreground(BRIGHT_COLOR_RED);
      ili9341_drawstring_7x13("Calibration failed", 30, 120);
      goto quit;
    } else
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
  reset_settings(M_LOW);
}


