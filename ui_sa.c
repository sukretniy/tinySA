/* Copyright (c) 2020, Erik Kaashoek erik@kaashoek.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */



void markmap_all_markers(void);
static void menu_marker_modify_cb(int item, uint8_t data);
extern const menuitem_t menu_marker_modify[];
void set_sweep_frequency(int type, uint32_t frequency);
uint32_t get_sweep_frequency(int type);
void clearDisplay(void);

void blit16BitWidthBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height,
                                 const uint16_t *bitmap);


const uint16_t left_icons [] =
{
#define I_EMPTY 0*16
        0x0000,
        0x0000,
        0x0000,
        0x0001,
        0x0001,
        0x0001,
        0x0000,
        0x0000,
        0x0000,
        0x0000,
        0x0001,
        0x0001,
        0x0001,
        0x0000,
        0x0000,
        0x0000,

#define I_HIGH_INPUT 1*16
        /* +-----------------+
           |                 |
           |          **     |
           |           ***   |
           |    ************ |
           |           ***   |
           |          **     |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           +-----------------+ */
        0x0000,
        0x0000,
        0x0060,
        0x0039,
        0x0fff,
        0x0039,
        0x0060,
        0x0000,
        0x0000,
        0x0000,
        0x0001,
        0x0001,
        0x0001,
        0x0000,
        0x0000,
        0x0000,

#define I_LOW_INPUT 2*16
        /* +-----------------+
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |                 |
           |          **     |
           |          ****   |
           |    ************ |
           |          ****   |
           |          **     |
           |                 |
           +-----------------+ */
        0x0000,
        0x0000,
        0x0000,
        0x0001,
        0x0001,
        0x0001,
        0x0000,
        0x0000,
        0x0000,
        0x0060,
        0x0039,
        0x0fff,
        0x0039,
        0x0060,
        0x0000,
        0x0000,

#define I_LOW_OUTPUT 3*16

        0b0000000000000000,
        0b0000000000000000,
        0b0000000000000000,
        0b0000000000000001,
        0b0000000000000001,
        0b0000000000000001,
        0b0000000000000000,
        0b0000000000000000,
        0b0000000000000000,
        0b0000000110000000,
        0b0000011100000001,
        0b0000111111111111,
        0b0000011100000001,
        0b0000000110000000,
        0b0000000000000000,
        0b0000000000000000,

#define I_HIGH_OUTPUT 4*16

        0b0000000000000000,
        0b0000000000000000,
        0b0000000110000000,
        0b0000011100000001,
        0b0000111111111111,
        0b0000011100000001,
        0b0000000110000000,
        0b0000000000000000,
        0b0000000000000000,
        0b0000000000000000,
        0b0000000000000001,
        0b0000000000000001,
        0b0000000000000001,
        0b0000000000000000,
        0b0000000000000000,
        0b0000000000000000,

#define I_CONNECT 5*16

        0b0000000000000000,
        0b0000000000000000,
        0b0000000000110000,
        0b0000000000111101,
        0b0000001111111111,
        0b0000010000111101,
        0b0000100000110000,
        0b0001000000000000,
        0b0001000000000000,
        0b0000100000110000,
        0b0000010000111101,
        0b0000001111111111,
        0b0000000000111101,
        0b0000000000110000,
        0b0000000000000000,
        0b0000000000000000,

};

const uint16_t right_icons [] =
{
#define I_SA    0
 /* Character 0 (0x00):
    width 16
    +-----------------+
    |                 |
    | *************** |
    | *             * |
    |**             * |
    | *  *          * |
    | *  *          * |
    | *  *   *      * |
    | *  *   *      * |
    | *  *   *   *  * |
    | *  * * *   *  * |
    | *  * * * * *  * |
    | *  * * * * *  * |
    |** *********** * |
    | *             * |
    | *************** |
    |                 |
    +-----------------+ */
 0x0000,
 0x7fff,
 0x4001,
 0xc001,
 0xc001,
 0xc001,
 0x4801,
 0x4801,
 0x4a89,
 0x4aa9,
 0xcaa9,
 0xdffd,
 0xc001,
 0x4001,
 0x7fff,
 0x0000,

#define I_GEN   1
 /* Character 0 (0x00):
    width 16
    +-----------------+
    |                 |
    | *************** |
    | *             * |
    |**             * |
    | *  *****   ** * |
    | *  *   *   *  * |
    | *  *   *   *  * |
    | *  *   *   *  * |
    | *  *   *   *  * |
    | *  *   *   *  * |
    | *  *   *   *  * |
    | * **   *****  * |
    |**             * |
    | *             * |
    | *************** |
    |                 |
    +-----------------+ */
 0x0000,
 0x7fff,
 0x4001,
 0xc001,
 0xcf8d,
 0xc889,
 0x4889,
 0x4889,
 0x4889,
 0x4889,
 0xc889,
 0xd8f9,
 0xc001,
 0x4001,
 0x7fff,
 0x0000,

#define I_CONFIG 2

        0b0000000000000000,
        0b0111111111111111,
        0b0100000000000001,
        0b1100000010000001,
        0b1100001111000001,
        0b1100011110001001,
        0b0100011100011101,
        0b0100011110111001,
        0b0100001111111001,
        0b0100011111110001,
        0b1100111110000001,
        0b1101111100000001,
        0b1100111000000001,
        0b0100000000000001,
        0b0111111111111111,
        0b0000000000000000,

#define I_SINUS 3

        0b0000000000000000,
        0b0111111111111111,  // 1
        0b0100000000000001,  // 2
        0b1100000000000001,  // 3
        0b1100000000110001,  // 4
        0b1100000001001001,  // 5
        0b0100000010000101,  // 6
        0b0101000010000101,  // 7
        0b0101000010000101,  // 8
        0b0101000010000001,  // 9
        0b1100100100000001,  //10
        0b1100011000000001,  //11
        0b1100000000000001,  //12
        0b0100000000000001,  //13
        0b0111111111111111,  //14
        0b0000000000000000,
};

enum {
  KM_START=1, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_REFLEVEL, KM_SCALE, KM_ATTENUATION,
  KM_ACTUALPOWER, KM_IF, KM_SAMPLETIME, KM_DRIVE, KM_LOWOUTLEVEL, KM_DECAY, KM_NOISE,
  KM_10MHZ, KM_REPEAT, KM_OFFSET, KM_TRIGGER, KM_LEVELSWEEP, KM_SWEEP_TIME,
};



#define KP_X(x) (48*(x) + 2 + (LCD_WIDTH-BUTTON_WIDTH-192))
#define KP_Y(y) (48*(y) + 2)


#define KP_PERIOD 10
#define KP_MINUS 11
#define KP_X1 12
#define KP_K 13
#define KP_M 14
#define KP_G 15
#define KP_BS 16
#define KP_INF 17
#define KP_DB 18
#define KP_PLUSMINUS 19
#define KP_KEYPAD 20
#define KP_m 21
#define KP_u 22
#define KP_n 23

#define KP_1    32
#define KP_2    33
#define KP_5    34
#define KP_10   35
#define KP_20   36
#define KP_50   37
#define KP_100  38
#define KP_200  39
#define KP_500  40


typedef struct {
  uint8_t x:4;
  uint8_t y:4;
  int8_t  c;
} keypads_t;

static const keypads_t *keypads;

static uint8_t keypads_last_index;

// 7 8 9 G
// 4 5 6 M
// 1 2 3 k
// 0 . < x

static const keypads_t keypads_freq[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, 0 },
  { 0, 2, 1 },
  { 1, 2, 2 },
  { 2, 2, 3 },
  { 0, 1, 4 },
  { 1, 1, 5 },
  { 2, 1, 6 },
  { 0, 0, 7 },
  { 1, 0, 8 },
  { 2, 0, 9 },
  { 3, 0, KP_G },
  { 3, 1, KP_M },
  { 3, 2, KP_K },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

// 7 8 9
// 4 5 6
// 1 2 3
// 0 . < x

static const keypads_t keypads_positive[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, 0 },
  { 0, 2, 1 },
  { 1, 2, 2 },
  { 2, 2, 3 },
  { 0, 1, 4 },
  { 1, 1, 5 },
  { 2, 1, 6 },
  { 0, 0, 7 },
  { 1, 0, 8 },
  { 2, 0, 9 },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

// 100 200 500 n
// 10  20  50  u
// 1   2   5   m
// 0   .   <   x

static const keypads_t keypads_pos_unit[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, 0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_5 },
  { 0, 1, KP_10 },
  { 1, 1, KP_20 },
  { 2, 1, KP_50 },
  { 0, 0, KP_100 },
  { 1, 0, KP_200 },
  { 2, 0, KP_500 },
  { 3, 0, KP_n },
  { 3, 1, KP_u },
  { 3, 2, KP_m },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

// 7 8 9 m
// 4 5 6 u
// 1 2 3 -
// 0 . < x

static const keypads_t keypads_plusmin_unit[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, 0 },
  { 0, 2, 1 },
  { 1, 2, 2 },
  { 2, 2, 3 },
  { 0, 1, 4 },
  { 1, 1, 5 },
  { 2, 1, 6 },
  { 0, 0, 7 },
  { 1, 0, 8 },
  { 2, 0, 9 },
  { 3, 0, KP_u},
  { 3, 1, KP_m},
  { 3, 2, KP_MINUS },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};
// 7 8 9
// 4 5 6
// 1 2 3 -
// 0 . < x

static const keypads_t keypads_plusmin[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, 0 },
  { 0, 2, 1 },
  { 1, 2, 2 },
  { 2, 2, 3 },
  { 0, 1, 4 },
  { 1, 1, 5 },
  { 2, 1, 6 },
  { 0, 0, 7 },
  { 1, 0, 8 },
  { 2, 0, 9 },
  { 3, 0, KP_u},
  { 3, 1, KP_m},
  { 3, 2, KP_MINUS },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

// 7 8 9
// 4 5 6
// 1 2 3 m
// 0 . < x
static const keypads_t keypads_time[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, 0 },
  { 0, 2, 1 },
  { 1, 2, 2 },
  { 2, 2, 3 },
  { 0, 1, 4 },
  { 1, 1, 5 },
  { 2, 1, 6 },
  { 0, 0, 7 },
  { 1, 0, 8 },
  { 2, 0, 9 },
//  { 3, 0, KP_n},
//  { 3, 1, KP_u},
  { 3, 2, KP_m },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, -1 }
};

static const keypads_t * const keypads_mode_tbl[] = {
  NULL,         // never used
  keypads_freq, // start
  keypads_freq, // stop
  keypads_freq, // center
  keypads_freq, // span
  keypads_freq, // cw freq
  keypads_plusmin_unit, // reflevel
  keypads_pos_unit, // scale
  keypads_positive, // attenuation
  keypads_plusmin_unit, // actual power
  keypads_freq, // IF
  keypads_positive, // sample delay
  keypads_positive, // drive
  keypads_plusmin,    // KM_LOWOUTLEVEL
  keypads_positive,    // KM_DECAY
  keypads_positive,    // KM_NOISE
  keypads_plusmin,    // KM_10MHz
  keypads_positive,    // KM_REPEA
  keypads_plusmin,    // KM_OFFSET
  keypads_plusmin_unit,    // KM_TRIGGER
  keypads_plusmin,    // KM_LEVELSWEEP
  keypads_time,     // KM_SWEEP_TIME
};

#ifdef __VNA__
static const char * const keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY"
};
#endif
#ifdef __SA__
static const char * const keypad_mode_label[] = {
  "error", "START", "STOP", "CENTER", "SPAN", "FREQ", "\2REF\0LEVEL", "SCALE", // 0-7
  "\2ATTENUATE\0 0-31dB", "\2ACTUAL\0POWER", "IF", "\2SAMPLE\0DELAY", "DRIVE", "LEVEL", "SCANS", "LEVEL", // 8-15
  "OFFSET" , "\2SAMPLE\0REPEAT", "OFFSET", "\2TRIGGER\0LEVEL", "\2LEVEL\0SWEEP", "\2SWEEP\0SECONDS"// 16-
};
#endif


// ===[MENU CALLBACKS]=========================================================


int generator_enabled = false;

extern const menuitem_t  menu_lowoutputmode[];
extern const menuitem_t  menu_highoutputmode[];
extern const menuitem_t  menu_modulation[];
extern const menuitem_t  menu_top[];
extern const menuitem_t  menu_tophigh[];
extern const menuitem_t  menu_topultra[];

 void menu_mode_cb(int item, uint8_t data)
{
  (void)data;
  set_mode(item);
//  draw_cal_status();
  switch (item) {
  case 0:
//    if (setting.mode != M_LOW)
//      set_mode(M_LOW);
    menu_move_back();
    ui_mode_normal();
    break;
  case 1:
//    if (setting.mode != M_HIGH)
//      set_mode(M_HIGH);
    menu_move_back();
    ui_mode_normal();
    break;
  case 2:
    menu_push_submenu(menu_lowoutputmode);
    break;
  case 3:
    menu_push_submenu(menu_highoutputmode);
    break;
#ifdef __ULTRA__
  case 7:
    menu_push_submenu(menu_topultra);
    break;
#endif
  }
  redraw_request |= REDRAW_CAL_STATUS;
}

void menu_load_preset_cb(int item, uint8_t data)
{
  (void)item;
  if (caldata_recall(data) == -1) {
    if (data == 0)
      reset_settings(setting.mode);  // Restore all defaults
     else {
      draw_menu();
      return;
     }
  }
  menu_move_back();
  ui_mode_normal();
}

void menu_store_preset_cb(int item, uint8_t data)
{
  (void)item;
  if (data == 100) {
    reset_settings(M_LOW);  // Restore all defaults in Low mode
 //   setting.mode = -1;
    data = 0;
  }
  caldata_save(data);
  menu_move_back();
  ui_mode_normal();
}


extern int dirty;
void menu_autosettings_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  reset_settings(setting.mode);

  active_marker = 0;
  for (int i = 1; i<MARKER_COUNT; i++ ) {
    markers[i].enabled = M_DISABLED;
  }
  markers[0].enabled = M_ENABLED;
  markers[0].mtype = M_REFERENCE | M_TRACKING;

  //  set_refer_output(1);

  //  SetPowerLevel(100); // Reset
  set_clear_storage();
  dirty = true;
  //  menu_move_back();   // stay in input menu
  ui_mode_normal();
//  draw_cal_status();
}

static void menu_calibrate_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 1:
    sweep_mode = SWEEP_CALIBRATE;
    menu_move_back();
    ui_mode_normal();
    break;
  case 2:
    reset_calibration();
    draw_menu();
    break;
  }
}

static void menu_scanning_speed_cb(int item, uint8_t data)
{
  (void)item;
  set_step_delay(data);
//    menu_move_back();
  ui_mode_normal();
}

static void menu_config_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 0:
    touch_cal_exec();
    redraw_frame();
    request_to_redraw_grid();
    draw_menu();
    break;
  case 1:
    touch_draw_test();
    redraw_frame();
    request_to_redraw_grid();
    draw_menu();
    break;
  case 2:
    sweep_mode = 0;         // Suspend sweep to save time
    menu_move_back();
    ui_mode_normal();
    setting.test = 0;
    setting.test_argument = 0;
    sweep_mode = SWEEP_SELFTEST;
    break;
  case 4:
    show_version();
    redraw_frame();
    request_to_redraw_grid();
    draw_menu();
  }
}

static void menu_dfu_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 0:
      enter_dfu();
  }
}


// const int menu_modulation_value[]={MO_NONE,MO_AM_1, MO_NFM, MO_WFM, MO_EXTERNAL};
const char *menu_modulation_text[]={"NONE","AM 1kHz","AM 10Hz","NARROW FM","WIDE FM", "EXTERNAL"};

static void menu_modulation_cb(int item, uint8_t data)
{
  (void)item;
//Serial.println(item);
  if (data) {
    set_sweep_frequency(ST_SPAN, 0);      // No other scanning allowed when modulation is on!!!!!
    set_level_sweep(0);
  }
  set_modulation(data);
  menu_move_back();
//  ui_mode_normal();   // Stay in menu mode
//  draw_cal_status();
}


const int menu_reffer_value[]={-1,0,1,2,3,4,5,6};
const char *menu_reffer_text[]={"OFF","30MHz","15MHz","10MHz","4MHz","3MHz","2MHz","1MHz"};
static void menu_reffer_cb(int item, uint8_t data)
{
  (void)item;
//Serial.println(item);
  set_refer_output(menu_reffer_value[data]);
  menu_move_back();
//  ui_mode_normal();   // Stay in menu mode
//  draw_cal_status();
}

static void menu_drive_cb(int item, uint8_t data)
{
  (void)item;
//Serial.println(item);
  set_drive(data);
  menu_move_back();
//  ui_mode_normal();
//  draw_cal_status();
}



#ifdef __SPUR__
static void menu_spur_cb(int item, uint8_t data)
{
  (void)data;
  (void)item;
  if (setting.spur)
    set_spur(0);
  else
    set_spur(1); // must be 0 or 1 !!!!
//  menu_move_back();
  ui_mode_normal();
  redraw_request |= REDRAW_CAL_STATUS;
}
#endif

static void menu_measure_cb(int item, uint8_t data)
{
  (void)item;
  menu_move_back();
#ifdef __MEASURE__
  switch(data) {
    case M_OFF:                                     // Off
      reset_settings(setting.mode);
      set_measurement(M_OFF);
      break;
    case M_IMD:                                     // IMD
      reset_settings(setting.mode);
      for (int i = 0; i< MARKERS_MAX; i++) {
        markers[i].enabled = M_ENABLED;
        markers[i].mtype = M_DELTA | M_TRACKING;
      }
      markers[0].mtype = M_REFERENCE | M_TRACKING;
      kp_help_text = "Frequency of fundamental";
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      set_sweep_frequency(ST_START, 0);
      set_sweep_frequency(ST_STOP, uistat.value*5);
      set_measurement(M_IMD);
      break;
    case M_OIP3:                                     // OIP3
      reset_settings(setting.mode);
      for (int i = 0; i< MARKERS_MAX; i++) {
        markers[i].enabled = M_ENABLED;
        markers[i].mtype = M_DELTA;
      }
      markers[0].mtype = M_REFERENCE | M_TRACKING;
      markers[1].mtype = M_TRACKING;
      kp_help_text = "Frequency of left signal";
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      int left =  uistat.value;
      kp_help_text = "Right signal";
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      int right =  uistat.value;
      set_sweep_frequency(ST_CENTER, (left+right)/2);
      set_sweep_frequency(ST_SPAN, (right - left)*5);
      set_measurement(M_OIP3);
      break;
    case M_PHASE_NOISE:                             // Phase noise
      reset_settings(setting.mode);
      for (int i = 0; i< MARKERS_MAX; i++) {
        markers[i].enabled = M_DISABLED;
        markers[i].mtype = M_NORMAL;
      }
      markers[0].enabled = M_ENABLED;
      markers[0].mtype = M_REFERENCE | M_TRACKING;
      markers[1].enabled = M_ENABLED;
      markers[1].mtype = M_DELTA | M_NOISE;
      kp_help_text = "Frequency of signal";
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      kp_help_text = "Frequency offset";
      ui_mode_keypad(KM_SPAN);
      ui_process_keypad();
      set_sweep_frequency(ST_SPAN, uistat.value*4);
      set_measurement(M_PHASE_NOISE);
      set_average(4);

      break;
    case M_STOP_BAND:                             // STop band measurement
      reset_settings(setting.mode);
      markers[1].enabled = M_ENABLED;
      markers[1].mtype = M_DELTA;
      markers[2].enabled = M_ENABLED;
      markers[2].mtype = M_DELTA;
      kp_help_text = "Frequency of signal";
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      kp_help_text = "Width of signal";
      ui_mode_keypad(KM_SPAN);
      ui_process_keypad();
      set_sweep_frequency(ST_SPAN, uistat.value*4);
      set_measurement(M_STOP_BAND);
//      SetAverage(4);

      break;
    case M_PASS_BAND:                             // STop band measurement
      reset_settings(setting.mode);
      markers[1].enabled = M_ENABLED;
      markers[1].mtype = M_DELTA;
      markers[2].enabled = M_ENABLED;
      markers[2].mtype = M_DELTA;
      kp_help_text = "Frequency of signal";
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      kp_help_text = "Width of signal";
      ui_mode_keypad(KM_SPAN);
      ui_process_keypad();
      set_sweep_frequency(ST_SPAN, uistat.value*2);
      set_measurement(M_PASS_BAND);
//      SetAverage(4);

      break;
    case M_LINEARITY:
      set_measurement(M_LINEARITY);
      ui_mode_normal();
      break;
  }
#endif
//  selection = -1;
  ui_mode_normal();
//  draw_cal_status();
}

static void menu_atten_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  set_auto_attenuation();
  menu_move_back();
  ui_mode_normal();
}

static void menu_reflevel_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  set_auto_reflevel(true);
  menu_move_back();
  ui_mode_normal();
}

static void menu_storage_cb(int item, uint8_t data)
{
  (void)item;
  switch(data) {
    case 0:
      set_storage();
      break;
    case 1:
      set_clear_storage();
      break;
    case 2:
      set_subtract_storage();
      break;
    case 3:
      toggle_waterfall();
      break;
  }
  menu_move_back();
  ui_mode_normal();
//  draw_cal_status();
}

static void menu_average_cb(int item, uint8_t data)
{
  (void)data;
  set_average(item);
  menu_move_back();
  ui_mode_normal();
  redraw_request |= REDRAW_CAL_STATUS;
}

static void
menu_marker_select_cb(int item, uint8_t data)
{
  (void)data;
//  int t;
  if (item >= 0 && item < MARKERS_MAX) {
    markers[item].enabled = true;
    active_marker_select(item);
    menu_push_submenu(menu_marker_modify);
    redraw_marker(active_marker);
    draw_menu();
  }
}

static void menu_marker_modify_cb(int item, uint8_t data)
{
  (void)item;
  if (markers[active_marker].enabled == M_ENABLED)
  {
    if (data == M_DELETE) {
      markers[active_marker].enabled = false;
      menu_move_back();
//      ui_mode_normal();
//      return;
    } else if (data == M_NORMAL) {
      markers[active_marker].mtype = M_NORMAL;
    } else if (data == M_REFERENCE) {
      for (int i = 0; i<MARKER_COUNT; i++ ) {
        if (markers[i].mtype & M_REFERENCE)
          markers[i].mtype &= ~M_REFERENCE;
      }
      markers[active_marker].mtype |= M_REFERENCE;
      markers[active_marker].mtype &= ~M_DELTA;
    } else {
      if (data == M_DELTA && (markers[active_marker].mtype & M_REFERENCE))
        markers[active_marker].mtype &= ~M_REFERENCE;
      if (markers[active_marker].mtype & data)
        markers[active_marker].mtype &= ~data;
      else
        markers[active_marker].mtype |= data;
    }
  }
  markmap_all_markers();
//  redraw_marker(active_marker, TRUE);
//  menu_move_back();
  draw_menu();
}


const int rbwsel[]={0,3,10,30,100,300,600};

static void menu_rbw_cb(int item, uint8_t data)
{
  (void)item;
  set_RBW(rbwsel[data]);
  menu_move_back();
  ui_mode_normal();
//  draw_cal_status();
}

static void menu_unit_cb (int item, uint8_t data)
{
  (void)item;
  set_unit(data);
  menu_move_back();
  ui_mode_normal();
}

enum {
  S_20,S_10,S_5,S_2,S_1,S_P5,S_P2,S_P1,S_P05,S_P02,S_P01
};
static const float menu_scale_per_value[11]={20,10,5,2,1,0.5,0.2,0.1,0.05,0.02,0.01};

static void menu_scale_per_cb(int item, uint8_t data)
{
  (void)item;
  set_scale(menu_scale_per_value[data]);
  menu_move_back();
  ui_mode_normal();
//  draw_cal_status();
}

static void menu_trigger_cb(int item, uint8_t data)
{
  (void)item;
  set_trigger(data);
//  menu_move_back();
  ui_mode_normal();
  redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
  completed = true;
}

#if 0
static void choose_active_trace(void)
{
  int i;
  if (trace[uistat.current_trace].enabled)
    // do nothing
    return;
  for (i = 0; i < TRACE_COUNT ; i++)
    if (trace[i].enabled) {
      uistat.current_trace = i;
      return;
    }
}
#endif
static void choose_active_marker(void)
{
  int i;
  for (i = 0; i < MARKER_COUNT; i++)
    if (markers[i].enabled == M_ENABLED) {
      active_marker = i;
      return;
    }
  active_marker = -1;
}

#ifdef __ULTRA__
static void menu_harmonic_cb(int item, uint8_t data)
{
  (void)item;
  set_harmonic(data);
  draw_menu();
}
#endif

static void menu_settings2_cb(int item, uint8_t data)
{
  (void)item;
  switch(data) {
  case 1:
    toggle_AGC();
    break;
  case 2:
    toggle_LNA();;
    break;
  case 3:
    toggle_tracking();
    break;
  case 4:
    toggle_below_IF();
    break;
  case 5:
    toggle_tracking_output();
    break;
  }
  draw_menu();
//  draw_cal_status();
}

static void menu_pause_cb(int item, uint8_t data)
{
  (void) data;
  (void) item;
  toggle_sweep();
//  menu_move_back();
//  ui_mode_normal();
  draw_menu();
//  draw_cal_status();
}

static void menu_outputmode_cb(int item, uint8_t data)
{
  (void) data;
  (void) item;
  toggle_mute();
  draw_menu();
}

//const int menu_drive_value[]={5,10,15,20};
const char *menu_drive_text[]={"-38dBm","-35dBm","-33dBm","-30dBm","-27dBm","-24dBm","-21dBm","  -19dBm", "  -7dBm"," -4dBm"," -2dBm","  1dBm","  4dBm","  7dBm"," 10dBm"," 13dBm"};



// ===[MENU DEFINITION]=========================================================

static const menuitem_t menu_store_preset_high[8] =
{
  { MT_CALLBACK, 0,     "\2STORE\0STARTUP",menu_store_preset_cb},
  { MT_CALLBACK, 5,     "STORE 5"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 6,     "STORE 6"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 7,     "STORE 7"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 8,     "STORE 8"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 100,   "\2FACTORY\0DEFAULTS",menu_store_preset_cb},
  { MT_CANCEL,   255, "\032 BACK", NULL },
  { MT_NONE,     0,     NULL,            NULL } // sentinel
};

static const menuitem_t menu_load_preset_high[] =
{
  { MT_CALLBACK, 0,     "\2LOAD\0STARTUP",menu_load_preset_cb},
  { MT_CALLBACK, 5,     "LOAD 5"  ,      menu_load_preset_cb},
  { MT_CALLBACK, 6,     "LOAD 6"  ,      menu_load_preset_cb},
  { MT_CALLBACK, 7,     "LOAD 7"  ,      menu_load_preset_cb},
  { MT_CALLBACK, 8,     "LOAD 8"  ,      menu_load_preset_cb},
  { MT_SUBMENU,  0,     "STORE"  ,       menu_store_preset_high},
  { MT_CANCEL,   255, "\032 BACK", NULL },
  { MT_NONE,     0,     NULL,            NULL } // sentinel
};

static const menuitem_t menu_store_preset[] =
{
  { MT_CALLBACK, 0,     "\2STORE AS\0STARTUP",menu_store_preset_cb},
  { MT_CALLBACK, 1,     "STORE 1"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 2,     "STORE 2"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 3,     "STORE 3"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 4,     "STORE 4"  ,      menu_store_preset_cb},
  { MT_CALLBACK, 100,   "\2FACTORY\0DEFAULTS",menu_store_preset_cb},
  { MT_CANCEL,   255, "\032 BACK", NULL },
  { MT_NONE,     0,     NULL,            NULL } // sentinel
};

static const menuitem_t menu_load_preset[] =
{
  { MT_CALLBACK, 0,     "\2LOAD\0STARTUP",menu_load_preset_cb},
  { MT_CALLBACK, 1,     "LOAD 1"  ,      menu_load_preset_cb},
  { MT_CALLBACK, 2,     "LOAD 2"  ,      menu_load_preset_cb},
  { MT_CALLBACK, 3,     "LOAD 3"  ,      menu_load_preset_cb},
  { MT_CALLBACK, 4,     "LOAD 4"  ,      menu_load_preset_cb},
  { MT_SUBMENU,  0,     "STORE"  ,       menu_store_preset},
  { MT_CANCEL,   255, "\032 BACK", NULL },
  { MT_NONE,     0,     NULL,            NULL } // sentinel
};

static const menuitem_t menu_drive[] = {
  { MT_CALLBACK, 15, " 15dBm",   menu_drive_cb},
  { MT_CALLBACK, 14, " 12dBm",   menu_drive_cb},
  { MT_CALLBACK, 13, "  9dBm",   menu_drive_cb},
  { MT_CALLBACK, 12, "  6dBm",   menu_drive_cb},
  { MT_CANCEL,   255, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_drive_wide3[] = {
 { MT_FORM | MT_CALLBACK, 5, "-24dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 4, "-27dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 3, "-30dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 2, "-33dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 1, "-35dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 0, "-38dBm",   menu_drive_cb},
  { MT_FORM | MT_CANCEL,   255, "\032 BACK", NULL },
 { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_drive_wide2[] = {
 { MT_FORM | MT_CALLBACK, 10, " -2dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 9, " -4dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 8, " -7dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 7, "-19dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 6, "-21dBm",   menu_drive_cb},
 { MT_FORM | MT_SUBMENU,  255, "\033 MORE", menu_drive_wide3},
 { MT_FORM | MT_CANCEL,   255, "\032 BACK", NULL },
 { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_drive_wide[] = {
  { MT_FORM | MT_CALLBACK, 15, " 13dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 14, " 10dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 13, "  7dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 12, "  4dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 11, "  1dBm",   menu_drive_cb},
  { MT_FORM | MT_SUBMENU,  255, "\033 MORE", menu_drive_wide2},
  { MT_FORM | MT_CANCEL,   255, "\032 BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

const menuitem_t  menu_modulation[] = {
  { MT_FORM | MT_TITLE,    0,  "MODULATION",NULL},
  { MT_FORM | MT_CALLBACK, MO_NONE,     "NONE",      menu_modulation_cb},
  { MT_FORM | MT_CALLBACK | MT_LOW, MO_AM_1kHz,  "AM 1kHz",   menu_modulation_cb},
  { MT_FORM | MT_CALLBACK | MT_LOW, MO_AM_10Hz,  "AM 10Hz",   menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, MO_NFM,      "NARROW FM", menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, MO_WFM,      "WIDE FM",   menu_modulation_cb},
  { MT_FORM | MT_CALLBACK | MT_LOW, MO_EXTERNAL, "EXTERNAL",  menu_modulation_cb},
  { MT_FORM | MT_CANCEL,   0,             "\032 BACK",NULL },
  { MT_FORM | MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t  menu_lowoutputmode[] = {
  { MT_FORM | MT_CALLBACK, 0,                   "LOW OUTPUT              %s",       menu_outputmode_cb},
  { MT_FORM | MT_KEYPAD,   KM_CENTER,           "FREQ: %s",         "10kHz..350MHz"},
  { MT_FORM | MT_KEYPAD,   KM_LOWOUTLEVEL,      "LEVEL: %s",        "-76..-6"},
  { MT_FORM | MT_SUBMENU,  0,                   "MODULATION: %s",   menu_modulation},
  { MT_FORM | MT_KEYPAD,   KM_SPAN,             "SPAN: %s",         "0..350MHz"},
  { MT_FORM | MT_KEYPAD | MT_LOW, KM_LEVELSWEEP,"LEVEL CHANGE: %s",   "-70..70"},
  { MT_FORM | MT_KEYPAD,   KM_SWEEP_TIME,       "SWEEP TIME: %s",   "0..600S"},
  //  { MT_FORM | MT_KEYPAD,   KM_10MHZ,        "10MHz: %s",         NULL},
  { MT_FORM | MT_CANCEL,   0,           "MODE",                     NULL },
  { MT_FORM | MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t  menu_highoutputmode[] = {
  { MT_FORM | MT_CALLBACK,  0,          "HIGH OUTPUT             %s",      menu_outputmode_cb},
  { MT_FORM | MT_KEYPAD,    KM_CENTER,  "FREQ: %s",         "240MHz..960MHz"},
  { MT_FORM | MT_SUBMENU,   0,          "LEVEL: %s",        menu_drive_wide},
  { MT_FORM | MT_SUBMENU,   0,          "MODULATION: %s",   menu_modulation},
  { MT_FORM | MT_KEYPAD,    KM_SPAN,    "SPAN: %s",         NULL},
  { MT_FORM | MT_CANCEL,    0,          "MODE",             NULL },
  { MT_FORM | MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t  menu_average[] = {
  { MT_CALLBACK, 0, "OFF",            menu_average_cb},
  { MT_CALLBACK, 1, "\2MIN\0HOLD",    menu_average_cb},
  { MT_CALLBACK, 2, "\2MAX\0HOLD",    menu_average_cb},
  { MT_CALLBACK, 3, "\2MAX\0DECAY",   menu_average_cb},
  { MT_CALLBACK, 4, "AVER 4",         menu_average_cb},
  { MT_CALLBACK, 5, "AVER 16",        menu_average_cb},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t menu_rbw[] = {
  { MT_CALLBACK, 0, "  AUTO",   menu_rbw_cb},
  { MT_CALLBACK, 1, "  3kHz",   menu_rbw_cb},
  { MT_CALLBACK, 2, " 10kHz",   menu_rbw_cb},
  { MT_CALLBACK, 3, " 30kHz",   menu_rbw_cb},
  { MT_CALLBACK, 4, "100kHz",   menu_rbw_cb},
  { MT_CALLBACK, 5, "300kHz",   menu_rbw_cb},
  { MT_CALLBACK, 6, "600kHz",   menu_rbw_cb},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,      0, NULL, NULL } // sentinel
};


static const menuitem_t menu_scale_per2[] = {
  { MT_CALLBACK, 6, "0.1 /",   menu_scale_per_cb},
  { MT_CALLBACK, 7, "0.2 /",   menu_scale_per_cb},
  { MT_CALLBACK, 8, "0.05/",   menu_scale_per_cb},
  { MT_CALLBACK, 9, "0.02/",   menu_scale_per_cb},
  { MT_CALLBACK,10, "0.01/",   menu_scale_per_cb},
//  { MT_CALLBACK,11, "0.005/",   menu_scale_per_cb},
//  { MT_SUBMENU,  0, "\033 MORE",    menu_scale_per2},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_scale_per[] = {
  { MT_CALLBACK, 0, " 20/",   menu_scale_per_cb},
  { MT_CALLBACK, 1, " 10/",   menu_scale_per_cb},
  { MT_CALLBACK, 2, "  5/",   menu_scale_per_cb},
  { MT_CALLBACK, 3, "  2/",   menu_scale_per_cb},
  { MT_CALLBACK, 4, "  1/",   menu_scale_per_cb},
  { MT_CALLBACK, 5, "0.5/",   menu_scale_per_cb},
  { MT_SUBMENU,  0, "\033 MORE",    menu_scale_per2},

  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_reffer2[] = {
  { MT_FORM | MT_CALLBACK, 5, "3MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 6, "2MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 7, "1MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};


static const menuitem_t menu_reffer[] = {
  { MT_FORM | MT_CALLBACK, 0, "OFF"  ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 1, "30MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 2, "15MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 3, "10MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 4, "4MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 6, "2MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 7, "1MHz" ,   menu_reffer_cb},
//  { MT_FORM | MT_SUBMENU,  0, "\033 MORE", menu_reffer2},
  { MT_FORM | MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_atten[] = {
  { MT_CALLBACK,0,               "AUTO",           menu_atten_cb},
  { MT_KEYPAD, KM_ATTENUATION,  "MANUAL",         "0..30"},
  { MT_CANCEL, 0,               "\032 BACK", NULL },
  { MT_FORM | MT_NONE,   0, NULL, NULL } // sentinel
};


static const menuitem_t menu_reflevel[] = {
  { MT_CALLBACK,0,          "AUTO",    menu_reflevel_cb},
  { MT_KEYPAD,  KM_REFLEVEL,  "MANUAL",     NULL},
  { MT_CANCEL, 0,           "\032 BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_search[] = {
  //{ MT_CALLBACK, "OFF", menu_marker_search_cb },
  { MT_CALLBACK, 0, "\2MIN\0" "\032 LEFT",  menu_marker_search_cb },
  { MT_CALLBACK, 1, "\2MIN\0" "\033 RIGHT", menu_marker_search_cb },
  { MT_CALLBACK, 2, "\2MAX\0" "\032 LEFT",  menu_marker_search_cb },
  { MT_CALLBACK, 3, "\2MAX\0" "\033 RIGHT", menu_marker_search_cb },
  { MT_CALLBACK, 4, "TRACKING",                 menu_marker_search_cb },
  { MT_CANCEL, 0, "\032 BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_modify[] = {
  { MT_CALLBACK, M_REFERENCE,   "REFER",        menu_marker_modify_cb},
  { MT_CALLBACK, M_DELTA,       "DELTA",        menu_marker_modify_cb},
  { MT_CALLBACK, M_NOISE,       "NOISE",        menu_marker_modify_cb},
  { MT_CALLBACK, M_TRACKING,    "TRACKING",     menu_marker_modify_cb},
  { MT_CALLBACK, M_NORMAL,      "NORMAL",       menu_marker_modify_cb},
  { MT_SUBMENU,  0,             "SEARCH",       menu_marker_search},
  { MT_CALLBACK, M_DELETE,      "DELETE",       menu_marker_modify_cb},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK, 1, "MARKER 1", menu_marker_sel_cb },
  { MT_CALLBACK, 2, "MARKER 2", menu_marker_sel_cb },
  { MT_CALLBACK, 3, "MARKER 3", menu_marker_sel_cb },
  { MT_CALLBACK, 4, "MARKER 4", menu_marker_sel_cb },
//  { MT_CALLBACK, 0, "ALL OFF", menu_marker_sel_cb },
  { MT_CALLBACK, 0, "DELTA", menu_marker_sel_cb },
  { MT_CALLBACK, 0, "NOISE", menu_marker_sel_cb },
  { MT_CALLBACK, 0, "TRACKING", menu_marker_sel_cb },
  { MT_CANCEL, 0, "\032 BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_select[] = {
  { MT_CALLBACK, 1, "MARKER 1", menu_marker_select_cb },
  { MT_CALLBACK, 2, "MARKER 2", menu_marker_select_cb },
  { MT_CALLBACK, 3, "MARKER 3", menu_marker_select_cb },
  { MT_CALLBACK, 4, "MARKER 4", menu_marker_select_cb },
  { MT_CANCEL, 0, "\032 BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};


const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, ST_START, "\033START", menu_marker_op_cb },
  { MT_CALLBACK, ST_STOP, "\033STOP", menu_marker_op_cb },
  { MT_CALLBACK, ST_CENTER, "\033CENTER", menu_marker_op_cb },
  { MT_CALLBACK, ST_SPAN, "\033SPAN", menu_marker_op_cb },
  { MT_CANCEL, 0, "\032 BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};



static const menuitem_t menu_marker[] = {
//  { MT_SUBMENU,  0, "\2SELECT\0MARKER",     menu_marker_sel},
  { MT_SUBMENU,  0, "\2MODIFY\0MARKERS",    menu_marker_select},
  { MT_SUBMENU,  0, "\2MARKER\0OPS",        menu_marker_ops},
  { MT_SUBMENU,  0, "\2SEARCH\0MARKER",     menu_marker_search},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_dfu[] = {
  { MT_FORM | MT_CALLBACK, 0, "ENTER DFU",      menu_dfu_cb},
  { MT_FORM | MT_CANCEL,   0, "\032 BACK",  NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

#ifdef __ULTRA__
static const menuitem_t menu_harmonic[] =
{
  { MT_CALLBACK, 2,     "2",                  menu_harmonic_cb},
  { MT_CALLBACK, 3,     "3",                  menu_harmonic_cb},
  { MT_CALLBACK, 4,     "4",                  menu_harmonic_cb},
  { MT_CALLBACK, 5,     "5",                  menu_harmonic_cb},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};
#endif


static const menuitem_t menu_scanning_speed[] =
{
 { MT_CALLBACK, 0,             "FAST",      menu_scanning_speed_cb},
 { MT_CALLBACK, 1,             "PRECISE",   menu_scanning_speed_cb},
 { MT_KEYPAD, KM_SAMPLETIME,   "\2SAMPLE\0DELAY",   "300..30000"},
 { MT_CANCEL,   0,             "\032 BACK", NULL },
 { MT_NONE,     0, NULL, NULL } // sentinel
};


static const menuitem_t menu_settings2[] =
{
  { MT_CALLBACK, 1,             "AGC",              menu_settings2_cb},
  { MT_CALLBACK, 2,             "LNA",              menu_settings2_cb},
  { MT_CALLBACK | MT_LOW, 3,    "BPF",              menu_settings2_cb},
  { MT_CALLBACK | MT_LOW, 4,    "\2BELOW\0IF",      menu_settings2_cb},
  { MT_KEYPAD,   KM_DECAY,      "\2HOLD\0SWEEPS",   "1..1000"},
  { MT_KEYPAD,   KM_NOISE,      "\2NOISE\0LEVEL",   "2..20"},
#ifdef __ULTRA__
  { MT_SUBMENU,0,               "HARMONIC",         menu_harmonic},
#endif
//  { MT_KEYPAD, KM_10MHZ,"\00210MHz\0ACTUAL",    NULL},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settings[] =
{
  { MT_CALLBACK | MT_LOW, 5,    "\2LO\0OUTPUT",menu_settings2_cb},
  { MT_KEYPAD, KM_ACTUALPOWER,  "\2ACTUAL\0POWER",  NULL},
  { MT_KEYPAD | MT_LOW, KM_IF,  "\2IF\0FREQ",       NULL},
  { MT_SUBMENU,0,               "\2SCAN\0SPEED",         menu_scanning_speed},
  { MT_KEYPAD, KM_REPEAT,       "\2SAMPLE\0REPEAT",          "1..100"},
  { MT_SUBMENU | MT_LOW,0,      "\2MIXER\0DRIVE",      menu_drive},
  { MT_SUBMENU,  0,             "\033 MORE",    menu_settings2},
  { MT_CANCEL,   0,             "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_measure[] = {
  { MT_CALLBACK,            M_OFF,        "OFF",              menu_measure_cb},
  { MT_CALLBACK,            M_IMD,        "HARMONIC",         menu_measure_cb},
  { MT_CALLBACK,            M_OIP3,       "OIP3",             menu_measure_cb},
  { MT_CALLBACK,            M_PHASE_NOISE,"\2PHASE\0NOISE",   menu_measure_cb},
  { MT_CALLBACK,            M_STOP_BAND,  "\2STOP\0BAND",     menu_measure_cb},
  { MT_CALLBACK,            M_PASS_BAND,  "\2PASS\0BAND",     menu_measure_cb},
  { MT_CALLBACK | MT_LOW,   M_LINEARITY,  "LINEAR",           menu_measure_cb},
  { MT_CANCEL, 0,               "\032 BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_calibrate[] =
{
 { MT_FORM | MT_TITLE,      0, "CONNECT INPUT AND OUTPUT",  NULL},
 { MT_FORM | MT_CALLBACK,   0, "CALIBRATE",                 menu_calibrate_cb},
 { MT_FORM | MT_CALLBACK,   0, "RESET CALBRATION",          menu_calibrate_cb},
 { MT_FORM | MT_CANCEL,     0, "\032 BACK",             NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_config[] = {
  { MT_CALLBACK, 0, "\2TOUCH\0CAL",     menu_config_cb},
  { MT_CALLBACK, 0, "\2TOUCH\0TEST",    menu_config_cb},
  { MT_CALLBACK, 0, "\2SELF\0TEST",     menu_config_cb},
  { MT_SUBMENU,  0, "\2LEVEL\0CAL",     menu_calibrate},
  { MT_CALLBACK, 0, "VERSION",          menu_config_cb},
  { MT_SUBMENU,  0, "\2EXPERT\0CONFIG", menu_settings},
  { MT_SUBMENU,  0, "\033DFU",  menu_dfu},
  { MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_display[] = {
  { MT_CALLBACK,0,              "\2PAUSE\0SWEEP",   menu_pause_cb},
  { MT_CALLBACK,0,              "\2STORE\0TRACE",   menu_storage_cb},
  { MT_CALLBACK,1,              "\2CLEAR\0STORED",  menu_storage_cb},
  { MT_CALLBACK,2,              "\2SUBTRACT\0STORED",menu_storage_cb},
  { MT_CALLBACK,3,              "WATERFALL",        menu_storage_cb},
  { MT_KEYPAD,  KM_SWEEP_TIME,  "\2SWEEP\0TIME",    NULL},

  { MT_CANCEL, 0,           "\032 BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_unit[] =
{
 { MT_CALLBACK,U_DBM,       "dBm",              menu_unit_cb},
 { MT_CALLBACK,U_DBMV,      "dBmV",             menu_unit_cb},
 { MT_CALLBACK,U_DBUV,      "dBuV",             menu_unit_cb},
 { MT_CALLBACK,U_VOLT,      "Volt",             menu_unit_cb},
// { MT_CALLBACK,U_UVOLT,     "uVolt",             menu_unit_cb},
 { MT_CALLBACK,U_WATT,      "Watt",             menu_unit_cb},
// { MT_CALLBACK,U_UWATT,    "uWatt",             menu_unit_cb},
  { MT_CANCEL, 0,           "\032 BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_trigger[] = {
  { MT_CALLBACK,T_AUTO,      "AUTO",              menu_trigger_cb},
  { MT_CALLBACK,T_NORMAL,    "NORMAL",             menu_trigger_cb},
  { MT_CALLBACK,T_SINGLE,    "SINGLE",             menu_trigger_cb},
  { MT_KEYPAD,  KM_TRIGGER,   "LEVEL",            NULL},
  { MT_CANCEL, 0,           "\032 BACK",NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_level[] = {
  { MT_SUBMENU, 0,          "\2REF\0LEVEL", menu_reflevel},
//  { MT_SUBMENU, 0,          "\2SCALE/\0DIV",menu_scale_per},
  { MT_KEYPAD,  KM_SCALE,   "\2SCALE/\0DIV",NULL},
  { MT_SUBMENU | MT_LOW, 0, "ATTEN",        menu_atten},
  { MT_SUBMENU,0,           "CALC",         menu_average},
  { MT_SUBMENU, 0,          "UNIT",         menu_unit},
  { MT_KEYPAD,  KM_OFFSET,  "\2EXTERN\0AMP",NULL},
  { MT_SUBMENU,  0,         "TRIGGER",      menu_trigger},
  { MT_CANCEL, 0,           "\032 BACK",NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_stimulus[] = {
  { MT_KEYPAD,  KM_START,   "START",            NULL},
  { MT_KEYPAD,  KM_STOP,    "STOP",             NULL},
  { MT_KEYPAD,  KM_CENTER,  "CENTER",           NULL},
  { MT_KEYPAD,  KM_SPAN,    "SPAN",             NULL},
  { MT_KEYPAD,  KM_CW,      "\2ZERO\0SPAN",          NULL},
  { MT_SUBMENU,0,           "RBW",              menu_rbw},
#ifdef __SPUR__
  { MT_CALLBACK | MT_LOW,0,           "\2SPUR\0REMOVAL", menu_spur_cb},
#endif
  { MT_CANCEL,  0,          "\032 BACK", NULL },
  { MT_NONE,    0, NULL, NULL } // sentinel
};



static const menuitem_t menu_mode[] = {
//  { MT_FORM | MT_TITLE,                 0,                      "tinySA MODE",           NULL},
  { MT_FORM | MT_CALLBACK | MT_ICON,    I_LOW_INPUT+I_SA,       "%s TO LOW INPUT",      menu_mode_cb},
  { MT_FORM | MT_CALLBACK | MT_ICON,    I_HIGH_INPUT+I_SA,      "%s TO HIGH INPUT",     menu_mode_cb},
  { MT_FORM | MT_CALLBACK | MT_ICON,    I_LOW_OUTPUT+I_SINUS,   "%s TO LOW OUTPUT",     menu_mode_cb},
  { MT_FORM | MT_CALLBACK | MT_ICON,    I_HIGH_OUTPUT+I_GEN,    "%s TO HIGH OUTPUT",    menu_mode_cb},
  { MT_FORM | MT_SUBMENU  | MT_ICON,    I_CONNECT+I_GEN,        "CAL OUTPUT: %s", menu_reffer},
#ifdef __ULTRA__
  { MT_FORM | MT_CALLBACK | MT_ICON,    I_LOW_INPUT+I_SA,       "ULTRA HIGH INPUT",menu_mode_cb},
#endif
//  { MT_FORM | MT_CANCEL,   0, "\032 BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

#ifdef __ULTRA__
const menuitem_t menu_topultra[] = {
  { MT_CALLBACK, 0, "RESET",        menu_autosettings_cb},
  { MT_SUBMENU,  0, "FREQ",         menu_stimulus},
  { MT_SUBMENU,  0, "LEVEL",        menu_level},
  { MT_SUBMENU,  0, "DISPLAY",      menu_display},
  { MT_SUBMENU,  0, "MARKER",       menu_marker},
  { MT_SUBMENU,  0, "MEASURE",      menu_measure},
  { MT_SUBMENU,  0, "SETTINGS",     menu_settings},
  { MT_CANCEL,   0, "MODE",NULL},
  { MT_NONE,     0, NULL, NULL } // sentinel,
 // MENUITEM_CLOSE,
};
#endif

const menuitem_t menu_top[] = {
  { MT_SUBMENU, 0,  "PRESET",       menu_load_preset},
  { MT_SUBMENU,  0, "FREQ",         menu_stimulus},
  { MT_SUBMENU,  0, "LEVEL",        menu_level},
  { MT_SUBMENU,  0, "DISPLAY",      menu_display},
  { MT_SUBMENU,  0, "MARKER",       menu_marker},
  { MT_SUBMENU,  0, "MEASURE",      menu_measure},
  { MT_SUBMENU,  0, "CONFIG",       menu_config},
  { MT_SUBMENU,  0, "MODE",         menu_mode},
  { MT_NONE,     0, NULL, NULL } // sentinel,
 // MENUITEM_CLOSE,
};

// ===[MENU DEFINITION END]======================================================

#define ACTIVE_COLOR RGBHEX(0x007FFF)

void frequency_string(char *buf, size_t len, int32_t freq);

int menu_is_form(const menuitem_t *menu)
{
  int i;
  for (i = 0; MT_MASK(menu[i].type) != MT_NONE; i++)
    if (menu[i].type & MT_FORM)
      return (true);
  return(false);
}


static void menu_item_modify_attribute(
    const menuitem_t *menu, int item, uint16_t *fg, uint16_t *bg)
{
  int mark = false;
  int m_auto = false;
  int data = menu[item].data;
  if (menu == menu_mode) {
    if (item == setting.mode)  {
      plot_printf(uistat.text, sizeof uistat.text, "RETURN");
      mark = true;
    } else if (item < 4){
      plot_printf(uistat.text, sizeof uistat.text, "SWITCH");
    }  else if (item == 4) {
      plot_printf(uistat.text, sizeof uistat.text, menu_reffer_text[setting.refer+1]);
    }
  } else if (menu == menu_highoutputmode && item == 2) {
      plot_printf(uistat.text, sizeof uistat.text, menu_drive_text[setting.drive]);
  } else if (menu == menu_lowoutputmode || menu == menu_highoutputmode) {
    if (item == 0) {
      if (setting.mute)
        strcpy(uistat.text, "OFF");
      else
        strcpy(uistat.text, "ON");
      mark = true;
    }
    if (item == 3) {
      plot_printf(uistat.text, sizeof uistat.text, menu_modulation_text[setting.modulation]);
    }
  } else if (menu == menu_reffer) {
    if (item < 5 && item == setting.refer + 1){
      mark = true;
   }
  } else if (menu == menu_reffer2) {
    if (item == setting.refer - 4){
      mark = true;
    }
  } else if (menu == menu_stimulus) {
    if (item == 6 && setting.spur) {
      mark = true;
    }
  } else if (menu == menu_average) {
    if (item == setting.average){
      mark = true;
    }
  } else if (menu == menu_scale_per) {
    if (menu_scale_per_value[data] == setting.scale){
      mark = true;
    }
  } else if (menu == menu_measure && MT_MASK(menu[item].type) == MT_CALLBACK) {
    if (data == setting.measurement){
      mark = true;
    }
  } else if (menu == menu_rbw) {
    if (rbwsel[item] == setting.rbw){
      mark = true;
    }

  } else if (MT_MASK(menu[item].type) == MT_CALLBACK && menu == menu_unit) {
    if (data == setting.unit){
      mark = true;
    }
  } else if (MT_MASK(menu[item].type) == MT_CALLBACK && (menu == menu_drive || menu == menu_drive_wide || menu == menu_drive_wide2|| menu == menu_drive_wide3)) {
    if (data == setting.drive){
      mark = true;
    }
  } else if (menu == menu_modulation && MT_MASK(menu[item].type) == MT_CALLBACK) {
    if (data == setting.modulation){
      mark = true;
    }
  } else if (menu == menu_trigger && MT_MASK(menu[item].type) == MT_CALLBACK) {
    if (data == setting.trigger){
      mark = true;
    }
  } else if (menu == menu_display /* || menu == menu_displayhigh */) {
    if (item ==0 && is_paused()){
      mark = true;
    }
    if (item ==1 && setting.show_stored){
      mark = true;
    }
    if (item == 3 && setting.subtract_stored){
      mark = true;
    }
    if (item == 4 && get_waterfall()){
      mark = true;
    }
#ifdef __SPUR__
    if (item == 5 && setting.spur) {
      mark = true;
    }
#endif
  } else if (menu == menu_settings) {
    if (item ==0 && setting.tracking_output){
      mark = true;
    } else if (item == 2 && setting.auto_IF)
      m_auto = true;
  } else if (menu == menu_scanning_speed) {
    if (item == setting.step_delay){
      mark = true;
    } else if (item == 2 && setting.step_delay > 1) {
      mark = true;
    }
#ifdef __ULTRA__
  } else if (MT_MASK(menu[item].type) == MT_CALLBACK && menu == menu_harmonic) {
    if (data == setting.harmonic)
      mark = true;
#endif
  } else if (MT_MASK(menu[item].type) == MT_CALLBACK && menu == menu_settings2) {
    int v=0;
    switch(data) {
    case 1: v = setting.agc; break;
    case 2: v = setting.lna; break;
    case 3: v = setting.tracking; break;
    case 4: v = setting.below_IF; break;
    }
    if (S_IS_AUTO(v))
      m_auto = true;
    else if (v == S_ON)
      mark = true;
  } else if (menu == menu_marker_modify && active_marker >= 0 && markers[active_marker].enabled == M_ENABLED) {
    if (data & markers[active_marker].mtype)
      mark = true;
    else if (item < 5 && data==markers[active_marker].mtype)    // This catches the M_NORMAL case
      mark = true;
  } else if (menu == menu_marker_search) {
    if (item == 4 && markers[active_marker].mtype & M_TRACKING)
      mark = true;
  } else if (menu == menu_marker_sel || menu == menu_marker_select) {
    if (item < 4 && markers[item].enabled)
      mark = true;
    else if (item == 4 && uistat.marker_delta)
      mark = true;
    else if (item == 5 && uistat.marker_noise)
      mark = true;
    else if (item == 6 && uistat.marker_tracking)
      mark = true;
  } else if (menu == menu_reflevel) {
    if ((item  == 0 && setting.auto_reflevel) || (item == 1 && !setting.auto_reflevel))
      mark = true;
  } else if (menu == menu_atten) {
    if ((item  == 0 && setting.auto_attenuation ) || (item  == 1 && !setting.auto_attenuation))
      mark = true;
  }
  if (m_auto) {
    *bg = LIGHT_GREY;
    *fg = config.menu_normal_color;
  } else if (mark) {
    *bg = DEFAULT_MENU_TEXT_COLOR;
    *fg = config.menu_normal_color;
  }
  if (ui_mode == UI_MENU && menu_is_form(menu)) {
    //    if (item == 0)
    //      redraw_frame();
    if (item <= 1) {
      area_width = 0;
    }
  }else{
    area_width = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
  }
}

static void fetch_numeric_target(void)
{
  switch (keypad_mode) {
  case KM_START:
    uistat.value = get_sweep_frequency(ST_START);
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_STOP:
    uistat.value = get_sweep_frequency(ST_STOP);
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_CENTER:
    uistat.value = get_sweep_frequency(ST_CENTER);
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_SPAN:
    uistat.value = get_sweep_frequency(ST_SPAN);
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_CW:
    uistat.value = get_sweep_frequency(ST_CW);
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_SCALE:
    uistat.value = setting.scale;
    plot_printf(uistat.text, sizeof uistat.text, "%f/", uistat.value);
    break;
  case KM_REFLEVEL:
    uistat.value = setting.reflevel;
    plot_printf(uistat.text, sizeof uistat.text, "%f", uistat.value);
    break;
  case KM_ATTENUATION:
    uistat.value = get_attenuation();
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", ((int32_t)uistat.value));
     break;
  case KM_ACTUALPOWER:
    uistat.value = get_level_offset();
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", ((int32_t)uistat.value));
    break;
  case KM_IF:
    uistat.value = setting.frequency_IF;
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_SAMPLETIME:
    uistat.value = setting.step_delay;
    plot_printf(uistat.text, sizeof uistat.text, "%3duS", ((int32_t)uistat.value));
    break;
  case KM_REPEAT:
    uistat.value = setting.repeat;
    plot_printf(uistat.text, sizeof uistat.text, "%2d", ((int32_t)uistat.value));
    break;
  case KM_DRIVE:
    uistat.value = setting.drive;
    plot_printf(uistat.text, sizeof uistat.text, "%3ddB", ((int32_t)uistat.value));
    break;
  case KM_LOWOUTLEVEL:
    uistat.value = get_attenuation();           // compensation for dB offset during low output mode
    int end_level =  ((int32_t)uistat.value)+setting.level_sweep;
    if (end_level < -76)
      end_level = -76;
    if (end_level > -6)
      end_level = -6;
    if (setting.level_sweep != 0)
      plot_printf(uistat.text, sizeof uistat.text, "%ddBm to %ddBm", ((int32_t)uistat.value), end_level);
    else
      plot_printf(uistat.text, sizeof uistat.text, "%ddBm", ((int32_t)uistat.value));
    break;
  case KM_DECAY:
    uistat.value = setting.decay;
    plot_printf(uistat.text, sizeof uistat.text, "%3d", ((int32_t)uistat.value));
    break;
  case KM_NOISE:
    uistat.value = setting.noise;
    plot_printf(uistat.text, sizeof uistat.text, "%3d", ((int32_t)uistat.value));
    break;
  case KM_10MHZ:
    uistat.value = setting_frequency_10mhz;
    plot_printf(uistat.text, sizeof uistat.text, "%3.6fMHz", uistat.value / 1000000.0);
    break;
  case KM_OFFSET:
    uistat.value = setting.offset;
    plot_printf(uistat.text, sizeof uistat.text, "%.1fdB", uistat.value);
    break;
  case KM_LEVELSWEEP:
    uistat.value = setting.level_sweep;
    plot_printf(uistat.text, sizeof uistat.text, "%.1fdB", uistat.value);
    break;
  case KM_SWEEP_TIME:
    if (setting.sweep_time < calc_min_sweep_time())
      uistat.value = calc_min_sweep_time();
    else
      uistat.value = setting.sweep_time;
    uistat.value /= 1000.0;
    plot_printf(uistat.text, sizeof uistat.text, "%.3FS", uistat.value);
    break;
  case KM_TRIGGER:
    uistat.value = setting.trigger_level;
    plot_printf(uistat.text, sizeof uistat.text, "%.1fdB", uistat.value);
    break;

  }
  
  {
    uint32_t x = uistat.value;
    int n = 0;
    for (; x >= 10 && n < 9; n++)
      x /= 10;
    uistat.digit = n;
  }
//  uistat.previous_value = uistat.value;
}

static void
set_numeric_value(void)
{
  switch (keypad_mode) {
  case KM_START:
    set_sweep_frequency(ST_START, uistat.value);
    break;
  case KM_STOP:
    set_sweep_frequency(ST_STOP, uistat.value);
    break;
  case KM_CENTER:
    set_sweep_frequency(ST_CENTER, uistat.value);
    break;
  case KM_SPAN:
    setting.modulation = MO_NONE;
    set_sweep_frequency(ST_SPAN, uistat.value);
    break;
  case KM_CW:
    set_sweep_frequency(ST_CW, uistat.value);
    break;
  case KM_SCALE:
    user_set_scale(uistat.value);
    break;
  case KM_REFLEVEL:
    user_set_reflevel(uistat.value);
    break;
  case KM_ATTENUATION:
    setting.auto_attenuation = false;
    set_attenuation(uistat.value);
    break;
  case KM_ACTUALPOWER:
    set_actual_power(uistat.value);
    config_save();
    break;
  case KM_IF:
    setting.auto_IF = false;
    set_IF(uistat.value);
//    config_save();
    break;
  case KM_SAMPLETIME:
    set_step_delay(uistat.value);
    break;
  case KM_REPEAT:
    set_repeat(uistat.value);
    break;
  case KM_DRIVE:
    set_drive(uistat.value);
    break;
  case KM_LOWOUTLEVEL:
    set_level(uistat.value);
    break;
  case KM_DECAY:
    set_decay(uistat.value);
    break;
  case KM_NOISE:
    set_noise(uistat.value);
    break;
  case KM_10MHZ:
    if (uistat.value < 9000000) {
      set_10mhz(setting_frequency_10mhz + uistat.value);
    } else
      set_10mhz(uistat.value);
    dirty = true;
    break;
  case KM_OFFSET:
    set_offset(uistat.value);
    break;
  case KM_LEVELSWEEP:
    setting.modulation = MO_NONE;
    set_level_sweep(uistat.value);
    break;
  case KM_SWEEP_TIME:
    set_sweep_time(uistat.value*1000.0);
    update_grid();
    break;
  case KM_TRIGGER:
    if (setting.trigger == T_AUTO )
      set_trigger(T_NORMAL);
    set_trigger_level(to_dBm(uistat.value));
    redraw_request |= REDRAW_CAL_STATUS | REDRAW_AREA;
    completed = true;

    break;
  }
}

void
menu_move_top(void)
{
  while (menu_current_level > 0)
    menu_move_back();
}

