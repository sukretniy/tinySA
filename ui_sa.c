
void markmap_all_markers(void);
static void menu_marker_type_cb(int item, uint8_t data);

void set_sweep_frequency(int type, uint32_t frequency);
uint32_t get_sweep_frequency(int type);
void clearDisplay(void);
void reset_settings(int);
//void ui_process_touch(void);
void SetPowerGrid(int);
void SetRefLevel(int);
void set_refer_output(int);
int get_refer_output(void);
void SetAttenuation(int);
int GetAttenuation(void);
void SetPowerLevel(int);
void SetGenerate(int);
void SetRBW(int);
void SetDrive(int d);
void SetIF(int f);
void SetStepDelay(int t);
extern int setting_rbw;
void SetSpur(int);
int GetSpur(void);
void SetAverage(int);
int GetAverage(void);
extern int setting_average;
void  SetStorage(void);
void  SetClearStorage(void);
void  SetSubtractStorage(void);
void toggle_waterfall(void);
void SetMode(int);
int GetMode(void);
void SetRefpos(int);
void SetScale(int);
void AllDirty(void);
void MenuDirty(void);
void ToggleLNA(void);
void ToggleAGC(void);
void redrawHisto(void);
void self_test(void);
void set_decay(int);
void set_noise(int);
extern int32_t frequencyExtra;
extern int setting_tracking;
extern int setting_drive;
extern int setting_lna;
extern int setting_agc;
extern int setting_decay;
extern int setting_noise;
void SetModulation(int);
extern int setting_modulation;
void set_measurement(int);
// extern int settingSpeed;
extern int setting_step_delay;



enum {
  KM_START=1, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_REFPOS, KM_SCALE, KM_ATTENUATION,
  KM_ACTUALPOWER, KM_IF, KM_SAMPLETIME, KM_DRIVE, KM_LOWOUTLEVEL, KM_DECAY, KM_NOISE
};



#define KP_X(x) (48*(x) + 2 + (320-BUTTON_WIDTH-192))
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
#define KP_N 21
#define KP_P 22


typedef struct {
  uint8_t x:4;
  uint8_t y:4;
  int8_t  c;
} keypads_t;

static const keypads_t *keypads;

static uint8_t keypads_last_index;


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

static const keypads_t keypads_scale[] = {
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

static const keypads_t keypads_level[] = {
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
  { 3, 2, KP_MINUS },
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
  keypads_level, // refpos
  keypads_scale, // scale
  keypads_scale, // attenuation
  keypads_level, // actual power
  keypads_freq, // IF
  keypads_level, // sample time
  keypads_scale, // drive
  keypads_level,    // KM_LOWOUTLEVEL
  keypads_level,    // KM_DECAY
  keypads_level,    // KM_NOISE
};

#ifdef __VNA__
static const char * const keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY"
};
#endif
#ifdef __SA__
static const char * const keypad_mode_label[] = {
  "error", "START", "STOP", "CENTER", "SPAN", "CW FREQ", "REFPOS", "SCALE", "\2ATTENUATE\0 0-31dB", "ACTUALPOWER", "IF", "SAMPLE TIME", "DRIVE", "LEVEL", "LEVEL", "LEVEL"
};
#endif


// ===[MENU CALLBACKS]=========================================================


int generator_enabled = false;

extern const menuitem_t  menu_lowoutputmode[];
extern const menuitem_t  menu_highoutputmode[];
extern const menuitem_t  menu_modulation[];
extern const menuitem_t  menu_top[];
extern const menuitem_t  menu_tophigh[];

static void menu_mode_cb(int item, uint8_t data)
{
  (void)data;
  SetMode(item-1);
//  draw_cal_status();
  switch (item) {
  case 1:
    menu_push_submenu(menu_top);
    break;
  case 2:
    menu_push_submenu(menu_tophigh);
    break;
  case 3:
    menu_push_submenu(menu_lowoutputmode);
    break;
  case 4:
    menu_push_submenu(menu_highoutputmode);
    break;
  }
//  draw_cal_status();
}

extern int dirty;
void menu_autosettings_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  reset_settings(GetMode());

  active_marker = 0;
  for (int i = 1; i<MARKER_COUNT; i++ ) {
    markers[i].enabled = M_DISABLED;
  }
  markers[0].enabled = M_TRACKING_ENABLED;
  markers[0].mtype = M_REFERENCE;

  //  set_refer_output(1);

  //  SetPowerLevel(100); // Reset
  SetClearStorage();
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
    menu_move_back();
    ui_mode_normal();
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


const int menu_modulation_value[]={MO_NONE,MO_AM, MO_NFM, MO_WFM};
const char *menu_modulation_text[]={"NONE","AM","NARROW FM","WIDE FM"};

static void menu_modulation_cb(int item, uint8_t data)
{
  (void)item;
//Serial.println(item);
  SetModulation(menu_modulation_value[data]);
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
  SetDrive(data);
  menu_move_back();
//  ui_mode_normal();
//  draw_cal_status();
}



#if 0

static void menu_spur_cb(int item, uint8_t data)
{
  (void)data;
  (void)item;
  if (GetSpur())
    SetSpur(0);
  else
    SetSpur(1); // must be 0 or 1 !!!!
//  menu_move_back();
  ui_mode_normal();
  draw_cal_status();
}
#endif

static void menu_measure_cb(int item, uint8_t data)
{
  (void)item;
  switch(data) {
    case M_OFF:                                     // Off
      reset_settings(GetMode());
      set_measurement(M_OFF);
      break;
    case M_IMD:                                     // IMD
      reset_settings(GetMode());
      for (int i = 0; i< MARKERS_MAX; i++) {
        markers[i].enabled = M_TRACKING_ENABLED;
        markers[i].mtype = M_DELTA;
      }
      markers[0].mtype = M_REFERENCE;
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      set_sweep_frequency(ST_START, 0);
      set_sweep_frequency(ST_STOP, uistat.value*5);
      set_measurement(M_IMD);
      break;
    case M_OIP3:                                     // OIP3
      reset_settings(GetMode());
      for (int i = 0; i< MARKERS_MAX; i++) {
        markers[i].enabled = M_TRACKING_ENABLED;
        markers[i].mtype = M_DELTA;
      }
      markers[0].mtype = M_REFERENCE;
      ui_mode_keypad(KM_CENTER);
      ui_process_keypad();
      ui_mode_keypad(KM_SPAN);
      ui_process_keypad();
      set_sweep_frequency(ST_SPAN, uistat.value*4);
      set_measurement(M_OIP3);
      break;
  }
  menu_move_back();
  ui_mode_normal();
//  draw_cal_status();
}

static void menu_storage_cb(int item, uint8_t data)
{
  (void)item;
  switch(data) {
    case 0:
      SetStorage();
      break;
    case 1:
      SetClearStorage();
      break;
    case 2:
      SetSubtractStorage();
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
  SetAverage(item);
  menu_move_back();
  ui_mode_normal();
  draw_cal_status();
}

static void menu_marker_type_cb(int item, uint8_t data)
{
  (void)item;
  if (markers[active_marker].enabled)
  {
    if (data == M_REFERENCE) {
      for (int i = 0; i<MARKER_COUNT; i++ ) {
        if (markers[i].mtype ==M_REFERENCE)
          markers[i].mtype = M_NORMAL;
      }
    }
    if (data == M_TRACKING) {
      if (markers[active_marker].enabled == M_ENABLED)
        markers[active_marker].enabled = M_TRACKING_ENABLED;
      else
        markers[active_marker].enabled = M_ENABLED;
    } else
      markers[active_marker].mtype = data;
  }
  markmap_all_markers();
//  redraw_marker(active_marker, TRUE);
  menu_move_back();
  draw_menu();
}


const int rbwsel[]={0,3,10,30,100,300};

static void menu_rbw_cb(int item, uint8_t data)
{
  (void)item;
  SetRBW(rbwsel[data]);
  menu_move_back();
  ui_mode_normal();
//  draw_cal_status();
}

int menu_dBper_value[]={1,2,5,10,20};

static void menu_dBper_cb(int item, uint8_t data)
{
  (void)item;
  SetScale(menu_dBper_value[data]);
  menu_move_back();
  ui_mode_normal();
//  draw_cal_status();
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
    if (markers[i].enabled) {
      active_marker = i;
      return;
    }
  active_marker = -1;
}

static void menu_settings2_cb(int item, uint8_t data)
{
  (void)item;
  switch(data) {
  case 0:
    ToggleAGC();
    break;
  case 1:
    ToggleLNA();;
    break;
  case 2:
    toggle_tracking();
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
  menu_move_back();
  ui_mode_normal();
  draw_menu();
//  draw_cal_status();
}

//const int menu_drive_value[]={5,10,15,20};
const char *menu_drive_text[]={"-30dBm","-27dBm","-24dBm","-21dBm","-18dBm","-15dBm","-12dBm","  -9dBm", " -6dBm"," -3dBm","  0dBm","  3dBm","  6dBm"," 10dBm"," 14dBm"," 18dBm"};



// ===[MENU DEFINITION]=========================================================

static const menuitem_t menu_drive[] = {
  { MT_CALLBACK, 7, " 20dBm",   menu_drive_cb},
  { MT_CALLBACK, 6, " 16dBm",   menu_drive_cb},
  { MT_CALLBACK, 5, " 12dBm",   menu_drive_cb},
  { MT_CALLBACK, 4, "  8dBm",   menu_drive_cb},
  { MT_CANCEL,   255, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_drive_wide3[] = {
 { MT_FORM | MT_CALLBACK, 5, "-15dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 4, "-18dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 3, "-21dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 2, "-24dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 1, "-27dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 0, "-30dBm",   menu_drive_cb},
  { MT_FORM | MT_CANCEL,   255, S_LARROW" BACK", NULL },
 { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_drive_wide2[] = {
 { MT_FORM | MT_CALLBACK, 10, "  0dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 9, " -3dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 8, " -6dBm",   menu_drive_cb},
 { MT_FORM | MT_CALLBACK, 7, " -9dBm",   menu_drive_cb  },
 { MT_FORM | MT_CALLBACK, 6, "-12dBm",   menu_drive_cb},
 { MT_FORM | MT_SUBMENU,  255, S_RARROW" MORE", menu_drive_wide3},
 { MT_FORM | MT_CANCEL,   255, S_LARROW" BACK", NULL },
 { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_drive_wide[] = {
  { MT_FORM | MT_CALLBACK, 15, " 18dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 14, " 14dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 13, " 10dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 12, "  6dBm",   menu_drive_cb},
  { MT_FORM | MT_CALLBACK, 11, "  3dBm",   menu_drive_cb},
  { MT_FORM | MT_SUBMENU,  255, S_RARROW" MORE", menu_drive_wide2},
  { MT_FORM | MT_CANCEL,   255, S_LARROW" BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

const menuitem_t  menu_modulation[] = {
  { MT_FORM | MT_TITLE,    0,  "MODULATION",NULL},
  { MT_FORM | MT_CALLBACK, 0,  "NONE",      menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, 1,  "AM",        menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, 2,  "NARROW FM", menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, 3,  "WIDE FM",   menu_modulation_cb},
  { MT_FORM | MT_CANCEL,   0,             S_LARROW" BACK",NULL },
  { MT_FORM | MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t  menu_lowoutputmode[] = {
  { MT_FORM | MT_TITLE,    0,               "LOW OUTPUT",       NULL},
  { MT_FORM | MT_KEYPAD,   KM_CENTER,       "FREQ: %s",         NULL},
  { MT_FORM | MT_KEYPAD,   KM_LOWOUTLEVEL,  "LEVEL: %s",        NULL},
  { MT_FORM | MT_SUBMENU,  0,               "MODULATION: %s",   menu_modulation},
  { MT_FORM | MT_KEYPAD,   KM_SPAN,         "SPAN: %s",         NULL},
  { MT_FORM | MT_CANCEL,   0,           S_LARROW" BACK",    NULL },
  { MT_FORM | MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t  menu_highoutputmode[] = {
  { MT_FORM | MT_TITLE,     0,          "HIGH OUTPUT",      NULL},
  { MT_FORM | MT_KEYPAD,    KM_CENTER,  "FREQ: %s",         NULL},
  { MT_FORM | MT_SUBMENU,   0,          "LEVEL: %s",        menu_drive_wide},
  { MT_FORM | MT_SUBMENU,   0,          "MODULATION: %s",   menu_modulation},
  { MT_FORM | MT_KEYPAD,    KM_SPAN,    "SPAN: %s",         NULL},
  { MT_FORM | MT_CANCEL,    0,          S_LARROW" BACK",NULL },
  { MT_FORM | MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t  menu_average[] = {
  { MT_CALLBACK, 0, "OFF",             menu_average_cb},
  { MT_CALLBACK, 1, "\2MIN\0HOLD",    menu_average_cb},
  { MT_CALLBACK, 2, "\2MAX\0HOLD",    menu_average_cb},
  { MT_CALLBACK, 3, "\2MAX\0DECAY",   menu_average_cb},
  { MT_CALLBACK, 4, "AVER 4",           menu_average_cb},
  { MT_CALLBACK, 5, "AVER 16",          menu_average_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};
#if 0
static const menuitem_t menu_storage[] = {
  { MT_CALLBACK, 0, "STORE",    menu_storage_cb},
  { MT_CALLBACK, 1, "CLEAR",    menu_storage_cb},
  { MT_CALLBACK, 2, "SUBTRACT", menu_storage_cb},
  { MT_CALLBACK, 3, "WATERFALL",menu_storage_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,      0, NULL, NULL } // sentinel
};
#endif

static const menuitem_t menu_rbw[] = {
  { MT_CALLBACK, 0, "  AUTO",   menu_rbw_cb},
  { MT_CALLBACK, 1, "  3kHz",   menu_rbw_cb},
  { MT_CALLBACK, 2, " 10kHz",   menu_rbw_cb},
  { MT_CALLBACK, 3, " 30kHz",   menu_rbw_cb},
  { MT_CALLBACK, 4, "100kHz",   menu_rbw_cb},
  { MT_CALLBACK, 5, "300kHz",   menu_rbw_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,      0, NULL, NULL } // sentinel
};


static const menuitem_t menu_dBper[] = {
  { MT_CALLBACK, 0, "  1dB/",   menu_dBper_cb},
  { MT_CALLBACK, 1, "  2dB/",   menu_dBper_cb},
  { MT_CALLBACK, 2, "  5dB/",   menu_dBper_cb},
  { MT_CALLBACK, 3, " 10dB/",   menu_dBper_cb},
  { MT_CALLBACK, 4, " 20dB/",   menu_dBper_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_reffer2[] = {
  { MT_FORM | MT_CALLBACK, 5, "3MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 6, "2MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 7, "1MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_reffer[] = {
  { MT_FORM | MT_CALLBACK, 0, "OFF"  ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 1, "30MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 2, "15MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 3, "10MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 4, "4MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_SUBMENU,  0, S_RARROW" MORE", menu_reffer2},
  { MT_FORM | MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_acquire[] = {
  { MT_CALLBACK, 0, "AUTO",     menu_autosettings_cb},
  { MT_KEYPAD, KM_ATTENUATION,  "ATTEN",         NULL},
  { MT_SUBMENU,0,               "RBW",           menu_rbw},
  { MT_SUBMENU,0,               "CALC",          menu_average},
  { MT_CANCEL, 0,               S_LARROW" BACK", NULL },
  { MT_FORM | MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_acquirehigh[] = {
  { MT_CALLBACK, 0, "AUTO",     menu_autosettings_cb},
  { MT_SUBMENU,0,               "RBW",              menu_rbw},
  { MT_SUBMENU,0,               "CALC",          menu_average},
  { MT_CANCEL, 0,               S_LARROW" BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};


static const menuitem_t menu_display[] = {
  { MT_KEYPAD, KM_REFPOS,       "\2REF\0LEVEL",     NULL},
  { MT_SUBMENU,0,               "\2SCALE/\0DIV",    menu_dBper},
  { MT_CALLBACK, 0, "STORE",    menu_storage_cb},
  { MT_CALLBACK, 1, "CLEAR",    menu_storage_cb},
  { MT_CALLBACK, 2, "SUBTRACT", menu_storage_cb},
  { MT_CALLBACK, 3, "WATERFALL",menu_storage_cb},
  { MT_CANCEL, 0,               S_LARROW" BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

#if 0
static const menuitem_t menu_scale[] = {
  { MT_KEYPAD, KM_REFPOS,       "\2REF\0LEVEL",     NULL},
  { MT_SUBMENU,0,               "\2SCALE/\0DIV",    menu_dBper},
  { MT_KEYPAD, KM_ATTENUATION,  "ATTEN",            NULL},
  { MT_SUBMENU,0,               "AVERAGE",          menu_average},
  { MT_KEYPAD, 0,               "\2SPUR\0REDUCT.",  menu_spur_cb},
  { MT_SUBMENU,0,               "RBW",              menu_rbw},
  { MT_CANCEL, 0,               S_LARROW" BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_scalehigh[] = {
  { MT_KEYPAD, KM_REFPOS,   "\2REF\0LEVEL",  NULL},
  { MT_SUBMENU,0,           "\2SCALE/\0DIV", menu_dBper},
  { MT_SUBMENU,0,           "AVERAGE",       menu_average},
  { MT_SUBMENU,0,           "RBW",           menu_rbw},
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

#endif

static const menuitem_t menu_stimulus[8] = {
  { MT_KEYPAD,  KM_START,   "START",            NULL},
  { MT_KEYPAD,  KM_STOP,    "STOP",             NULL},
  { MT_KEYPAD,  KM_CENTER,  "CENTER",           NULL},
  { MT_KEYPAD,  KM_SPAN,    "SPAN",             NULL},
  { MT_KEYPAD,  KM_CW,      "CW FREQ",          NULL},
  { MT_CALLBACK,0,          "\2PAUSE\0SWEEP",   menu_pause_cb},
  { MT_CANCEL,  0,          S_LARROW" BACK", NULL },
  { MT_NONE,    0, NULL, NULL } // sentinel
};


static const menuitem_t menu_marker_type[] = {
  { MT_CALLBACK, M_REFERENCE,   "REFERENCE",    menu_marker_type_cb},
  { MT_CALLBACK, M_NORMAL,      "NORMAL",       menu_marker_type_cb},
  { MT_CALLBACK, M_DELTA,       "DELTA",        menu_marker_type_cb},
  { MT_CALLBACK, M_TRACKING,    "TRACKING",     menu_marker_type_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_search[] = {
  //{ MT_CALLBACK, "OFF", menu_marker_search_cb },
  { MT_CALLBACK, 0, "MAXIMUM",                      menu_marker_search_cb },
  { MT_CALLBACK, 1, "MINIMUM",                      menu_marker_search_cb },
  { MT_CALLBACK, 2, "\2SEARCH\0" S_LARROW" LEFT",   menu_marker_search_cb },
  { MT_CALLBACK, 3, "\2SEARCH\0" S_RARROW" RIGHT",  menu_marker_search_cb },
  { MT_CALLBACK, 4, "TRACKING",                     menu_marker_search_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK, 0, "MARKER 1",             menu_marker_sel_cb},
  { MT_CALLBACK, 0, "MARKER 2",             menu_marker_sel_cb},
  { MT_CALLBACK, 0, "MARKER 3",             menu_marker_sel_cb},
  { MT_CALLBACK, 0, "MARKER 4",             menu_marker_sel_cb},
  { MT_CALLBACK, 0, "ALL OFF",              menu_marker_sel_cb},
  { MT_SUBMENU,  0, "\2SEARCH\0MARKER",     menu_marker_search},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_marker[] = {
  { MT_SUBMENU,  0, "\2SELECT\0MARKER",     menu_marker_sel},
  { MT_SUBMENU,  0, "\2MARKER\0TYPE",       menu_marker_type},
  { MT_CALLBACK, 0, S_RARROW"START",        menu_marker_op_cb},
  { MT_CALLBACK, 0, S_RARROW"STOP",         menu_marker_op_cb},
  { MT_CALLBACK, 0, S_RARROW"CENTER",       menu_marker_op_cb},
  { MT_CALLBACK, 0, S_RARROW"SPAN",         menu_marker_op_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_dfu[] = {
  { MT_FORM | MT_CALLBACK, 0, "ENTER DFU",      menu_dfu_cb},
  { MT_FORM | MT_CANCEL,   0, S_LARROW" BACK",  NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settings2[] =
{
  { MT_CALLBACK, 0, "AGC",              menu_settings2_cb},
  { MT_CALLBACK, 1, "LNA",              menu_settings2_cb},
  { MT_CALLBACK, 2, "BPF",              menu_settings2_cb},
  { MT_KEYPAD, KM_DECAY,                "\2HOLD\0TIME",   NULL},
  { MT_KEYPAD, KM_NOISE,                "\2NOISE\0LEVEL",   NULL},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settings[] =
{
  { MT_KEYPAD, KM_ACTUALPOWER,    "\2ACTUAL\0POWER",  NULL},
  { MT_KEYPAD, KM_IF,             "\2IF\0FREQ",       NULL},
  { MT_KEYPAD, KM_SAMPLETIME,     "\2SAMPLE\0TIME",   NULL},
  { MT_SUBMENU,0,                 "\2LO\0DRIVE",      menu_drive},
  { MT_SUBMENU,  0,                 S_RARROW" MORE",    menu_settings2},
  { MT_CANCEL,   0,                 S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_measure[] = {
  { MT_CALLBACK, M_OFF,  "OFF",      menu_measure_cb},
  { MT_CALLBACK, M_IMD,  "IMD",      menu_measure_cb},
  { MT_CALLBACK, M_OIP3, "OIP3",     menu_measure_cb},
  { MT_CANCEL, 0,               S_LARROW" BACK", NULL },
  { MT_NONE,   0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settingshigh2[] =
{
  { MT_CALLBACK, 0, "AGC",              menu_settings2_cb},
  { MT_CALLBACK, 1, "LNA",              menu_settings2_cb},
  { MT_KEYPAD, KM_DECAY,                "\2HOLD\0TIME",   NULL},
  { MT_KEYPAD, KM_NOISE,                "\2NOISE\0LEVEL",   NULL},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settingshigh[] =
{
  { MT_KEYPAD, KM_ACTUALPOWER,    "\2ACTUAL\0POWER",  NULL},
  { MT_KEYPAD, KM_SAMPLETIME,     "\2SAMPLE\0TIME",   NULL},
  { MT_SUBMENU,  0,                 S_RARROW" MORE",    menu_settingshigh2},
  { MT_CANCEL,   0,                 S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_calibrate[] =
{
 { MT_FORM | MT_TITLE,      0, "CONNECT INPUT AND OUTPUT",  NULL},
 { MT_FORM | MT_CALLBACK,   0, "CALIBRATE",                 menu_calibrate_cb},
 { MT_FORM | MT_CALLBACK,   0, "RESET CALBRATION",          menu_calibrate_cb},
 { MT_FORM | MT_CANCEL,     0, S_LARROW" BACK",             NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_config[] = {
  { MT_FORM | MT_CALLBACK, 0, "TOUCH CAL",     menu_config_cb},
  { MT_FORM | MT_CALLBACK, 0, "TOUCH TEST",    menu_config_cb},
  { MT_FORM | MT_CALLBACK, 0, "SELF TEST",     menu_config_cb},
  { MT_FORM | MT_SUBMENU,  0, "CALIBRATE",     menu_calibrate},
  { MT_FORM | MT_CALLBACK, 0, "VERSION",       menu_config_cb},
//  { MT_SUBMENU,  0, "SETTINGS",         menu_settings},
//  { MT_SUBMENU,  0, "RBW", menu_rbw},
  { MT_FORM | MT_SUBMENU,  0, S_RARROW"DFU",  menu_dfu},
  { MT_FORM | MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_mode[] = {
  { MT_FORM | MT_TITLE,    0, "MODE",           NULL},
  { MT_FORM | MT_CALLBACK, 0, "LOW INPUT",      menu_mode_cb},
  { MT_FORM | MT_CALLBACK, 0, "HIGH INPUT",     menu_mode_cb},
  { MT_FORM | MT_CALLBACK, 0, "LOW OUTPUT",     menu_mode_cb},
  { MT_FORM | MT_CALLBACK, 0, "HIGH OUTPUT",    menu_mode_cb},
  { MT_FORM | MT_SUBMENU,  0, "CAL OUTPUT: %s", menu_reffer},
  { MT_FORM | MT_SUBMENU,  0, "CONFIG",         menu_config},
//  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_FORM | MT_NONE,     0, NULL, NULL } // sentinel
};
#if 1
const menuitem_t menu_top[] = {
  { MT_SUBMENU,  0, "ACQUIRE",      menu_acquire},
  { MT_SUBMENU,  0, "SCAN",         menu_stimulus},
  { MT_SUBMENU,  0, "DISPLAY",      menu_display},
  { MT_SUBMENU,  0, "MARKER",       menu_marker},
  { MT_SUBMENU,  0, "MEASURE",      menu_measure},
  { MT_SUBMENU,  0, "SETTINGS",     menu_settings},
  { MT_CANCEL,   0, S_LARROW" MODE",NULL},
  { MT_NONE,     0, NULL, NULL } // sentinel,
 // MENUITEM_CLOSE,
};

const menuitem_t menu_tophigh[] =
{
 { MT_SUBMENU,  0, "ACQUIRE",      menu_acquirehigh},
 { MT_SUBMENU,  0, "SCAN",         menu_stimulus},
 { MT_SUBMENU,  0, "DISPLAY",      menu_display},
 { MT_SUBMENU,  0, "MARKER",       menu_marker},
 { MT_SUBMENU,  0, "MEASURE",      menu_measure},
 { MT_SUBMENU,  0, "SETTINGS",     menu_settings},
 { MT_CANCEL,   0, S_LARROW" MODE",NULL},
 { MT_NONE,     0, NULL, NULL } // sentinel,
 // MENUITEM_CLOSE,
};
#else
const menuitem_t menu_top[] = {
  { MT_CALLBACK, 0, "AUTO",         menu_autosettings_cb},
  { MT_SUBMENU,  0, "SCAN",         menu_stimulus},
  { MT_SUBMENU,  0, "MARKER",       menu_marker},
  { MT_SUBMENU,  0, "DISPLAY",      menu_scale},
  { MT_SUBMENU,  0, "STORAGE",      menu_storage},
  { MT_SUBMENU,  0, "SETTINGS",     menu_settings},
  { MT_CANCEL,   0, S_LARROW" MODE",NULL},
  { MT_NONE,     0, NULL, NULL } // sentinel,
 // MENUITEM_CLOSE,
};

const menuitem_t menu_tophigh[] = {
  { MT_CALLBACK, 0, "AUTO",         menu_autosettings_cb},
  { MT_SUBMENU,  0, "SCAN",         menu_stimulus},
  { MT_SUBMENU,  0, "MARKER",       menu_marker},
  { MT_SUBMENU,  0, "DISPLAY",      menu_scalehigh},
  { MT_SUBMENU,  0, "STORAGE",      menu_storage},
  { MT_SUBMENU,  0, "SETTINGS",     menu_settingshigh},
  { MT_CANCEL,   0, S_LARROW" MODE",NULL},
  { MT_NONE,     0, NULL, NULL } // sentinel,
 // MENUITEM_CLOSE,
};
#endif
// ===[MENU DEFINITION END]======================================================

#undef BOARD_NAME
#define BOARD_NAME  "tinySA"

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
  if (menu == menu_mode) {
    if (item == GetMode()+1)  {
      mark = true;
    } else if (item == 5) {
      plot_printf(uistat.text, sizeof uistat.text, menu_reffer_text[get_refer_output()+1]);
    }
  } else if (menu == menu_highoutputmode && item == 2) {
      plot_printf(uistat.text, sizeof uistat.text, menu_drive_text[setting_drive]);
  } else if (menu == menu_lowoutputmode || menu == menu_highoutputmode) {
    if (item == 3) {
      plot_printf(uistat.text, sizeof uistat.text, menu_modulation_text[setting_modulation]);
    }
  } else if (menu == menu_reffer) {
    if (item < 5 && item == get_refer_output() + 1){
      mark = true;
   }
  } else if (menu == menu_reffer2) {
    if (item == get_refer_output() - 4){
      mark = true;
    }
  } else if (menu == menu_stimulus) {
    if (item == 5 /* PAUSE */ && !(sweep_mode&SWEEP_ENABLE)) {
      mark = true;
    }
  } else if (menu == menu_average) {
    if (item == GetAverage()){
      mark = true;
    }
  } else if (menu == menu_dBper) {
    if (menu_dBper_value[item] == get_trace_scale(1)){
      mark = true;
    }
  } else if (menu == menu_rbw) {
    if (rbwsel[item] == GetRBW()){
      mark = true;
    }

  } else if (menu == menu_drive || menu == menu_drive_wide || menu == menu_drive_wide2|| menu == menu_drive_wide3) {
    if (menu[item].data == setting_drive){
      mark = true;
    }

  } else if (menu == menu_display) {
    if (item ==2 && GetStorage()){
      mark = true;
    }
    if (item == 4 && GetSubtractStorage()){
      mark = true;
    }
    if (item == 5 && get_waterfall()){
      mark = true;
    }
  } else if (menu == menu_settings2 || menu == menu_settingshigh2) {
    if (item ==0 && setting_agc){
      mark = true;
    }
    if (item == 1 && setting_lna){
      mark = true;
    }
    if (item == 2 && setting_tracking){         // should not happen in high mode
      mark = true;
    }
  } else if (menu == menu_marker_type && active_marker >= 0 && markers[active_marker].enabled) {
    if (item == 3 && markers[active_marker].enabled == M_TRACKING_ENABLED)
      mark = true;
    else if (item == markers[active_marker].mtype)
      mark = true;
  } else if (menu == menu_marker_sel) {
    if (item < MARKERS_MAX && markers[item].enabled)
      mark = true;
  }
  if (mark) {
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
    uistat.value = get_trace_scale(uistat.current_trace) * 1000;
    plot_printf(uistat.text, sizeof uistat.text, "%ddB/", uistat.value / 1000);
    break;
  case KM_REFPOS:
    uistat.value = get_trace_refpos(uistat.current_trace) * 1000;
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", uistat.value / 1000);
    break;
  case KM_ATTENUATION:
    uistat.value = GetAttenuation();
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", uistat.value);
     break;
  case KM_ACTUALPOWER:
    uistat.value = settingLevelOffset();
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", uistat.value);
    break;
  case KM_IF:
    uistat.value = frequency_IF;
    plot_printf(uistat.text, sizeof uistat.text, "%3.3fMHz", uistat.value / 1000000.0);
    break;
  case KM_SAMPLETIME:
    uistat.value = setting_step_delay;
    plot_printf(uistat.text, sizeof uistat.text, "%3duS", uistat.value);
    break;
  case KM_DRIVE:
    uistat.value = setting_drive;
    plot_printf(uistat.text, sizeof uistat.text, "%3ddB", uistat.value);
    break;
  case KM_LOWOUTLEVEL:
    uistat.value = GetAttenuation();           // compensation for dB offset during low output mode
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", uistat.value);
    break;
  case KM_DECAY:
    uistat.value = setting_decay;
    plot_printf(uistat.text, sizeof uistat.text, "%3d", uistat.value);
    break;
  case KM_NOISE:
    uistat.value = setting_noise;
    plot_printf(uistat.text, sizeof uistat.text, "%3d", uistat.value);
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
    set_sweep_frequency(ST_SPAN, uistat.value);
    break;
  case KM_CW:
    set_sweep_frequency(ST_CW, uistat.value);
    break;
  case KM_SCALE:
    set_trace_scale(0, uistat.value / 1000.0);
    set_trace_scale(1, uistat.value / 1000.0);
    set_trace_scale(2, uistat.value / 1000.0);
    break;
  case KM_REFPOS:
    SetRefpos(uistat.value);
    break;
  case KM_ATTENUATION:
    SetAttenuation(uistat.value);
    break;
  case KM_ACTUALPOWER:
    SetPowerLevel(uistat.value);
    config_save();
    break;
  case KM_IF:
    SetIF(uistat.value);
    config_save();
    break;
  case KM_SAMPLETIME:
    SetStepDelay(uistat.value);
    break;
  case KM_DRIVE:
    SetDrive(uistat.value);
    break;
  case KM_LOWOUTLEVEL:
    SetAttenuation(uistat.value);
    break;
  case KM_DECAY:
    set_decay(uistat.value);
    break;
  case KM_NOISE:
    set_noise(uistat.value);
    break;
  }
}
