
void markmap_all_markers(void);
static void menu_marker_type_cb(int item, uint8_t data);

void set_sweep_frequency(int type, uint32_t frequency);
uint32_t get_sweep_frequency(int type);
void clearDisplay(void);
//void ui_process_touch(void);
void SetPowerGrid(int);
void SetRefLevel(int);
void set_refer_output(int);
int get_refer_output(void);
void SetAttenuation(int);
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
extern int32_t frequencyExtra;
extern int setting_tracking;
extern int setting_drive;
extern int setting_lna;
extern int setting_agc;
void SetModulation(int);
extern int setting_modulation;
// extern int settingSpeed;
extern int setting_step_delay;



enum {
  KM_START=1, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_REFPOS, KM_SCALE, KM_ATTENUATION,
  KM_ACTUALPOWER, KM_IF, KM_SAMPLETIME, KM_DRIVE, KM_LOWOUTLEVEL, KM_HIGHOUTLEVEL
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
  keypads_level,    // KM_HIGHOUTLEVEL
};

#ifdef __VNA__
static const char * const keypad_mode_label[] = {
  "START", "STOP", "CENTER", "SPAN", "CW FREQ", "SCALE", "REFPOS", "EDELAY", "VELOCITY%", "DELAY"
};
#endif
#ifdef __SA__
static const char * const keypad_mode_label[] = {
  "error", "START", "STOP", "CENTER", "SPAN", "CW FREQ", "REFPOS", "SCALE", "ATTENUATION", "ACTUALPOWER", "IF", "SAMPLE TIME", "DRIVE", "LEVEL", "LEVEL"
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
  draw_cal_status();
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
  draw_cal_status();
}

extern int dirty;
void menu_autosettings_cb(int item, uint8_t data)
{
  (void)item;
  (void)data;
  int current_mode = GetMode();
  SetMode(-1);              // Force setmode to do something
  SetMode(current_mode);

  active_marker = 0;
  for (int i = 1; i<MARKER_COUNT; i++ ) {
    markers[i].enabled = false;
  }
  markers[0].enabled = true;
  markers[0].mtype = M_REFERENCE;

  //  set_refer_output(1);

  //  SetPowerLevel(100); // Reset
  SetClearStorage();
  dirty = true;
  //  menu_move_back();   // stay in input menu
  ui_mode_normal();
  draw_cal_status();
}

static void menu_calibrate_cb(int item, uint8_t data)
{
  (void)data;
  switch (item) {
  case 1:
    calibrate();
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
    self_test();
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

int menu_modulation_value[]={0, MO_NONE,MO_AM, MO_NFM, MO_WFM};
char *menu_modulation_text[]={"NONE","AM","NARROW FM","WIDE FM"};
static void menu_modulation_cb(int item, uint8_t data)
{
  (void)data;
//Serial.println(item);
  SetModulation(menu_modulation_value[item]);
  menu_move_back();
//  ui_mode_normal();   // Stay in menu mode
  draw_cal_status();
}


const int menu_reffer_value[]={-1,0,1,2,3,4,5,6};
const char *menu_reffer_text[]={"OFF","30MHz","15MHz","10MHz","4MHz","3MHz","2MHz","1MHz"};
static void menu_reffer_cb(int item, uint8_t data)
{
  (void)data;
//Serial.println(item);
  set_refer_output(menu_reffer_value[item]);
  menu_move_back();
//  ui_mode_normal();   // Stay in menu mode
  draw_cal_status();
}

static void menu_reffer_cb2(int item, uint8_t data)
{
  (void)data;
//Serial.println(item);
  set_refer_output(menu_reffer_value[item+5]);
  menu_move_back();
  //  ui_mode_normal();   // Stay in menu mode
  draw_cal_status();
}

static void menu_spur_cb(int item, uint8_t data)
{
  (void)data;
  (void)item;
#if 0
  if (GetSpur())
    SetSpur(0);
  else
    SetSpur(1); // must be 0 or 1 !!!!
//  menu_move_back();
  ui_mode_normal();
  draw_cal_status();
#endif
}

static void menu_storage_cb(int item, uint8_t data)
{
  (void)data;
  switch(item) {
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
  draw_cal_status();
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
  (void)data;
  if (markers[active_marker].enabled)
  {
    if (item == M_REFERENCE) {
      for (int i = 0; i<MARKER_COUNT; i++ ) {
        if (markers[i].mtype ==M_REFERENCE)
          markers[i].mtype = M_NORMAL;
      }
    }
    markers[active_marker].mtype = item;
  }
  markmap_all_markers();
//  redraw_marker(active_marker, TRUE);
  menu_move_back();
  draw_menu();
}


const int rbwsel[]={0,3,10,30,100,300};

static void menu_rbw_cb(int item, uint8_t data)
{
  (void)data;
  SetRBW(rbwsel[item]); 
  menu_move_back();
  ui_mode_normal();
  draw_cal_status();
}

int menu_dBper_value[]={1,2,5,10,20};

static void menu_dBper_cb(int item, uint8_t data)
{
  (void)data;
  SetScale(menu_dBper_value[item]);
  menu_move_back();
  ui_mode_normal();
  draw_cal_status();
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

static void menu_scale_cb(int item, uint8_t data)
{
  (void)item;
  int status;
  status = btn_wait_release();
  if (status & EVT_BUTTON_DOWN_LONG) {
    ui_mode_numeric(data);
//    ui_process_numeric();
  } else {
    ui_mode_keypad(data);
    ui_process_keypad();
  }
  draw_cal_status();
}

static void menu_lowoutputmode_cb(int item, uint8_t data)
{
  int status;
  int km = data;
  (void) item;
//  if (km == KM_SCALE && trace[uistat.current_trace].type == TRC_DELAY) {
//    km = KM_SCALEDELAY;
//  }

  status = btn_wait_release();

  if (item == 3) {
    menu_push_submenu(menu_modulation);
  } else
  {
    if (status & EVT_BUTTON_DOWN_LONG) {
      ui_mode_numeric(km);
      //    ui_process_numeric();
    } else {
      area_width = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
      redraw_frame();         // Remove form numbers
      ui_mode_keypad(km);
      ui_process_keypad();
    }
  }
  draw_cal_status();
}
#if 0
static void menu_highoutputmode_cb(int item, uint8_t data)
{
  int status;
  int km = data;
  (void) item;

//  if (km == KM_SCALE && trace[uistat.current_trace].type == TRC_DELAY) {
//    km = KM_SCALEDELAY;
//  }
  status = btn_wait_release();
  if (status & EVT_BUTTON_DOWN_LONG) {
    ui_mode_numeric(km);
//    ui_process_numeric();
  } else {
    area_width = AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH;
    redraw_frame();         // Remove form numbers
    ui_mode_keypad(km);
    ui_process_keypad();
  }
  draw_cal_status();
}
#endif

static void menu_settings_cb(int item, uint8_t data)
{
  (void)item;
  int status;
  status = btn_wait_release();
  if (status & EVT_BUTTON_DOWN_LONG) {
    ui_mode_numeric(data);
    //    ui_process_numeric();
  } else {
    ui_mode_keypad(data);
    ui_process_keypad();
  }
  draw_cal_status();
}

static void menu_settings2_cb(int item, uint8_t data)
{
  (void)data;
  switch(item) {
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
  draw_cal_status();
  draw_menu();
}

static void menu_stimulus_cb(int item, uint8_t data)
{
  (void) data;
  int status;
  int km = item+KM_START;
  switch (km) {
  case KM_START: /* START */
  case KM_STOP: /* STOP */
  case KM_CENTER: /* CENTER */
  case KM_SPAN: /* SPAN */
  case KM_CW: /* CW */
    status = btn_wait_release();
    if (status & EVT_BUTTON_DOWN_LONG) {
      ui_mode_numeric(km);
//      ui_process_numeric();
    } else {
      ui_mode_keypad(km);
      ui_process_keypad();
    }
    break;
  case KM_CW+1: /* PAUSE */
    toggle_sweep();
    menu_move_back();
    ui_mode_normal();
    draw_menu();
    break;
  }
  draw_cal_status();
}

// ===[MENU DEFINITION]=========================================================

const menuitem_t  menu_modulation[] = {
  { MT_FORM | MT_TITLE,    0,  "MODULATION",NULL},
  { MT_FORM | MT_CALLBACK, 0,  "NONE",      menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, 0,  "AM",        menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, 0,  "NARROW FM", menu_modulation_cb},
  { MT_FORM | MT_CALLBACK, 0,  "WIDE FM",   menu_modulation_cb},
  { MT_FORM | MT_CANCEL,   0,             S_LARROW" BACK",NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t  menu_lowoutputmode[] = {
  { MT_FORM | MT_TITLE,    0,             "LOW OUTPUT",  NULL},
  { MT_FORM | MT_CALLBACK, KM_CENTER,     "FREQ: %s",    menu_lowoutputmode_cb},
  { MT_FORM | MT_CALLBACK, KM_LOWOUTLEVEL, "LEVEL: %s",   menu_lowoutputmode_cb},
  { MT_FORM | MT_SUBMENU,  0,              "MODULATION: %s",   menu_modulation},
  { MT_FORM | MT_CANCEL,   0,             S_LARROW" BACK",NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

const menuitem_t  menu_highoutputmode[] = {
  { MT_FORM | MT_TITLE,    0,             "HIGH OUTPUT", NULL},
  { MT_FORM | MT_CALLBACK, KM_CENTER,     "FREQ: %s",    menu_lowoutputmode_cb},    // same menu as low mode
  { MT_FORM | MT_CALLBACK, KM_HIGHOUTLEVEL,      "LEVEL: %s",   menu_lowoutputmode_cb},
  { MT_FORM | MT_SUBMENU,  0,              "MODULATION: %s",   menu_modulation},
  { MT_FORM | MT_CANCEL,   0,             S_LARROW" BACK",NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t  menu_average[] = {
  { MT_CALLBACK, 0, "OFF",   menu_average_cb},
  { MT_CALLBACK, 0, "MIN",   menu_average_cb},
  { MT_CALLBACK, 0, "MAX",   menu_average_cb},
  { MT_CALLBACK, 0, " 2 ",   menu_average_cb},
  { MT_CALLBACK, 0, " 4 ",   menu_average_cb},
  { MT_CALLBACK, 0, " 8 ",   menu_average_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t menu_storage[] = {
  { MT_CALLBACK, 0, "STORE",    menu_storage_cb},
  { MT_CALLBACK, 0, "CLEAR",    menu_storage_cb},
  { MT_CALLBACK, 0, "SUBTRACT", menu_storage_cb},
  { MT_CALLBACK, 0, "WATERFALL",menu_storage_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,      0, NULL, NULL } // sentinel
};

static const menuitem_t menu_rbw[] = {
  { MT_CALLBACK, 0, "  AUTO",   menu_rbw_cb},
  { MT_CALLBACK, 0, "  3kHz",   menu_rbw_cb},
  { MT_CALLBACK, 0, " 10kHz",   menu_rbw_cb},
  { MT_CALLBACK, 0, " 30kHz",   menu_rbw_cb},
  { MT_CALLBACK, 0, "100kHz",   menu_rbw_cb},
  { MT_CALLBACK, 0, "300kHz",   menu_rbw_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,      0, NULL, NULL } // sentinel
};


static const menuitem_t menu_dBper[] = {
  { MT_CALLBACK, 0, "  1dB/",   menu_dBper_cb},
  { MT_CALLBACK, 0, "  2dB/",   menu_dBper_cb},
  { MT_CALLBACK, 0, "  5dB/",   menu_dBper_cb},
  { MT_CALLBACK, 0, " 10dB/",   menu_dBper_cb},
  { MT_CALLBACK, 0, " 20dB/",   menu_dBper_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_reffer2[] = {
  { MT_FORM | MT_CALLBACK, 0, "3MHz" ,   menu_reffer_cb2},
  { MT_FORM | MT_CALLBACK, 0, "2MHz" ,   menu_reffer_cb2},
  { MT_FORM | MT_CALLBACK, 0, "1MHz" ,   menu_reffer_cb2},
  { MT_FORM | MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_reffer[] = {
  { MT_FORM | MT_CALLBACK, 0, "OFF"  ,   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 0, "30MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 0, "15MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 0, "10MHz",   menu_reffer_cb},
  { MT_FORM | MT_CALLBACK, 0, "4MHz" ,   menu_reffer_cb},
  { MT_FORM | MT_SUBMENU,  0, S_RARROW" MORE", menu_reffer2},
  { MT_FORM | MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_scale[] = {
  { MT_CALLBACK, KM_REFPOS,         "\2REF\0LEVEL",  menu_scale_cb},
  { MT_SUBMENU,  0,                 "\2SCALE/\0DIV",     menu_dBper},
  { MT_CALLBACK, KM_ATTENUATION,    "ATTEN",         menu_scale_cb},
  { MT_SUBMENU,  0,                 "AVERAGE",       menu_average},
  { MT_CALLBACK, 0,                 "\2SPUR\0REDUCT.",menu_spur_cb},
  { MT_SUBMENU,  0,                 "RBW",           menu_rbw},
  { MT_CANCEL,   0,                 S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_scalehigh[] = {
  { MT_CALLBACK, KM_REFPOS, "\2REF\0LEVEL",  menu_scale_cb},
  { MT_SUBMENU,  0,         "\2SCALE/\0DIV", menu_dBper},
  { MT_SUBMENU,  0,         "AVERAGE",       menu_average},
  { MT_SUBMENU,  0,         "RBW",           menu_rbw},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};



static const menuitem_t menu_stimulus[] = {
  { MT_CALLBACK, 0, "START",            menu_stimulus_cb},
  { MT_CALLBACK, 0, "STOP",             menu_stimulus_cb},
  { MT_CALLBACK, 0, "CENTER",           menu_stimulus_cb},
  { MT_CALLBACK, 0, "SPAN",             menu_stimulus_cb},
  { MT_CALLBACK, 0, "CW FREQ",          menu_stimulus_cb},
//  { MT_SUBMENU,  0, "RBW",              menu_rbw},
  { MT_CALLBACK, 0, "\2PAUSE\0SWEEP",   menu_stimulus_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};


static const menuitem_t menu_marker_type[] = {
  { MT_CALLBACK, 0, "REFERENCE",    menu_marker_type_cb},
  { MT_CALLBACK, 0, "NORMAL",       menu_marker_type_cb},
  { MT_CALLBACK, 0, "DELTA",        menu_marker_type_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

const menuitem_t menu_marker_search[] = {
  //{ MT_CALLBACK, "OFF", menu_marker_search_cb },
  { MT_CALLBACK, 0, "MAXIMUM", menu_marker_search_cb },
  { MT_CALLBACK, 0, "MINIMUM", menu_marker_search_cb },
  { MT_CALLBACK, 0, "\2SEARCH\0" S_LARROW" LEFT", menu_marker_search_cb },
  { MT_CALLBACK, 0, "\2SEARCH\0" S_RARROW" RIGHT", menu_marker_search_cb },
  { MT_CALLBACK, 0, "TRACKING", menu_marker_search_cb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

static const menuitem_t menu_marker_sel[] = {
  { MT_CALLBACK, 0, "MARKER 1",     menu_marker_sel_cb},
  { MT_CALLBACK, 0, "MARKER 2",     menu_marker_sel_cb},
  { MT_CALLBACK, 0, "MARKER 3",     menu_marker_sel_cb},
  { MT_CALLBACK, 0, "MARKER 4",     menu_marker_sel_cb},
  { MT_CALLBACK, 0, "ALL OFF",      menu_marker_sel_cb},
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
  { MT_FORM | MT_CALLBACK, 0, "ENTER DFU", menu_dfu_cb},
  { MT_FORM | MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settings2[] =
{
  { MT_CALLBACK, 0, "AGC",              menu_settings2_cb},
  { MT_CALLBACK, 0, "LNA",              menu_settings2_cb},
  { MT_CALLBACK, 0, "BPF",              menu_settings2_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settings[] =
{
  { MT_CALLBACK, KM_ACTUALPOWER,    "\2ACTUAL\0POWER",  menu_settings_cb},
  { MT_CALLBACK, KM_IF,             "\2IF\0FREQ",       menu_settings_cb},
  { MT_CALLBACK, KM_SAMPLETIME,     "\2SAMPLE\0TIME",   menu_settings_cb},
  { MT_CALLBACK, KM_DRIVE,          "\2LO\0DRIVE",      menu_settings_cb},
  { MT_SUBMENU,  0,                 S_RARROW" MORE",    menu_settings2},
  { MT_CANCEL,   0,                 S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};


static const menuitem_t menu_settingshigh2[] =
{
  { MT_CALLBACK, 0, "AGC",              menu_settings2_cb},
  { MT_CALLBACK, 0, "LNA",              menu_settings2_cb},
  { MT_CANCEL,   0, S_LARROW" BACK", NULL },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

static const menuitem_t menu_settingshigh[] =
{
  { MT_CALLBACK, KM_ACTUALPOWER,    "\2ACTUAL\0POWER",  menu_settings_cb},
  { MT_CALLBACK, KM_SAMPLETIME,     "\2SAMPLE\0TIME",   menu_settings_cb},
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
  { MT_NONE,     0, NULL, NULL } // sentinel
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
  { MT_NONE,     0, NULL, NULL } // sentinel
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
  { MT_NONE,     0, NULL, NULL } // sentinel
};

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
  if (menu == menu_mode) {
    if (item == GetMode()+1) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    } else if (item == 5) {
      plot_printf(uistat.text, sizeof uistat.text, menu_reffer_text[get_refer_output()+1]);
    }
  } else if (menu == menu_lowoutputmode || menu == menu_highoutputmode) {
    if (item == 3) {
      plot_printf(uistat.text, sizeof uistat.text, menu_modulation_text[setting_modulation]);
    }
  } else if (menu == menu_reffer) {
    if (item < 5 && item == get_refer_output() + 1){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_reffer2) {
    if (item == get_refer_output() - 4){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_stimulus) {
    if (item == 5 /* PAUSE */ && !(sweep_mode&SWEEP_ENABLE)) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_scale) {
#if 0
    if (item == 4 /* Spur reduction */ && GetSpur()) {
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
#endif
  } else if (menu == menu_average) {
    if (item == GetAverage()){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_dBper) {
    if (menu_dBper_value[item] == get_trace_scale(1)){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_rbw) {
    if (rbwsel[item] == GetRBW()){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }

  } else if (menu == menu_storage) {
    if (item ==0 && GetStorage()){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
    if (item == 2 && GetSubtractStorage()){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
    if (item == 3 && get_waterfall()){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  } else if (menu == menu_settings2 || menu == menu_settingshigh2) {
    if (item ==0 && setting_agc){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
    if (item == 1 && setting_lna){
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
    if (item == 2 && setting_tracking){         // should not happen in high mode
      *bg = DEFAULT_MENU_TEXT_COLOR;
      *fg = config.menu_normal_color;
    }
  }
  if (ui_mode == UI_MENU && menu_is_form(menu)) {
    //    if (item == 0)
    //      redraw_frame();
    if (item <= 1) {

      area_width = 0;
#if 0
      //    area_height = HEIGHT - 32;
      int y = MENU_BUTTON_HEIGHT*item;
      uint16_t bg = config.menu_normal_color;
      uint16_t fg = DEFAULT_MENU_TEXT_COLOR;
      //    ili9341_fill(320-MENU_BUTTON_WIDTH, y, MENU_BUTTON_WIDTH, MENU_BUTTON_HEIGHT-2, bg);
      ili9341_set_foreground(fg);
      ili9341_set_background(bg);
      char buf[15];
      ili9341_fill(50+25, y, 170, MENU_BUTTON_HEIGHT-2, bg);
      if (menu == menu_lowoutputmode) {
        switch (item) {
        case 0:
          set_sweep_frequency(ST_SPAN, 0);          // For CW sweep mode
          plot_printf(buf, sizeof buf, "%3.3fMHz", frequency0 / 1000000.0);
          break;
        case 1:
          plot_printf(buf, sizeof buf, "%ddB", -10 - setting_attenuate);
          break;
        }
      }
      if (menu == menu_highoutputmode) {
        switch (item) {
        case 0:
          set_sweep_frequency(ST_SPAN, 0);          // For CW sweep mode
          plot_printf(buf, sizeof buf, "%3.3fMHz", frequency0 / 1000000.0);
          break;
        case 1:
          plot_printf(buf, sizeof buf, "%ddB", -10 - setting_drive);
          break;
        }
      }
      ili9341_drawstring_size(buf, 130, y+6, 2);
#endif
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
    uistat.value = setting_attenuate;
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
    uistat.value = setting_attenuate;
    uistat.value = -5 - uistat.value;           // compensation for dB offset during low output mode
    plot_printf(uistat.text, sizeof uistat.text, "%ddB", uistat.value);
    break;
  case KM_HIGHOUTLEVEL:
    uistat.value = setting_drive*5 + 5;
    plot_printf(uistat.text, sizeof uistat.text, "%3ddB", uistat.value);
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
    uistat.value = -5 - uistat.value ;           // compensation for dB offset during low output mode
    SetAttenuation(uistat.value);
    break;
  case KM_HIGHOUTLEVEL:
    uistat.value = uistat.value / 5 - 1 ;           // compensation for dB offset during high output mode
    SetDrive(uistat.value);
    break;
  }
}
