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


extern volatile int SI4432_Sel;         // currently selected SI4432
void SI4432_Write_Byte(byte ADR, byte DATA );
byte SI4432_Read_Byte( byte ADR );
int VFO = 0;
int points = 101; // For 's' and 'm' commands

VNA_SHELL_FUNCTION(cmd_mode)
{
  static const char cmd_low_high[] = "low|high";
  static const char cmd_in_out[] = "input|output";
  if (argc != 2) {
  usage:
    shell_printf("usage: mode %s %s\r\n", cmd_low_high,cmd_in_out);
    return;
  }
  int lh = get_str_index(argv[0], cmd_low_high);
  int io = get_str_index(argv[1], cmd_in_out);
  if (lh<0 || io<0)
    goto usage;
  switch(lh+io*2)
  {
  case 0:
    set_mode(M_LOW);
    break;
  case 1:
    set_mode(M_HIGH);
    break;
  case 2:
    set_mode(M_GENLOW);
    break;
  case 3:
    set_mode(M_GENHIGH);
    break;
  }
}

VNA_SHELL_FUNCTION(cmd_modulation )
{
  static const char cmd_mod[] = "off|AM_1kHz|AM_10Hz|NFM|WFM|extern";
  if (argc != 1) {
  usage:
    shell_printf("usage: modulation %s\r\n", cmd_mod);
    return;
  }
  static const int cmd_mod_val[] = { MO_NONE, MO_AM_1kHz, MO_AM_10Hz, MO_NFM, MO_WFM, MO_EXTERNAL};
  int m = get_str_index(argv[1], cmd_mod);
  if (m<0)
     goto usage;
  set_modulation(cmd_mod_val[m]);
}

VNA_SHELL_FUNCTION(cmd_spur)
{
  if (argc != 1) {
  usage:
    shell_printf("usage: spur on|off\r\n");
    return;
  }
  if (strcmp(argv[0],"on") == 0) {
    setting.spur = 1;
  } else if (strcmp(argv[0],"off") == 0) {
    setting.spur = 0;
  } else
    goto usage;
}

VNA_SHELL_FUNCTION(cmd_load)
{
  if (argc != 1) {
  usage:
    shell_printf("usage: load 0..4\r\n");
    return;
  }
  int a = my_atoi(argv[0]);
  if (0 <= a && a <= 4) {
    caldata_recall(a);
  } else
    goto usage;
}


VNA_SHELL_FUNCTION(cmd_attenuate)
{
  if (argc != 1) {
  usage:
    shell_printf("usage: attenuate 0..31|auto\r\n");
    return;
  }
  if (strcmp(argv[0],"auto") == 0) {
    set_auto_attenuation();
  } else {
    int a = my_atoi(argv[0]);
    if (a < 0 || a>31)
      goto usage;
    set_attenuation(a);
  }
}

VNA_SHELL_FUNCTION(cmd_level)
{
  if (argc != 1) {
    shell_printf("usage: level -76..-6\r\n");
    return;
  }
  float f = my_atof(argv[0]);
  set_level(f);
}



VNA_SHELL_FUNCTION(cmd_levelsweep)
{
  if (argc != 1) {
    shell_printf("usage: levelsweep -76..+76\r\n");
    return;
  }
  float f = my_atof(argv[0]);
  set_level_sweep(f);
}

VNA_SHELL_FUNCTION(cmd_leveloffset)
{
  if (argc == 0) {
    shell_printf("leveloffset low %.1f\r\n", (float) config.low_level_offset);
    shell_printf("leveloffset high %.1f\r\n", (float)config.high_level_offset);
    return;
  } else if (argc == 2) {
    float v = my_atof(argv[1]);
    if (strcmp(argv[0],"low") == 0)
      config.low_level_offset = v;
    else if (strcmp(argv[0],"high") == 0)
      config.low_level_offset = v;
    else
      goto usage;
  } else {
  usage:
    shell_printf("leveloffset [low|high] [<offset>]\r\n");
  }
}


VNA_SHELL_FUNCTION(cmd_rbw)
{
  if (argc != 1) {
  usage:
    shell_printf("usage: rbw 2..600|auto\r\n");
    return;
  }
  if (strcmp(argv[0],"auto") == 0 || strcmp(argv[0],"0") == 0) {
    set_RBW(0);
  } else {
    int a = my_atoi(argv[0]);
    if (a < 2 || a>600)
      goto usage;
    set_RBW(a);
  }
}

VNA_SHELL_FUNCTION(cmd_if)
{
  if (argc != 1) {
  usage:
    shell_printf("usage: if {433M..435M}\r\n");
    return;
  } else {
    int a = my_atoi(argv[0]);
    if (a!= 0 &&( a < 433000000 || a>435000000))
      goto usage;
    setting.auto_IF = false;
    set_IF(a);
  }
}


VNA_SHELL_FUNCTION(cmd_v)


{
    if (argc != 1) {
        shell_printf("%d\r\n", SI4432_Sel);
        return;
    }
    VFO = my_atoi(argv[0]);
    shell_printf("VFO %d\r\n", VFO);
}

int xtoi(char *t)
{

  int v=0;
  while (*t) {
    if ('0' <= *t && *t <= '9')
      v = v*16 + *t - '0';
    else if ('a' <= *t && *t <= 'f')
      v = v*16 + *t - 'a' + 10;
    else if ('A' <= *t && *t <= 'F')
      v = v*16 + *t - 'A' + 10;
    else
      return v;
    t++;
  }
  return v;
}

VNA_SHELL_FUNCTION(cmd_y)
{
  int rvalue;
  int lvalue = 0;
  if (argc != 1 && argc != 2) {
    shell_printf("usage: y {addr(0-95)} [value(0-FF)]\r\n");
    return;
  }
  rvalue = xtoi(argv[0]);
  SI4432_Sel = VFO;
  if (argc == 2){
    lvalue = xtoi(argv[1]);
    SI4432_Write_Byte(rvalue, lvalue);
  } else {
    lvalue = SI4432_Read_Byte(rvalue);
    shell_printf("%x\r\n", lvalue);
  }
}

VNA_SHELL_FUNCTION(cmd_selftest)
{
  if (argc < 1 || argc > 2) {
    shell_printf("usage: selftest (1-3) [arg]\r\n");
    return;
  }
  setting.test = my_atoi(argv[0]);
  if (argc == 1)
    setting.test_argument = 0;
  else
    setting.test_argument = my_atoi(argv[1]);
  sweep_mode = SWEEP_SELFTEST;
}

#ifdef __ULTRA_SA__
VNA_SHELL_FUNCTION(cmd_x)
{
  uint32_t reg;


  if (argc != 1) {
    shell_printf("usage: x value(0-FFFFFFFF)\r\n");
    return;
  }
  reg = xtoi(argv[0]);

  if ((reg & 7) == 5) {
   if (reg & (1<<22))
      VFO = 1;
    else
      VFO = 0;
   reg &= ~0xc00000;    // Force led to show lock
   reg |=  0x400000;
  }

  ADF4351_WriteRegister32(VFO, reg);
  shell_printf("x=%x\r\n", reg);
}
#endif


VNA_SHELL_FUNCTION(cmd_i)
{
  int rvalue;
return;             // Don't use!!!!
  SI4432_Init();
  shell_printf("SI4432 init done\r\n");
  if (argc == 1) {
    rvalue = xtoi(argv[0]);
    set_switches(rvalue);
    set_mode(rvalue);
    shell_printf("SI4432 mode %d set\r\n", rvalue);
  }
}

VNA_SHELL_FUNCTION(cmd_o)
{
  (void) argc;
  uint32_t value = my_atoi(argv[0]);
  if (VFO == 0)
    setting.frequency_IF = value;
  set_freq(VFO, value);
}

VNA_SHELL_FUNCTION(cmd_d)
{
  (void) argc;
  (void) argv;
//  int32_t a = my_atoi(argv[0]);
//  setting.drive = a;
}


VNA_SHELL_FUNCTION(cmd_a)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("a=%d\r\n", frequencyStart);
    return;
  }
  int32_t value = my_atoi(argv[0]);
  frequencyStart = value;
}


VNA_SHELL_FUNCTION(cmd_b)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("b=%d\r\n", frequencyStop);
    return;
  }
  int32_t value = my_atoi(argv[0]);
  frequencyStop = value;
}

VNA_SHELL_FUNCTION(cmd_t)
{
  (void)argc;
  (void)argv;
}

VNA_SHELL_FUNCTION(cmd_e)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("e=%d\r\n", setting.tracking);
    return;
  }
  setting.tracking = my_atoi(argv[0]);
  if (setting.tracking == -1)
    setting.tracking = false;
  else
    setting.tracking = true;

  if (argc >1)
    frequencyExtra = my_atoi(argv[1]);
}

VNA_SHELL_FUNCTION(cmd_s)
{
  (void)argc;
  if (argc != 1) {
    shell_printf("s=%d\r\n", points);
    return;
  }
  points = my_atoi(argv[0]);
}

void sweep_remote(void)
{
  int old_step = setting.frequency_step;
  uint32_t f_step = (frequencyStop-frequencyStart)/ points;
  setting.frequency_step = f_step;
  streamPut(shell_stream, '{');
  dirty = true;
  for (int i = 0; i<points; i++) {
    if (operation_requested)
      break;
    float val = perform(false, i, frequencyStart - setting.frequency_IF + f_step * i, false);
    streamPut(shell_stream, 'x');
    int v = val*2 + 256;
    streamPut(shell_stream, (uint8_t)(v & 0xFF));
    streamPut(shell_stream, (uint8_t)((v>>8) & 0xFF));
  // enable led
  }
  streamPut(shell_stream, '}');
  setting.frequency_step = old_step;
  sweep_mode = 0;
}

VNA_SHELL_FUNCTION(cmd_m)
{
  (void)argc;
  (void)argv;

//  set_mode(0);
//  setting.tracking = false; //Default test setup
//  setting.step_atten = false;
//  set_attenuation(0);
//  set_reflevel(-10);
//  set_sweep_frequency(ST_START,frequencyStart - setting.frequency_IF );
//  set_sweep_frequency(ST_STOP, frequencyStop - setting.frequency_IF);
//  draw_cal_status();

  pause_sweep();
//  update_rbw();
  chThdSleepMilliseconds(10);
  sweep_mode = SWEEP_REMOTE;
//  update_rbw();
}

VNA_SHELL_FUNCTION(cmd_p)
{
  (void)argc;
return;
  int p = my_atoi(argv[0]);
  int a = my_atoi(argv[1]);
  if (p==5)
    set_attenuation(-a);
  if (p==6)
    set_mode(a);
  if (p==1)
    if (get_refer_output() != a)
      set_refer_output(a);
}

VNA_SHELL_FUNCTION(cmd_w)
{
  (void)argc;
  int p = my_atoi(argv[0]);
return;
  set_RBW(p);
}

VNA_SHELL_FUNCTION(cmd_correction)
{
  (void)argc;
  if (argc == 0) {
    shell_printf("index frequency value\r\n");
    for (int i=0; i<CORRECTION_POINTS; i++) {
      shell_printf("%d %d %.1f\r\n", i, config.correction_frequency[i], config.correction_value[i]);
    }
    return;
  }
  if (argc == 1 && (strcmp(argv[0],"reset") == 0)) {
    for (int i=0; i<CORRECTION_POINTS; i++) {
      config.correction_value[i] = 0.0;
    }
    shell_printf("correction table reset\r\n");
    return;
  }
  if (argc != 3) {
    shell_printf("usage: correction 0-9 frequency(Hz) value(dB)\r\n");
    return;
  }
  int i = my_atoi(argv[0]);
  uint32_t f = my_atoui(argv[1]);
  float v = my_atof(argv[2]);
  config.correction_frequency[i] = f;
  config.correction_value[i] = v;
  shell_printf("updated %d to %d %.1f\r\n", i, config.correction_frequency[i], config.correction_value[i]);
}
