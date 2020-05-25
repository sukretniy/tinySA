
extern volatile int SI4432_Sel;         // currently selected SI4432
void SI4432_Write_Byte(byte ADR, byte DATA );
byte SI4432_Read_Byte( byte ADR );
int VFO = 0;
int points = 101; // For 's' and 'm' commands


VNA_SHELL_FUNCTION(cmd_mode)
{
  if (argc != 2) {
  usage:
    shell_printf("usage: mode low|high input|output\r\n");
    return;
  }
  if (strcmp(argv[0],"low") == 0) {
    if (strcmp(argv[1],"input") == 0)
      set_mode(M_LOW);
    else if(strcmp(argv[1],"output") == 0)
      set_mode(M_GENLOW);
    else
      goto usage;
  } else if (strcmp(argv[0],"high") == 0) {
    if (strcmp(argv[1],"input") == 0)
      set_mode(M_HIGH);
    else if(strcmp(argv[1],"output") == 0)
      set_mode(M_GENHIGH);
    else
      goto usage;
  } else
    goto usage;
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
    shell_printf("usage: if {freq}\r\n");
    return;
  } else {
    int a = my_atoi(argv[0]);
    if (a < 433000000 || a>435000000)
      goto usage;
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
    shell_printf("usage: x {addr(0-95)} [value(0-FF)]\r\n");
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
#ifdef __ULTRA_SA__
  ADF4351_WriteRegister32(VFO, reg);
#endif
  shell_printf("x=%x\r\n", reg);
}


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
//  set_freq(VFO, value);
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
