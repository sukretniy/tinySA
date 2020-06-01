/* Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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
#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include <math.h>
#include "si4432.h"

#define CS_SI0_HIGH     palSetPad(GPIOC, GPIO_RX_SEL)
#define CS_SI1_HIGH     palSetPad(GPIOC, GPIO_LO_SEL)
#define CS_PE_HIGH      palSetPad(GPIOC, GPIO_PE_SEL)

#define RF_POWER_HIGH   palSetPad(GPIOC, GPIO_RF_PWR)


#define CS_SI0_LOW     palClearPad(GPIOC, GPIO_RX_SEL)
#define CS_SI1_LOW     palClearPad(GPIOC, GPIO_LO_SEL)
#define CS_PE_LOW      palClearPad(GPIOC, GPIO_PE_SEL)

#define SPI2_CLK_HIGH   palSetPad(GPIOB, GPIO_SPI2_CLK)
#define SPI2_CLK_LOW    palClearPad(GPIOB, GPIO_SPI2_CLK)

#define SPI2_SDI_HIGH   palSetPad(GPIOB, GPIO_SPI2_SDI)
#define SPI2_SDI_LOW    palClearPad(GPIOB, GPIO_SPI2_SDI)

#define SPI2_SDO    ((palReadPort(GPIOB) & (1<<GPIO_SPI2_SDO))?1:0)


//#define MAXLOG 1024
//unsigned char SI4432_logging[MAXLOG];
//volatile int log_index = 0;

//#define SI4432_log(X)   { if (log_index < MAXLOG)  SI4432_logging[log_index++] = X; }
#define SI4432_log(X)

void shiftOut(uint8_t val)
{
     uint8_t i;
     SI4432_log(SI4432_Sel);
     SI4432_log(val);
     for (i = 0; i < 8; i++)  {
           if (val & (1 << (7 - i)))
             SPI2_SDI_HIGH;
           else
             SPI2_SDI_LOW;
           SPI2_CLK_HIGH;
           SPI2_CLK_LOW;
     }
}

uint8_t shiftIn(void) {
    uint8_t value = 0;
    uint8_t i;
    for (i = 0; i < 8; ++i) {
      SPI2_CLK_HIGH;
        value |= SPI2_SDO << (7 - i);
        SPI2_CLK_LOW;
    }
    return value;
}

const int SI_nSEL[3] = { GPIO_RX_SEL, GPIO_LO_SEL, 0}; // #3 is dummy!!!!!!

volatile int SI4432_Sel = 0;         // currently selected SI4432
// volatile int SI4432_guard = 0;

#ifdef __SI4432_H__
#define SELECT_DELAY 10
void SI4432_Write_Byte(byte ADR, byte DATA )
{
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
  SPI2_CLK_LOW;
  palClearPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  chThdSleepMicroseconds(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA );
  palSetPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
}

void SI4432_Write_3_Byte(byte ADR, byte DATA1, byte DATA2, byte DATA3 )
{
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
  SPI2_CLK_LOW;
  palClearPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  chThdSleepMicroseconds(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA1 );
  shiftOut( DATA2 );
  shiftOut( DATA3 );
  palSetPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
}

byte SI4432_Read_Byte( byte ADR )
{
  byte DATA ;
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
  SPI2_CLK_LOW;
  palClearPad(GPIOC, SI_nSEL[SI4432_Sel]);
  shiftOut( ADR );
  DATA = shiftIn();
  palSetPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
  return DATA ;
}



void SI4432_Reset(void)
{
  int count = 0;
  SI4432_Read_Byte ( 0x03 );    // Clear pending interrupts
  SI4432_Read_Byte ( 0x04 );
  // always perform a system reset (don't send 0x87)
  SI4432_Write_Byte( 0x07, 0x80);
  chThdSleepMilliseconds(50);
  // wait for chiprdy bit
  while (count++ < 100 && ( SI4432_Read_Byte ( 0x04 ) & 0x02 ) == 0) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Drive(int d)
{
  SI4432_Write_Byte(0x6D, (byte) (0x18+(d & 7)));
}

void SI4432_Transmit(int d)
{
  int count = 0;
  SI4432_Write_Byte(0x6D, (byte) (0x18+(d & 7)));
  if (( SI4432_Read_Byte ( 0x02 ) & 0x03 ) == 2)
    return; // Already in transmit mode
  chThdSleepMilliseconds(20);
  SI4432_Write_Byte( 0x07, 0x03);
  chThdSleepMilliseconds(20);
  SI4432_Write_Byte( 0x07, 0x0b);
  chThdSleepMilliseconds(30);
  while (count++ < 100 && ( SI4432_Read_Byte ( 0x02 ) & 0x03 ) != 2) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Receive(void)
{
  int count = 0;
  if (( SI4432_Read_Byte ( 0x02 ) & 0x03 ) == 1)
    return; // Already in receive mode
  chThdSleepMilliseconds(20);
  SI4432_Write_Byte( 0x07, 0x03);
  chThdSleepMilliseconds(20);
  SI4432_Write_Byte( 0x07, 0x07);
  chThdSleepMilliseconds(30);
  while (count++ < 100 && ( SI4432_Read_Byte ( 0x02 ) & 0x03 ) != 1) {
    chThdSleepMilliseconds(5);
  }
}


// User asks for an RBW of WISH, go through table finding the last triple
// for which WISH is greater than the first entry, use those values,
// Return the first entry of the following triple for the RBW actually achieved
static const short RBW_choices[] =
{     // Each quadrupple is:  ndec, fils, WISH*10, corr*10
#if 1
      5,1,26,-5,
      5,2,28,-5,
      5,3,31,0,
      5,4,32,0,
      5,5,37,0,
      5,6,42,0,
      5,7,45,5,
      4,1,49,5,
      4,2,54,-5,
      4,3,59,-5,
      4,4,61,-5,
      4,5,72,0,
      4,6,82,0,
      4,7,88,-5,
      3,1,95,0,
      3,2,106,0,
      3,3,115,0,
      3,4,121,-5,
      3,5,142,0,
      3,6,162,0,
      3,7,175,0,
      2,1,189,0,
      2,2,210,-5,
      2,3,227,0,
      2,4,240,-5,
      2,5,282,0,
      2,6,322,0,
      2,7,347,0,
      1,1,377,0,
      1,2,417,-5,
      1,3,452,0,
      1,4,479,-5,
      1,5,562,0,
      1,6,641,0,
      1,7,692,0,
      0,1,752,-5,
      0,2,832,-5,
      0,3,900,0,
      0,4,953,-5,
      0,5,1121,0,
      0,6,1279,0,
      0,7,1379,0,
      1,4,1428,15,
      1,5,1678,20,
      1,9,1811,-55,
      0,15,1915,-105,
      0,1,2251,15,
      0,2,2488,20,
      0,3,2693,20,
      0,4,2849,15,
      0,8,3355,-15,
      0,9,3618,-55,
      0,10,4202,-15,
      0,11,4684,-15,
      0,12,5188,-20,
      0,13,5770,-15,
      0,14,6207,-10,
#else
5,1,26,0,
5,2,28,0,
5,3,31,0,
5,4,32,0,
5,5,37,0,
5,6,42,0,
5,7,45,-5,
4,1,49,-5,
4,2,54,-10,
4,3,59,-10,
4,4,61,-10,
4,5,72,-10,
4,6,82,-5,
4,7,88,0,
3,1,95,0,
3,2,106,0,
3,3,115,-5,
3,4,121,-5,
3,5,142,-10,
3,6,162,0,
3,7,175,0,
2,1,189,0,
2,2,210,0,
2,3,227,0,
2,4,240,5,
2,5,282,-5,
2,6,322,0,
2,7,347,0,
1,1,377,0,
1,2,417,0,
1,3,452,0,
1,4,479,5,
1,5,562,0,
1,6,641,0,
1,7,692,0,
0,1,752,5,
0,2,832,5,
0,3,900,0,
0,4,953,5,
0,5,1121,0,
0,6,1279,0,
0,7,1379,0,
1,4,1428,-15,
1,5,1678,-25,
1,9,1811,50,
0,15,1915,105,
0,1,2251,-20,
0,2,2488,0,
0,3,2693,-15,
0,4,2849,-10,
0,8,3355,20,
0,9,3618,55,
0,10,4202,20,
0,11,4684,20,
0,12,5188,25,
0,13,5770,15,
0,14,6207,15,
#endif
#if 0
     5,1,26,0,
     5,2,28,0,
     5,3,31,0,
     5,4,32,0,
     5,5,37,0,
     5,6,42,10,
     5,7,45,10,
     4,1,49,10,
     4,2,54,10,
     4,3,59,10,
     4,4,61,10,
     4,5,72,10,
     4,6,82,10,
     4,7,88,10,
     3,1,95,10,
     3,2,106,5,
     3,3,115,0,
     3,4,121,0,
     3,5,142,0,
     3,6,162,10,
     3,7,175,10,
     2,1,189,10,
     2,2,210,10,
     2,3,227,10,
     2,4,240,10,
     2,5,282,5,
     2,6,322,10,
     2,7,347,10,
     1,1,377,10,
     1,2,417,10,
     1,3,452,10,
     1,4,479,5,
     1,5,562,10,
     1,6,641,10,
     1,7,692,10,
     0,1,752,10,
     0,2,832,10,
     0,3,900,5,
     0,4,953,10,
     0,5,1121,10,
     0,6,1279,10,
     0,7,1379,5,
     1,4,1428,0,
     1,5,1678,-10,
     1,9,1811,65,
     0,15,1915,120,
     0,1,2251,-5,
     0,2,2488,0,
     0,3,2693,-5,
     0,4,2849,0,
     0,8,3355,30,
     0,9,3618,60,
     0,10,4202,25,
     0,11,4684,25,
     0,12,5188,35,
     0,13,5770,35,
     0,14,6207,35
#endif
};

static float SI4432_RSSI_correction = 0;

float SI4432_SET_RBW(float w)  {
  uint8_t dwn3=0;
  int32_t WISH = (uint32_t)(w * 10.0);
  uint8_t ndec, fils, i;
  if (WISH > 6207)   WISH=6207;     // Final value in RBW_choices[]
  if (WISH > 1379) dwn3 = 1 ;
  for (i=3; i<sizeof(RBW_choices)/sizeof(RBW_choices[0]); i+=4)
    if (WISH <= RBW_choices[i-1])  break;
  ndec = RBW_choices[i-3];
  fils = RBW_choices[i-2];
  WISH = RBW_choices[i-1];        // RBW achieved by Si4432 in Hz
  SI4432_RSSI_correction = RBW_choices[i]/10.0;
  uint8_t BW = (dwn3 << 7) | (ndec << 4) | fils ;
  SI4432_Write_Byte(0x1C , BW ) ;
  return (((float)WISH) / 10.0) ;
}

float SI4432_force_RBW(int i)
{
  return(SI4432_SET_RBW((float)(RBW_choices[i*4+2]/10.0)));
}

float SI4432_RBW_table(int i){
  if (i < 0)
    return 0;
  if (i * 4 >= (int)(sizeof RBW_choices) / 2 )
    return 0;
  return(RBW_choices[i*4-1]);
}

int setting_frequency_10mhz = 10000000;

void set_10mhz(int f)
{
  setting_frequency_10mhz = f;
}

void SI4432_Set_Frequency ( long Freq ) {
  int hbsel;
  long Carrier;
  if (Freq >= 480000000) {
    hbsel = 1;
    Freq = Freq / 2;
  } else {
    hbsel = 0;
  }
  int sbsel = 1;
  long N = Freq / setting_frequency_10mhz;
  Carrier = ( 4 * ( Freq - N * setting_frequency_10mhz )) / 625;
  int Freq_Band = ( N - 24 ) | ( hbsel << 5 ) | ( sbsel << 6 );
#if 0
  SI4432_Write_Byte ( 0x75, Freq_Band );
  SI4432_Write_Byte ( 0x76, (Carrier>>8) & 0xFF );
  SI4432_Write_Byte ( 0x77, Carrier & 0xFF  );
#else
  SI4432_Write_3_Byte ( 0x75, Freq_Band, (Carrier>>8) & 0xFF, Carrier & 0xFF  );
#endif
}

int actualStepDelay = 1500;
//extern int setting.repeat;

float SI4432_RSSI(uint32_t i, int s)
{
  (void) i;
  int32_t RSSI_RAW;
  (void) i;
  // SEE DATASHEET PAGE 61
#ifdef USE_SI4463
  if (SI4432_Sel == 2) {
    RSSI_RAW = Si446x_getRSSI();
  } else
#endif
//START_PROFILE
    SI4432_Sel = s;
    my_microsecond_delay(actualStepDelay);
    // chThdSleepMicroseconds(actualStepDelay);
    i = setting.repeat;
    RSSI_RAW  = 0;
    while (i-->0)
      RSSI_RAW += ((unsigned int)SI4432_Read_Byte( 0x26 )) << 4 ;
    RSSI_RAW = RSSI_RAW / setting.repeat;
 //   if (MODE_INPUT(setting.mode) && RSSI_RAW == 0)
 //     SI4432_Init();
  float dBm = (((float)RSSI_RAW)/16.0 - 240.0)/2.0 + SI4432_RSSI_correction;
#ifdef __SIMULATION__
  dBm = Simulated_SI4432_RSSI(i,s);
#endif
//STOP_PROFILE
  // Serial.println(dBm,2);
  return dBm ;
}


void SI4432_Sub_Init(void)
{
  SI4432_Reset();


  SI4432_Write_Byte(0x69, 0x60); //AGC override according to WBS3


#if 0           // Not sure if these add any value
  //set VCO and PLL Only for SI4432 V2
  SI4432_Write_Byte(0x72, 0x1F); //write 0x1F to the Frequency Deviation register
  // VCO tuning registers
  SI4432_Write_Byte(0x5A, 0x7F); //write 0x7F to the VCO Current Trimming register
  SI4432_Write_Byte(0x58, 0x80); //write 0xD7 to the ChargepumpCurrentTrimmingOverride register
  SI4432_Write_Byte(0x59, 0x40); //write 0x40 to the Divider Current Trimming register
#endif
#if 0
  //set the AGC,  BAD FOR PERFORMANCE!!!!!!
  SI4432_Write_Byte(0x6A, 0x0B); //write 0x0B to the AGC Override 2 register
  //set ADC reference voltage to 0.9V,  BAD FOR PERFORMANCE!!!!!!
  SI4432_Write_Byte(0x68, 0x04); //write 0x04 to the Deltasigma ADC Tuning 2 register

  SI4432_Write_Byte(0x1F, 0x03); //write 0x03 to the Clock Recovery Gearshift Override register

#endif


  SI4432_Write_Byte(0x05, 0x0);
  SI4432_Write_Byte(0x06, 0x0);
  // Enable receiver chain
//  SI4432_Write_Byte(0x07, 0x05);
  // Clock Recovery Gearshift Value
  SI4432_Write_Byte(0x1F, 0x00);
  // IF Filter Bandwidth
  SI4432_SET_RBW(10) ;
//  // Register 0x75 Frequency Band Select
//  byte sbsel = 1 ;  // recommended setting
//  byte hbsel = 0 ;  // low bands
//  byte fb = 19 ;    // 430�439.9 MHz
//  byte FBS = (sbsel << 6 ) | (hbsel << 5 ) | fb ;
//  SI4432_Write_Byte(0x75, FBS) ;
  SI4432_Write_Byte(0x75, 0x46) ;
  // Register 0x76 Nominal Carrier Frequency
  // WE USE 433.92 MHz
  // Si443x-Register-Settings_RevB1.xls
//  SI4432_Write_Byte(0x76, 0x62) ;
  SI4432_Write_Byte(0x76, 0x00) ;
  // Register 0x77 Nominal Carrier Frequency
  SI4432_Write_Byte(0x77, 0x00) ;
  // RX MODEM SETTINGS
  SI4432_Write_Byte(0x1C, 0x81) ;
  SI4432_Write_Byte(0x1D, 0x3C) ;
  SI4432_Write_Byte(0x1E, 0x02) ;
  SI4432_Write_Byte(0x1F, 0x03) ;
  // SI4432_Write_Byte(0x20, 0x78) ;
  SI4432_Write_Byte(0x21, 0x01) ;
  SI4432_Write_Byte(0x22, 0x11) ;
  SI4432_Write_Byte(0x23, 0x11) ;
  SI4432_Write_Byte(0x24, 0x01) ;
  SI4432_Write_Byte(0x25, 0x13) ;
  SI4432_Write_Byte(0x2A, 0xFF) ;
  SI4432_Write_Byte(0x2C, 0x28) ;
  SI4432_Write_Byte(0x2D, 0x0C) ;
  SI4432_Write_Byte(0x2E, 0x28) ;


  SI4432_Write_Byte(0x69, 0x60); // AGC, no LNA, fast gain increment


// GPIO automatic antenna switching
  SI4432_Write_Byte(0x0B, 0x12) ; // Normal
  SI4432_Write_Byte(0x0C, 0x15) ;

}

#define V0_XTAL_CAPACITANCE 0x64
#define V1_XTAL_CAPACITANCE 0x64



void SI4432_Init()
{

  RF_POWER_HIGH;                // Power the RF part
  chThdSleepMilliseconds(25);

  //DebugLine("IO set");
  SI4432_Sel = 0;
  SI4432_Sub_Init();

  SI4432_Sel = 1;
  SI4432_Sub_Init();
//DebugLine("1 init done");

  SI4432_Sel = 0;
//  SI4432_Receive();// Enable receiver chain
//  SI4432_Write_Byte(0x09, V0_XTAL_CAPACITANCE);// Tune the crystal
  SI4432_Set_Frequency(433700000);
  SI4432_Write_Byte(0x0D, 0x1F) ; // Set GPIO2 output to ground


  SI4432_Sel = 1;
//  SI4432_Write_Byte(0x09, V1_XTAL_CAPACITANCE);// Tune the crystal
  SI4432_Set_Frequency(443700000);
  SI4432_Write_Byte(0x0D, 0x1F) ; // Set GPIO2 output to ground

  //  SI4432_Write_Byte(0x6D, 0x1C);//Set low power
//  SI4432_Transmit(0);

//  SI4432_Write_Byte(0x0D, 0xC0) ; // Set GPIO2 maximumdrive and clock output
//  SI4432_Write_Byte(0x0A, 0x02) ; // Set 10MHz output
}

void SI4432_SetReference(int freq)
{
  SI4432_Sel = 1;         //Select Lo module
  if (freq < 0 || freq > 7 ) {
    SI4432_Write_Byte(0x0D, 0x1F) ; // Set GPIO2 to GND
  } else {
    SI4432_Write_Byte(0x0D, 0xC0) ; // Set GPIO2 maximumdrive and clock output
    SI4432_Write_Byte(0x0A, freq & 0x07) ; // Set GPIO2 frequency
  }
}

//------------PE4302 -----------------------------------------------

// Comment out this define to use parallel mode PE4302

#define PE4302_en 10

void PE4302_init(void) {
  CS_PE_LOW;
}

#define PE4302_DELAY 100

void PE4302_shiftOut(uint8_t val)
{
     uint8_t i;
     SI4432_log(SI4432_Sel);
     SI4432_log(val);
     for (i = 0; i < 8; i++)  {
           if (val & (1 << (7 - i)))
             SPI2_SDI_HIGH;
           else
             SPI2_SDI_LOW;
//           chThdSleepMicroseconds(PE4302_DELAY);
           SPI2_CLK_HIGH;
//           chThdSleepMicroseconds(PE4302_DELAY);
           SPI2_CLK_LOW;
//           chThdSleepMicroseconds(PE4302_DELAY);
     }
}

void PE4302_Write_Byte(unsigned char DATA )
{
//  chThdSleepMicroseconds(PE4302_DELAY);
  SPI2_CLK_LOW;
//  chThdSleepMicroseconds(PE4302_DELAY);
  PE4302_shiftOut(DATA);
//  chThdSleepMicroseconds(PE4302_DELAY);
  CS_PE_HIGH;
//  chThdSleepMicroseconds(PE4302_DELAY);
  CS_PE_LOW;
//  chThdSleepMicroseconds(PE4302_DELAY);

}

#endif



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
extern float actual_rbw;
float myfrand(void)
{
  seed = (unsigned int) (1103515245 * seed + 12345) ;
  return ((float) seed) / 1000000000.0;
}
#define NOISE  ((myfrand()-2) * 2)  // +/- 4 dBm noise
extern int settingAttenuate;

//#define LEVEL(i, f, v) (v * (1-(fabs(f - frequencies[i])/actual_rbw/1000)))

float LEVEL(uint32_t i, uint32_t f, int v)
{
  float dv;
  float df = fabs((float)f - (float)i);
  if (df < actual_rbw*1000)
    dv = df/(actual_rbw*1000);
  else
    dv =  1 + 50*(df - actual_rbw*1000)/(actual_rbw*1000);
  return (v - dv - settingAttenuate);
}

float Simulated_SI4432_RSSI(uint32_t i, int s)
{
  SI4432_Sel = s;
  float v = -100 + log10(actual_rbw)*10 + NOISE;
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
//------------------------------- ADF4351 -------------------------------------

#ifdef __ULTRA_SA__

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CS_ADF0_HIGH     palSetPad(GPIOA, 9)
#define CS_ADF1_HIGH     palSetPad(GPIOA, 10)

#define CS_ADF0_LOW     palClearPad(GPIOA, 9)
#define CS_ADF1_LOW     palClearPad(GPIOA, 10)

#define SPI3_CLK_HIGH   palSetPad(GPIOA, 1)
#define SPI3_CLK_LOW    palClearPad(GPIOA, 1)

#define SPI3_SDI_HIGH   palSetPad(GPIOA, 2)
#define SPI3_SDI_LOW    palClearPad(GPIOA, 2)


void ADF_shiftOut(uint8_t val)
{
     uint8_t i;
     for (i = 0; i < 8; i++)  {
           if (val & (1 << (7 - i)))
             SPI3_SDI_HIGH;
           else
             SPI3_SDI_LOW;
//           chThdSleepMicroseconds(10);
           SPI3_CLK_HIGH;
//           chThdSleepMicroseconds(10);
           SPI3_CLK_LOW;
//           chThdSleepMicroseconds(10);
     }
}

//unsigned long registers[6] =  {0x4580A8, 0x80080C9, 0x4E42, 0x4B3, 0xBC803C, 0x580005} ;
//unsigned long registers[6] =  {0x4C82C8, 0x80083E9, 0x6E42, 0x8004B3, 0x8C81FC, 0x580005} ;

//uint32_t registers[6] =  {0x320000, 0x8008011, 0x4E42, 0x4B3,0x8C803C , 0x580005} ;         //25 MHz ref

uint32_t registers[6] =  {0xA00000, 0x8000011, 0x4E42, 0x4B3,0xDC003C , 0x580005} ;         //10 MHz ref

int debug = 0;
int ADF4351_LE[2] = { 9, 10};
int ADF4351_Mux = 7;


//#define DEBUG(X) // Serial.print( X )
//#define DEBUGLN(X) Serial.println( X )
//#define DEBUGFLN(X,Y) Serial.println( X,Y )
//#define DEBUGF(X,Y) Serial.print( X,Y )
#define DEBUG(X)
#define DEBUGLN(X)


double RFout, //Output freq in MHz
#if 0   //Black modules
  PFDRFout[6] = {25.0,25.0,25.0,10.0,10.0,10.0}, //Reference freq in MHz
  Chrystal[6] = {25.0,25.0,25.0,10.0,10.0,10.0},
#else // Green modules
  PFDRFout[6] = {10.0,10.0,10.0,10.0,10.0,10.0}, //Reference freq in MHz
  Chrystal[6] = {10.0,10.0,10.0,10.0,10.0,10.0},
#endif

  OutputChannelSpacing = 0.010, // = 0.01
  FRACF; // Temp

unsigned int long RFint,  // Output freq/10Hz
  INTA,         // Temp
  RFcalc, //UI
  MOD, //Temp
  FRAC; //Temp

byte OutputDivider; // Temp
byte lock=2; //Not used

// Lock = A4

void ADF4351_Setup()
{
//  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL );
//  palSetPadMode(GPIOA, 2, PAL_MODE_OUTPUT_PUSHPULL );

  SPI3_CLK_HIGH;
  SPI3_SDI_HIGH;
  CS_ADF0_HIGH;
  CS_ADF1_HIGH;
//  bitSet (registers[2], 17); // R set to 8
//  bitClear (registers[2], 14); // R set to 8

//  while(1) {
//
  ADF4351_set_frequency(0,100000000,0);
  ADF4351_set_frequency(1,150000000,0);
//  ADF4351_Set(0);
//  ADF4351_Set(1);
//  chThdSleepMilliseconds(1000);
//  }
//  bitSet (registers[2], 17); // R set to 8
//  bitClear (registers[2], 14); // R set to 8
//  for (int i=0; i<6; i++) pinMode(ADF4351_LE[i], OUTPUT);          // Setup pins
//  for (int i=0; i<6; i++) digitalWrite(ADF4351_LE[i], HIGH);
//  pinMode(ADF4351_Mux, INPUT);
//  SPI.begin();                          // Init SPI bus
//  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  //SPI.setDataMode(SPI_MODE0);           // CPHA = 0  Clock positive
  //SPI.setBitOrder(MSBFIRST);
}

void ADF4351_WriteRegister32(int channel, const uint32_t value)
{
  palClearPad(GPIOA, ADF4351_LE[channel]);
//  chThdSleepMicroseconds(10);
  for (int i = 3; i >= 0; i--) ADF_shiftOut((value >> (8 * i)) & 0xFF);
//  chThdSleepMicroseconds(10);
  palSetPad(GPIOA, ADF4351_LE[channel]);
//  chThdSleepMicroseconds(10);
  palClearPad(GPIOA, ADF4351_LE[channel]);
//  chThdSleepMicroseconds(10);
}

void ADF4351_disable_output()
{
    bitClear (registers[4], 5); // digital lock
    ADF4351_Set(0);
}

void ADF4351_enable_output()
{
    bitSet (registers[4], 5); // digital lock
    ADF4351_Set(0);
}
void ADF4351_Set(int channel)
{ for (int i = 5; i >= 0; i--) {
    ADF4351_WriteRegister32(channel, registers[i]);
//    if (debug)  Serial.println(registers[i],HEX);
}
}

void ADF4351_set_frequency(int channel, unsigned long freq, int drive)  // freq / 10Hz
{
  ADF4351_prep_frequency(channel,freq, drive);
  ADF4351_Set(channel);
}

void ADF4351_spur_mode(int S)
{
    if (S & 1) {
      bitSet (registers[2], 29); // R set to 8
    } else {
      bitClear (registers[2], 29); // R set to 8
    }
    if (S & 2)
      bitSet (registers[2], 30); // R set to 8
    else
      bitClear (registers[2], 30); // R set to 8
}

void ADF4351_R_counter(int R)
{
      int dbl = false;
      if (R < 0) {
        dbl = true;
        R = -R;
      }
      if (R<1)
        return;
      if (dbl) {
        bitSet (registers[2], 25); // Reference doubler
      } else {
        bitClear (registers[2], 25); // Reference doubler
      }
      for (int channel=0; channel < 6; channel++) {
        PFDRFout[channel] = Chrystal[channel] * (dbl?2:1) / R;
      }
      registers[2] &= ~ (((unsigned long)0x3FF) << 14);
      registers[2] |= (((unsigned long)R) << 14);
}

void ADF4351_CP(int p)
{
      registers[2] &= ~ (((unsigned long)0xF) << 9);
      registers[2] |= (((unsigned long)p) << 9);
}

void ADF4351_level(int p)
{
      registers[4] &= ~ (((unsigned long)0x3) << 3);
      registers[4] |= (((unsigned long)p) << 3);
}

void ADF4351_channel_spacing(int spacing)
{
  OutputChannelSpacing = 0.001 * spacing;
}

static uint32_t gcd(uint32_t x, uint32_t y)
{
  uint32_t z;
  while (y != 0) {
    z = x % y;
    x = y;
    y = z;
  }
  return x;
}

void ADF4351_prep_frequency(int channel, unsigned long freq, int drive)  // freq / 10Hz
{
  (void)drive;
//  if (channel == 0)
    RFout=freq/1000000.0;  // To MHz
//  else
 //   RFout=freq/1000002.764;  // To MHz

    if (RFout >= 2200) {
      OutputDivider = 1;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (RFout >= 1100) {
      OutputDivider = 2;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else if (RFout >= 550) {
      OutputDivider = 4;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    } else if (RFout >= 275)  {
      OutputDivider = 8;
      bitWrite (registers[4], 22, 0);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 1);
    } else if (RFout >= 137.5)  {
      OutputDivider = 16;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 0);
    } else if (RFout >= 68.75) {
      OutputDivider = 32;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 0);
      bitWrite (registers[4], 20, 1);
    } else {
      OutputDivider = 64;
      bitWrite (registers[4], 22, 1);
      bitWrite (registers[4], 21, 1);
      bitWrite (registers[4], 20, 0);
    }

    INTA = (RFout * OutputDivider) / PFDRFout[channel];
    MOD = (PFDRFout[channel] / OutputChannelSpacing) + 0.01;
//    MOD = 3125;
    FRACF = (((RFout * OutputDivider) / PFDRFout[channel]) - INTA) * MOD;
    FRAC = round(FRACF);

  while (FRAC > 4095 || MOD > 4095) {
    FRAC = FRAC >> 1;
    MOD = MOD >> 1;
 //   Serial.println( "MOD/FRAC reduced");
  }

    int32_t k = gcd(FRAC, MOD);
    if (k > 1) {
      FRAC /= k;
      MOD /= k;
//      Serial.print( "MOD/FRAC gcd reduced");
    }
//    while (denom >= (1<<20)) {
//      num >>= 1;
//      denom >>= 1;
//    }


//  if (INTA <= 75) Serial.println( "INTA <= 75");
//  if (FRAC > 4095) Serial.println( "FRAC > 4095");
//  if (MOD > 4095) Serial.println( "MOD > 4095");


//  if (FRAC > 4095) Serial.println( "FRAC > 4095");
//  if (MOD > 4095) Serial.println( "MOD > 4095");
//  if (INTA > 4095) Serial.println( "INT > 4095");

  if (debug) {
    DEBUG("  ODIV=");
    DEBUG(OutputDivider);
    DEBUG("  INT=");
    DEBUG(INTA);
    DEBUG("  FRAC=");
    DEBUG(FRAC);
    DEBUG("  MOD=");
    DEBUG(MOD);
    DEBUG( " CalF=");
//    DEBUGFLN(PFDRFout[channel] *(INTA + ((double)FRAC)/MOD)/OutputDivider,6);

//  DEBUG("  FRACF=");
//  DEBUGF(FRACF,6);
  }
    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;
    if (MOD == 1) MOD = 2;
    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // restore address "001"
    bitSet (registers[1], 27); // Prescaler at 8/9
/*
    drive = 1;
    if (drive == 0) {
      bitClear (registers[4], 3); // +5dBm + out
      bitClear (registers[4], 4); // +5dBm
      bitClear (registers[4], 6); // +5dBm - out
      bitClear (registers[4], 7); // +5dBm
    } else if (drive == 1) {
      bitSet (registers[4], 6); // +5dBm
      bitClear (registers[4], 7); // +5dBm - out
      bitSet (registers[4], 3); // +5dBm
      bitClear (registers[4], 4); // +5dBm + out
    } else if (drive == 2) {
      bitClear (registers[4], 6); // +5dBm - out
      bitSet (registers[4], 7); // +5dBm
      bitClear (registers[4], 3); // +5dBm + out
      bitSet (registers[4], 4); // +5dBm
    }
    else {
      bitSet (registers[4], 6); // +5dBm - out
      bitSet (registers[4], 7); // +5dBm
      bitSet (registers[4], 3); // +5dBm + out
      bitSet (registers[4], 4); // +5dBm
    }
*/
//    bitSet (registers[4], 5); // enable + output
//    bitClear (registers[4], 8); // enable B output

#if 0
    if (FRAC == 0)
      bitSet (registers[2], 8); // INT mode
    else
      bitClear (registers[2], 8); // INT mode
    bitSet (registers[2], 13); // Double buffered

    bitSet (registers[2], 28); // Digital lock == "110" sur b28 b27 b26
    bitSet (registers[2], 27); // digital lock
    bitClear (registers[2], 26); // digital lock

    //bitSet (registers[4], 10); // Mute till lock
//    bitSet (registers[3], 23); // Fast lock
 #endif
//   bitSet (registers[4], 10); // Mute till lock
//    ADF4351_Set(channel);
}



#endif
