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
  // always perform a system reset (don't send 0x87)
  SI4432_Write_Byte( 0x07, 0x80);
  chThdSleepMilliseconds(50);
  // wait for chiprdy bit
  while (count++ < 100 && ( SI4432_Read_Byte ( 0x04 ) & 0x02 ) == 0) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Transmit(int d)
{
  int count = 0;
  SI4432_Write_Byte(0x6D, (byte) (0x1C+d));
  if (( SI4432_Read_Byte ( 0x02 ) & 0x03 ) == 2)
    return; // Already in transmit mode
  chThdSleepMilliseconds(10);
  SI4432_Write_Byte( 0x07, 0x0b);
  chThdSleepMilliseconds(20);
  while (count++ < 100 && ( SI4432_Read_Byte ( 0x02 ) & 0x03 ) != 2) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Receive(void)
{
  int count = 0;
  if (( SI4432_Read_Byte ( 0x02 ) & 0x03 ) == 1)
    return; // Already in receive mode
  chThdSleepMilliseconds(10);
  SI4432_Write_Byte( 0x07, 0x07);
  chThdSleepMilliseconds(20);
  while (count++ < 100 && ( SI4432_Read_Byte ( 0x02 ) & 0x03 ) != 1) {
    chThdSleepMilliseconds(5);
  }
}


// First entry of each triple is RBW in khz times 10, so 377 = 37.7khz
// User asks for an RBW of WISH, go through table finding the last triple
// for which WISH is greater than the first entry, use those values,
// Return the first entry of the following triple for the RBW actually achieved
static const short RBW_choices[] = {     // Each triple is:  ndec, fils, WISH*10
     0, 5,1,26, 5,2,28, 5,3,31, 5,4,32, 5,5,37, 5,6,42, 5,7,
    45,4,1,    49,4,2,    54,4,3,    59,4,4,    61,4,5,    72,4,6,    82,4,7,
    88,3,1,    95,3,2,   106,3,3,   115,3,4,   121,3,5,   142,3,6,   162,3,7,
   175,2,1,   189,2,2,   210,2,3,   227,2,4,   240,2,5,   282,2,6,   322,2,7,
   347,1,1,   377,1,2,   417,1,3,   452,1,4,   479,1,5,   562,1,6,   641,1,7,
   692,0,1,   752,0,2,   832,0,3,   900,0,4,   953,0,5,  1121,0,6,  1279,0,7,
  1379,1,4,  1428,1,5,  1678,1,9,  1811,0,15, 1915,0,1,  2251,0,2,  2488,0,3,
  2693,0,4,  2849,0,8,  3355,0,9,  3618,0,10, 4202,0,11, 4684,0,12, 5188,0,13,
  5770,0,14,   6207
};

float SI4432_SET_RBW(float w)  {
  uint8_t dwn3=0;
  int32_t WISH = (uint32_t)(w * 10.0);
  uint8_t ndec, fils, i;
  if (WISH > 6207)   WISH=6207;     // Final value in RBW_choices[]
  if (WISH > 1379) dwn3 = 1 ;
  for (i=3; i<sizeof(RBW_choices)/sizeof(RBW_choices[0]); i+=3)
    if (WISH <= RBW_choices[i])  break;
  ndec = RBW_choices[i-2];
  fils = RBW_choices[i-1];
  WISH = RBW_choices[i];        // RBW achieved by Si4432 in Hz
  uint8_t BW = (dwn3 << 7) | (ndec << 4) | fils ;
  SI4432_Write_Byte(0x1C , BW ) ;
  return ((float)WISH / 10.0) ;
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
  int N = Freq / 10000000;
  Carrier = ( 4 * ( Freq - N * 10000000 )) / 625;
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


float SI4432_RSSI(uint32_t i, int s)
{
  (void) i;
  int RSSI_RAW;
  (void) i;
  // SEE DATASHEET PAGE 61
#ifdef USE_SI4463
  if (SI4432_Sel == 2) {
    RSSI_RAW = Si446x_getRSSI();
  } else
#endif
//START_PROFILE
    SI4432_Sel = s;
    chThdSleepMicroseconds(actualStepDelay);
    RSSI_RAW = (unsigned char)SI4432_Read_Byte( 0x26 ) ;
 //   if (MODE_INPUT(setting_mode) && RSSI_RAW == 0)
 //     SI4432_Init();
  float dBm = (RSSI_RAW-240)/2.0;
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
  SI4432_Receive();// Enable receiver chain
//  SI4432_Write_Byte(0x09, V0_XTAL_CAPACITANCE);// Tune the crystal
  SI4432_Set_Frequency(433700000);
  SI4432_Write_Byte(0x0D, 0x1F) ; // Set GPIO2 output to ground


  SI4432_Sel = 1;
//  SI4432_Write_Byte(0x09, V1_XTAL_CAPACITANCE);// Tune the crystal
  SI4432_Set_Frequency(443700000);
  SI4432_Write_Byte(0x6D, 0x1C);//Set low power
  SI4432_Transmit(0);

  SI4432_Write_Byte(0x0D, 0xC0) ; // Set GPIO2 maximumdrive and clock output
  SI4432_Write_Byte(0x0A, 0x02) ; // Set 10MHz output
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
           chThdSleepMicroseconds(PE4302_DELAY);
           SPI2_CLK_HIGH;
           chThdSleepMicroseconds(PE4302_DELAY);
           SPI2_CLK_LOW;
           chThdSleepMicroseconds(PE4302_DELAY);
     }
}

void PE4302_Write_Byte(unsigned char DATA )
{
  chThdSleepMicroseconds(PE4302_DELAY);
  SPI2_CLK_LOW;
  chThdSleepMicroseconds(PE4302_DELAY);
  PE4302_shiftOut(DATA);
  chThdSleepMicroseconds(PE4302_DELAY);
  CS_PE_HIGH;
  chThdSleepMicroseconds(PE4302_DELAY);
  CS_PE_LOW;
  chThdSleepMicroseconds(PE4302_DELAY);

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
