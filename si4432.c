/* All rights reserved.
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
#include "spi.h"

#pragma GCC push_options
#pragma GCC optimize ("Og")

// Define for use hardware SPI mode
#define USE_HARDWARE_SPI_MODE

// 10MHz clock
#define SI4432_10MHZ 10000000U
// !!!! FROM ili9341.c for disable it !!!!
#define LCD_CS_HIGH    palSetPad(GPIOB, GPIOB_LCD_CS)
#define SI_CS_LOW      palClearPad(GPIOA, GPIOA_SI_SEL)
#define SI_CS_HIGH     palSetPad(GPIOA, GPIOA_SI_SEL)

#define SI_SDN_LOW      palClearPad(GPIOC, 15)
#define SI_SDN_HIGH     palSetPad(GPIOC, 15)


// Hardware or software SPI use
#ifdef USE_HARDWARE_SPI_MODE
#define SI4432_SPI         SPI1
#define SI4432_SPI_SPEED   SPI_BR_DIV32
//#define SI4432_SPI_SPEED   SPI_BR_DIV64
static uint32_t old_spi_settings;
#else
static uint32_t old_port_moder;
static uint32_t new_port_moder;
#endif


#define CS_SI0_HIGH     palSetPad(GPIOB, GPIOB_RX_SEL)
#define CS_SI1_HIGH     palSetPad(GPIOB, GPIOB_LO_SEL)
#define CS_PE_HIGH      palSetPad(GPIOA, GPIOA_PE_SEL)

#define RF_POWER_HIGH   palSetPad(GPIOB, GPIOB_RF_PWR)
//#define SPI2_CLK_HIGH   palSetPad(GPIOB, GPIO_SPI2_CLK)
//#define SPI2_CLK_LOW    palClearPad(GPIOB, GPIO_SPI2_CLK)

#define CS_SI0_LOW      palClearPad(GPIOB, GPIOB_RX_SEL)
#define CS_SI1_LOW      palClearPad(GPIOB, GPIOB_LO_SEL)
#define CS_PE_LOW       palClearPad(GPIOA, GPIOA_PE_SEL)

#define SPI1_CLK_HIGH   palSetPad(GPIOB, GPIOB_SPI_SCLK)
#define SPI1_CLK_LOW    palClearPad(GPIOB, GPIOB_SPI_SCLK)

#define SPI1_SDI_HIGH   palSetPad(GPIOB, GPIOB_SPI_MOSI)
#define SPI1_SDI_LOW    palClearPad(GPIOB, GPIOB_SPI_MOSI)
#define SPI1_RESET      palClearPort(GPIOB, (1<<GPIOB_SPI_SCLK)|(1<<GPIOB_SPI_MOSI))

#define SPI1_SDO       ((palReadPort(GPIOB)>>GPIOB_SPI_MISO)&1)
#define SPI1_portSDO   (palReadPort(GPIOB)&(1<<GPIOB_SPI_MISO))
#ifdef __PE4302__
#define CS_PE_HIGH      palSetPad(GPIOC, GPIO_PE_SEL)
#define CS_PE_LOW      palClearPad(GPIOC, GPIO_PE_SEL)
#endif

//#define MAXLOG 1024
//unsigned char SI4432_logging[MAXLOG];
//volatile int log_index = 0;

//#define SI4432_log(X)   { if (log_index < MAXLOG)  SI4432_logging[log_index++] = X; }
#define SI4432_log(X)

void start_SI4432_SPI_mode(void){
#ifdef USE_HARDWARE_SPI_MODE
  old_spi_settings = SI4432_SPI->CR1;
  SPI_BR_SET(SI4432_SPI, SI4432_SPI_SPEED);
#else
  // Init legs mode for software bitbang
  old_port_moder = GPIOB->MODER;
  new_port_moder = old_port_moder & ~(PIN_MODE_ANALOG(GPIOB_SPI_SCLK)|PIN_MODE_ANALOG(GPIOB_SPI_MISO)|PIN_MODE_ANALOG(GPIOB_SPI_MOSI));
  new_port_moder|= PIN_MODE_OUTPUT(GPIOB_SPI_SCLK)|PIN_MODE_INPUT(GPIOB_SPI_MISO)|PIN_MODE_OUTPUT(GPIOB_SPI_MOSI);
  GPIOB->MODER = new_port_moder;
  // Pull down SPI
  SPI1_SDI_LOW;
  SPI1_CLK_LOW;
#endif
}

void stop_SI4432_SPI_mode(void){
#ifdef USE_HARDWARE_SPI_MODE
  SI4432_SPI->CR1 = old_spi_settings;
#else
  // Restore hardware SPI
  GPIOB->MODER = old_port_moder;
#endif
}

static void shiftOut(uint8_t val)
{
#ifdef USE_HARDWARE_SPI_MODE
  SPI_WRITE_8BIT(SI4432_SPI, val);
  while (SPI_IS_BUSY(SI4432_SPI)) // drop rx and wait tx
    (void)SPI_READ_8BIT(SI4432_SPI);
#else
  SI4432_log(SI4432_Sel);
  SI4432_log(val);
  uint8_t i = 0;
  do {
    SPI1_SDI_HIGH;
    SPI1_CLK_HIGH;
    SPI1_RESET;
    val<<=1;
  }while((++i) & 0x07);
#endif
}

static uint8_t shiftIn(void)
{
#ifdef USE_HARDWARE_SPI_MODE
  SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
  while (SPI_RX_IS_EMPTY(SI4432_SPI)); //wait rx data in buffer
  return SPI_READ_8BIT(SI4432_SPI);
#else
  uint32_t value = 0;
  uint8_t i = 0;
  do {
    value<<=1;
    SPI1_CLK_HIGH;
    value|=SPI1_portSDO;
    SPI1_CLK_LOW;
  }while((++i) & 0x07);
  return value>>GPIOB_SPI_MISO;
#endif
}

static inline void shiftInBuf(uint16_t sel, uint8_t addr, deviceRSSI_t *buf, uint16_t size, uint16_t delay) {
#ifdef USE_HARDWARE_SPI_MODE
  do{
    palClearPad(GPIOB, sel);
    SPI_WRITE_8BIT(SI4432_SPI, addr);
    while (SPI_IS_BUSY(SI4432_SPI))      // drop rx and wait tx
      (void)SPI_READ_8BIT(SI4432_SPI);

    SPI_WRITE_8BIT(SI4432_SPI, 0xFF);
    while (SPI_RX_IS_EMPTY(SI4432_SPI)); //wait rx data in buffer
    *buf++=SPI_READ_8BIT(SI4432_SPI);
    palSetPad(GPIOB, sel);
    if (delay)
      my_microsecond_delay(delay);
  }while(--size);
#else
  uint8_t i = 0;
  do{
    uint32_t value = addr;
    palClearPad(GPIOB, sel);
    do {
      if (value & 0x80)
        SPI1_SDI_HIGH;
      SPI1_CLK_HIGH;
      SPI1_RESET;
      value<<=1;
    }while((++i) & 0x07);
    value = 0;
    do {
      SPI1_CLK_HIGH;
      value<<=1;
      value|=SPI1_portSDO;
      SPI1_CLK_LOW;
    }while((++i) & 0x07);
    palSetPad(GPIOB, sel);
    *buf++=value>>GPIOB_SPI_MISO;
    if (delay)
      my_microsecond_delay(delay);
  }while(--size);
#endif
}
#if 0
static void shiftOutBuf(uint8_t *buf, uint16_t size) {
  uint8_t i = 0;
  do{
    uint8_t val = *buf++;
    do{
      if (val & 0x80)
        SPI1_SDI_HIGH;
      else
        SPI1_SDI_LOW;
      val<<=1;
      SPI1_CLK_HIGH;
      SPI1_CLK_LOW;
    }while((++i) & 0x07);
  }while(--size);
}
#endif

int SI4432_step_delay = 1500;
int SI4432_offset_delay = 1500;
#define MINIMUM_WAIT_FOR_RSSI   280

#ifdef __SI4432__
#define CS_SI0_HIGH     palSetPad(GPIOC, GPIO_RX_SEL)
#define CS_SI1_HIGH     palSetPad(GPIOC, GPIO_LO_SEL)

#define RF_POWER_HIGH   palSetPad(GPIOB, GPIO_RF_PWR)


#define CS_SI0_LOW     palClearPad(GPIOC, GPIO_RX_SEL)
#define CS_SI1_LOW     palClearPad(GPIOC, GPIO_LO_SEL)


const uint16_t SI_nSEL[MAX_SI4432+1] = { GPIO_RX_SEL, GPIO_LO_SEL, 0}; // #3 is dummy!!!!!!

volatile int SI4432_Sel = 0;         // currently selected SI4432
// volatile int SI4432_guard = 0;

#ifdef __SI4432_H__
#define SELECT_DELAY 10
void SI4432_Write_Byte(uint8_t ADR, uint8_t DATA )
{
  set_SPI_mode(SPI_MODE_SI);
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI1_CLK_LOW;
  palClearPad(GPIOB, SI_nSEL[SI4432_Sel]);
//  my_microsecond_delay(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA );
  palSetPad(GPIOB, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
}

void SI4432_Write_2_Byte(uint8_t ADR, uint8_t DATA1, uint8_t DATA2)
{
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI2_CLK_LOW;
  palClearPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  chThdSleepMicroseconds(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA1 );
  shiftOut( DATA2 );
  palSetPad(GPIOC, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
}

void SI4432_Write_3_Byte(uint8_t ADR, uint8_t DATA1, uint8_t DATA2, uint8_t DATA3 )
{
  set_SPI_mode(SPI_MODE_SI);
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI1_CLK_LOW;
  palClearPad(GPIOB, SI_nSEL[SI4432_Sel]);
//  my_microsecond_delay(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA1 );
  shiftOut( DATA2 );
  shiftOut( DATA3 );
  palSetPad(GPIOB, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
}

uint8_t SI4432_Read_Byte( uint8_t ADR )
{
  set_SPI_mode(SPI_MODE_SI);
  uint8_t DATA ;
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI1_CLK_LOW;
  palClearPad(GPIOB, SI_nSEL[SI4432_Sel]);
  shiftOut( ADR );
  DATA = shiftIn();
  palSetPad(GPIOB, SI_nSEL[SI4432_Sel]);
//  SI4432_guard = 0;
  return DATA ;
}



void SI4432_Reset(void)
{
  int count = 0;
  SI4432_Read_Byte (SI4432_INT_STATUS1);    // Clear pending interrupts
  SI4432_Read_Byte (SI4432_INT_STATUS2);
  // always perform a system reset (don't send 0x87)
  SI4432_Write_Byte(SI4432_STATE, 0x80);
  chThdSleepMilliseconds(10);
  // wait for chiprdy bit
  while (count++ < 100 && ( SI4432_Read_Byte (SI4432_INT_STATUS2) & 0x02 ) == 0) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Drive(int d)
{
  SI4432_Write_Byte(SI4432_TX_POWER, (uint8_t) (0x18+(d & 7)));
}

void SI4432_Transmit(int d)
{
  int count = 0;
  SI4432_Write_Byte(SI4432_TX_POWER, (uint8_t) (0x18+(d & 7)));
  if (( SI4432_Read_Byte(SI4432_DEV_STATUS) & 0x03 ) == 2)
    return; // Already in transmit mode
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x02);
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x0b);
  chThdSleepMilliseconds(10);
  while (count++ < 100 && ( SI4432_Read_Byte(SI4432_DEV_STATUS) & 0x03 ) != 2) {
    chThdSleepMilliseconds(10);
  }
}

void SI4432_Receive(void)
{
  int count = 0;
  if (( SI4432_Read_Byte (SI4432_DEV_STATUS) & 0x03 ) == 1)
    return; // Already in receive mode
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x02);
  chThdSleepMilliseconds(3);
  SI4432_Write_Byte(SI4432_STATE, 0x07);
  chThdSleepMilliseconds(10);
  while (count++ < 100 && ( SI4432_Read_Byte(SI4432_DEV_STATUS) & 0x03 ) != 1) {
    chThdSleepMilliseconds(5);
  }
}

// User asks for an RBW of WISH, go through table finding the last triple
// for which WISH is greater than the first entry, use those values,
// Return the first entry of the following triple for the RBW actually achieved
#define IF_BW(dwn3, ndec, filset) (((dwn3)<<7)|((ndec)<<4)|(filset))
typedef struct {
  uint8_t  reg;                   // IF_BW(dwn3, ndec, filset)
  int8_t   RSSI_correction_x_10;  // Correction * 10
  uint16_t RBWx10;                // RBW * 10 in kHz
}RBW_t; // sizeof(RBW_t) = 4 bytes
RBW_t RBW_choices[] = {
// BW register    corr  freq
//                              {IF_BW(0,5,1),0,26},
//                              {IF_BW(0,5,2),0,28},
                              {IF_BW(0,5,3),3,31},
                              {IF_BW(0,5,4),-3,32},
                              {IF_BW(0,5,5),6,37},
                              {IF_BW(0,5,6),5,42},
                              {IF_BW(0,5,7),5,45},
                              {IF_BW(0,4,1),0,49},
                              {IF_BW(0,4,2),0,54},
                              {IF_BW(0,4,3),0,59},
                              {IF_BW(0,4,4),0,61},
                              {IF_BW(0,4,5),5,72},
                              {IF_BW(0,4,6),5,82},
                              {IF_BW(0,4,7),5,88},
                              {IF_BW(0,3,1),0,95},
                              {IF_BW(0,3,2),0,106},
                              {IF_BW(0,3,3),2,115},
                              {IF_BW(0,3,4),0,121},
                              {IF_BW(0,3,5),5,142},
                              {IF_BW(0,3,6),5,162},
                              {IF_BW(0,3,7),5,175},
                              {IF_BW(0,2,1),0,189},
                              {IF_BW(0,2,2),0,210},
                              {IF_BW(0,2,3),3,227},
                              {IF_BW(0,2,4),0,240},
                              {IF_BW(0,2,5),5,282},
                              {IF_BW(0,2,6),5,322},
                              {IF_BW(0,2,7),5,347},
                              {IF_BW(0,1,1),0,377},
                              {IF_BW(0,1,2),0,417},
                              {IF_BW(0,1,3),1,452},
                              {IF_BW(0,1,4),0,479},
                              {IF_BW(0,1,5),5,562},
                              {IF_BW(0,1,6),5,641},
                              {IF_BW(0,1,7),5,692},
                              {IF_BW(0,0,1),0,752},
                              {IF_BW(0,0,2),0,832},
                              {IF_BW(0,0,3),0,900},
                              {IF_BW(0,0,4),-1,953},
                              {IF_BW(0,0,5),9,1121},
                              {IF_BW(0,0,6),2,1279},
                              {IF_BW(0,0,7),5,1379},
                              {IF_BW(1,1,4),20,1428},
                              {IF_BW(1,1,5),26,1678},
                              {IF_BW(1,1,9),-50,1811},
                              {IF_BW(1,0,15),-100,1915},
                              {IF_BW(1,0,1),20,2251},
                              {IF_BW(1,0,2),22,2488},
                              {IF_BW(1,0,3),21,2693},
                              {IF_BW(1,0,4),15,2849},
                              {IF_BW(1,0,8),-15,3355},
                              {IF_BW(1,0,9),-53,3618},
                              {IF_BW(1,0,10),-15,4202},
                              {IF_BW(1,0,11),-13,4684},
                              {IF_BW(1,0,12),-20,5188},
                              {IF_BW(1,0,13),-14,5770},
                              {IF_BW(1,0,14),-9,6207},


};

const int SI4432_RBW_count = ((int)(sizeof(RBW_choices)/sizeof(RBW_t)));

static pureRSSI_t SI4432_RSSI_correction = float_TO_PURE_RSSI(-120);

uint16_t SI4432_force_RBW(int i)
{
  SI4432_Write_Byte(SI4432_IF_FILTER_BW, RBW_choices[i].reg);                     // Write RBW settings to Si4432
  SI4432_RSSI_correction = float_TO_PURE_RSSI(RBW_choices[i].RSSI_correction_x_10 - 1200)/10;  // Set RSSI correction
//  SI4432_RSSI_correction = float_TO_PURE_RSSI( - 1200)/10;  // Set RSSI correction
  return RBW_choices[i].RBWx10;                                                   // RBW achieved by Si4432 in kHz * 10
}

uint16_t SI4432_SET_RBW(uint16_t WISH)  {
  int i;
  for (i=0; i < SI4432_RBW_count - 1; i++)
    if (WISH <= RBW_choices[i].RBWx10) 
      break; 
  return SI4432_force_RBW(i);
}


int SI4432_frequency_changed = false;
int SI4432_offset_changed = false;

// #define __CACHE_BAND__  // Is not reliable!!!!!!

#ifdef __CACHE_BAND__
static int old_freq_band[2] = {-1,-1};
static int written[2]= {0,0};
#endif

void SI4432_Set_Frequency ( uint32_t Freq ) {
//  int mode = SI4432_Read_Byte(0x02) & 0x03;           // Disabled as unreliable
//  SI4432_Write_Byte(0x07, 0x02);    // Switch to tune mode

//  Freq = (Freq / 1000 ) * 1000; // force freq to 1000 grid

  uint8_t hbsel;
  if (0) shell_printf("%d: Freq %q\r\n", SI4432_Sel, Freq);
  if (Freq >= 480000000U) {
    hbsel = 1<<5;
    Freq>>=1;
  } else {
    hbsel = 0;
  }
  uint8_t sbsel = 1 << 6;
  uint32_t N = (Freq / config.setting_frequency_10mhz - 24)&0x1F;
  uint32_t K = Freq % config.setting_frequency_10mhz;
  uint32_t Carrier = (K<<2) / 625;
  uint8_t Freq_Band = N | hbsel | sbsel;
//  int count = 0;
//  my_microsecond_delay(200);
//  int s;
//  while (count++ < 100 && ( (s = SI4432_Read_Byte ( 0x02 )) & 0x03 ) != 0) {
//    my_microsecond_delay(100);
//  }

#ifdef __CACHE_BAND__
  if (old_freq_band[SI4432_Sel] == Freq_Band) {
    if (written[SI4432_Sel] < 4) {
      SI4432_Write_Byte ( 0x75, Freq_Band );
      written[SI4432_Sel]++;
    }
    SI4432_Write_Byte(SI4432_FREQCARRIER_H, (Carrier>>8) & 0xFF );
    SI4432_Write_Byte(SI4432_FREQCARRIER_L, Carrier & 0xFF  );
  } else {
#endif
#if 0       // Do not use multi byte write
    SI4432_Write_Byte ( 0x75, Freq_Band );                          // Freq band must be written first !!!!!!!!!!!!
    SI4432_Write_Byte(SI4432_FREQCARRIER_H, (Carrier>>8) & 0xFF );
    SI4432_Write_Byte(SI4432_FREQCARRIER_L, Carrier & 0xFF  );
#else
    SI4432_Write_3_Byte (SI4432_FREQBAND, Freq_Band, (Carrier>>8) & 0xFF, Carrier & 0xFF);
#endif
#ifdef __CACHE_BAND__
    old_freq_band[SI4432_Sel] = Freq_Band;
    written[SI4432_Sel] = 0;
  }
#endif
  SI4432_frequency_changed = true;
//  if (mode == 1)        // RX mode            Disabled as unreliable
//    SI4432_Write_Byte( 0x07, 0x07);
//  else
//    SI4432_Write_Byte( 0x07, 0x0B);
}

//extern int setting.repeat;

#ifdef __FAST_SWEEP__
extern deviceRSSI_t age[POINTS_COUNT];
static int buf_index = 0;
static bool  buf_read = false;

#if 0
int SI4432_is_fast_mode(void)
{
  return buf_read;
}
#endif

void SI4432_Fill(int s, int start)
{
  set_SPI_mode(SPI_MODE_SI);
  SI4432_Sel = s;
  uint16_t sel = SI_nSEL[SI4432_Sel];
#if 0
  uint32_t t = calc_min_sweep_time_us(); // Time to delay in uS for all sweep
  if (t < setting.sweep_time_us){
    t = setting.sweep_time_us - t;
    t = t / (sweep_points - 1);          // Now in uS per point
  }
  else
    t = 0;
#endif
  uint32_t t = setting.additional_step_delay_us;
  systime_t measure = chVTGetSystemTimeX();
//  __disable_irq();
#if 0
  SPI1_CLK_LOW;
  int i = start;
  do {
    palClearPad(GPIOC, sel);
    shiftOut(SI4432_REG_RSSI);
    age[i]=(char)shiftIn();
    palSetPad(GPIOC, sel);
    if (++i >= sweep_points) break;
    if (t)
      my_microsecond_delay(t);
  } while(1);
#else
  shiftInBuf(sel, SI4432_REG_RSSI, &age[start], sweep_points - start, t);
#endif
//  __enable_irq();
  setting.measure_sweep_time_us = (chVTGetSystemTimeX() - measure)*100;
  buf_index = start; // Is used to skip 1st entry during level triggering
  buf_read = true;
}
#endif


pureRSSI_t getSI4432_RSSI_correction(void){
  return SI4432_RSSI_correction;
};

pureRSSI_t SI4432_RSSI(uint32_t i, int s)
{
  (void) i;
  int32_t RSSI_RAW;
  (void) i;
  // SEE DATASHEET PAGE 61
#ifdef USE_SI4463           // Not used!!!!!!!
  if (SI4432_Sel == 2) {
    RSSI_RAW = Si446x_getRSSI();
  } else
#endif
//START_PROFILE
#ifdef __FAST_SWEEP__
  if (buf_read) {
    if (buf_index == sweep_points-1)
      buf_read = false;
    return DEVICE_TO_PURE_RSSI(age[buf_index++]);
  }
#endif
  SI4432_Sel = s;
  int stepdelay = SI4432_step_delay;
  if (SI4432_frequency_changed) {
    if (stepdelay < MINIMUM_WAIT_FOR_RSSI) {
      stepdelay = MINIMUM_WAIT_FOR_RSSI;
    }
    SI4432_frequency_changed = false;
  } else if (SI4432_offset_changed) {
//    stepdelay = MINIMUM_WAIT_FOR_RSSI + (stepdelay - MINIMUM_WAIT_FOR_RSSI)/8;
    stepdelay = SI4432_offset_delay;
    SI4432_offset_changed = false;
  }
  if (stepdelay)
    my_microsecond_delay(stepdelay);
    // my_microsecond_delay(SI4432_step_delay);
  i = setting.repeat;
  RSSI_RAW  = 0;
  do{
    RSSI_RAW += DEVICE_TO_PURE_RSSI((deviceRSSI_t)SI4432_Read_Byte(SI4432_REG_RSSI));
    if (--i == 0) break;
    my_microsecond_delay(100);
  }while(1);

  if (setting.repeat > 1)
    RSSI_RAW = RSSI_RAW / setting.repeat;
 //   if (MODE_INPUT(setting.mode) && RSSI_RAW == 0)
 //     SI4432_Init();
#ifdef __SIMULATION__
#error "Fixme!!! add correct simulation in pureRSSI_t type"
  RSSI_RAW = Simulated_SI4432_RSSI(i,s);
#endif
//STOP_PROFILE
  // Serial.println(dBm,2);
  return RSSI_RAW;
}


void SI4432_Sub_Init(void)
{
  SI4432_Reset();


  SI4432_Write_Byte(SI4432_AGC_OVERRIDE, 0x60); //AGC override according to WBS3


#if 0           // Not sure if these add any value
  //set VCO and PLL Only for SI4432 V2
  SI4432_Write_Byte(SI4432_FREQ_DEVIATION, 0x1F); //write 0x1F to the Frequency Deviation register
  // VCO tuning registers
  SI4432_Write_Byte(SI4432_VCO_CURRENT_TRIM, 0x7F); //write 0x7F to the VCO Current Trimming register
  SI4432_Write_Byte(SI4432_CHARGEPUMP_OVERRIDE, 0x80); //write 0xD7 to the ChargepumpCurrentTrimmingOverride register
  SI4432_Write_Byte(SI4432_DIVIDER_CURRENT_TRIM, 0x40); //write 0x40 to the Divider Current Trimming register
#endif
#if 0
  //set the AGC,  BAD FOR PERFORMANCE!!!!!!
  SI4432_Write_Byte(0x6A, 0x0B); //write 0x0B to the AGC Override 2 register
  //set ADC reference voltage to 0.9V,  BAD FOR PERFORMANCE!!!!!!
  SI4432_Write_Byte(0x68, 0x04); //write 0x04 to the Deltasigma ADC Tuning 2 register

  SI4432_Write_Byte(0x1F, 0x03); //write 0x03 to the Clock Recovery Gearshift Override register

#endif


  SI4432_Write_Byte(SI4432_INT_ENABLE1, 0x0);
  SI4432_Write_Byte(SI4432_INT_ENABLE2, 0x0);
  // Enable receiver chain
//  SI4432_Write_Byte(SI4432_STATE, 0x05);
  // Clock Recovery Gearshift Value
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_GEARSHIFT, 0x00);
  // IF Filter Bandwidth
  SI4432_SET_RBW(100) ;
//  // Register 0x75 Frequency Band Select
//  uint8_t sbsel = 1 ;  // recommended setting
//  uint8_t hbsel = 0 ;  // low bands
//  uint8_t fb = 19 ;    // 430�439.9 MHz
//  uint8_t FBS = (sbsel << 6 ) | (hbsel << 5 ) | fb ;
//  SI4432_Write_Byte(SI4432_FREQBAND, FBS) ;
//  SI4432_Write_Byte(SI4432_FREQBAND, 0x46) ;
  // Register 0x76 Nominal Carrier Frequency
  // WE USE 433.92 MHz
  // Si443x-Register-Settings_RevB1.xls
//  SI4432_Write_Byte(SI4432_FREQCARRIER_H, 0x62) ;
//  SI4432_Write_Byte(SI4432_FREQCARRIER_H, 0x00) ;
  // Register 0x77 Nominal Carrier Frequency
//  SI4432_Write_Byte(SI4432_FREQCARRIER_L, 0x00) ;
  // RX MODEM SETTINGS
//  SI4432_Write_3_Byte(SI4432_IF_FILTER_BW, 0x81, 0x3C, 0x02) ;    // <----------
//  SI4432_Write_Byte(SI4432_IF_FILTER_BW, 0x81) ;    // <----------
  SI4432_Write_Byte(SI4432_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x00) ;
  SI4432_Write_Byte(SI4432_AFC_TIMING_CONTROL, 0x02) ;
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_GEARSHIFT, 0x03) ;
//  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_OVERSAMPLING, 0x78) ;    // <----------
//  SI4432_Write_3_Byte(SI4432_CLOCK_RECOVERY_OFFSET2, 0x01, 0x11, 0x11) ;    // <----------
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_OFFSET2, 0x01) ;
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_OFFSET1, 0x11) ;
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_OFFSET0, 0x11) ;
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_TIMING_GAIN1, 0x01) ;
  SI4432_Write_Byte(SI4432_CLOCK_RECOVERY_TIMING_GAIN0, 0x13) ;
  SI4432_Write_Byte(SI4432_AFC_LIMITER, 0xFF) ;

//  SI4432_Write_3_Byte(0x2C, 0x28, 0x0c, 0x28) ;    // <----------
//  SI4432_Write_Byte(Si4432_OOK_COUNTER_VALUE_1, 0x28) ;
//  SI4432_Write_Byte(Si4432_OOK_COUNTER_VALUE_2, 0x0C) ;
//  SI4432_Write_Byte(Si4432_SLICER_PEAK_HOLD, 0x28) ;

  SI4432_Write_Byte(SI4432_DATAACCESS_CONTROL, 0x61); // Disable all packet handling

  SI4432_Write_Byte(SI4432_AGC_OVERRIDE, 0x60); // AGC, no LNA, fast gain increment


// GPIO automatic antenna switching
  SI4432_Write_Byte(SI4432_GPIO0_CONF, 0x12) ; // Normal
  SI4432_Write_Byte(SI4432_GPIO1_CONF, 0x15) ;

//  SI4432_Receive();

}

#define V0_XTAL_CAPACITANCE 0x64
#define V1_XTAL_CAPACITANCE 0x64



void SI4432_Init()
{
  return;
#if 1

  CS_SI0_LOW;                       // Drop CS so power can be removed
  CS_SI1_LOW;                       // Drop CS so power can be removed
  CS_PE_LOW;                        // low is the default safe state
  SPI1_CLK_LOW;                     // low is the default safe state
  SPI1_SDI_LOW;                     // will be set with any data out

  palClearPad(GPIOA, GPIOA_RF_PWR);  // Drop power
  chThdSleepMilliseconds(10);      // Wait
  palSetPad(GPIOA, GPIOA_RF_PWR);    // Restore power
  CS_SI0_HIGH;                      // And set chip select lines back to inactive
  CS_SI1_HIGH;
  chThdSleepMilliseconds(10);      // Wait
#endif
  SPI1_CLK_LOW;
  //DebugLine("IO set");
  SI4432_Sel = SI4432_RX;
  SI4432_Sub_Init();

  SI4432_Sel = SI4432_LO;
  SI4432_Sub_Init();
//DebugLine("1 init done");

  SI4432_Sel = SI4432_RX;
//  SI4432_Receive();// Enable receiver chain
//  SI4432_Write_Byte(Si4432_CRYSTAL_OSC_LOAD_CAP, V0_XTAL_CAPACITANCE);// Tune the crystal
//  SI4432_Set_Frequency(433800000);
  SI4432_Write_Byte(SI4432_GPIO2_CONF, 0x1F) ; // Set GPIO2 output to ground


  SI4432_Sel = SI4432_LO;
//  SI4432_Write_Byte(Si4432_CRYSTAL_OSC_LOAD_CAP, V1_XTAL_CAPACITANCE);// Tune the crystal
//  SI4432_Set_Frequency(443800000);
  SI4432_Write_Byte(SI4432_GPIO2_CONF, 0x1F) ; // Set GPIO2 output to ground

  //  SI4432_Write_Byte(SI4432_TX_POWER, 0x1C);//Set low power
//  SI4432_Transmit(0);

//  SI4432_Write_Byte(SI4432_GPIO2_CONF, 0xC0) ; // Set GPIO2 maximumdrive and clock output
//  SI4432_Write_Byte(Si4432_UC_OUTPUT_CLOCK, 0x02) ; // Set 10MHz output
}

void SI4432_SetReference(int freq)
{
  SI4432_Sel = SI4432_LO;         //Select Lo module
  if (freq < 0 || freq > 7 ) {
    SI4432_Write_Byte(SI4432_GPIO2_CONF, 0x1F) ; // Set GPIO2 to GND
  } else {
    SI4432_Write_Byte(SI4432_GPIO2_CONF, 0xC0) ; // Set GPIO2 maximumdrive and clock output
    SI4432_Write_Byte(Si4432_UC_OUTPUT_CLOCK, freq & 0x07) ; // Set GPIO2 frequency
  }
}
#endif
#endif

//------------PE4302 -----------------------------------------------
#ifdef __PE4302__

// Comment out this define to use parallel mode PE4302

#define PE4302_en 10

void PE4302_init(void) {
  CS_PE_LOW;
}

#define PE4302_DELAY 100
#if 0
void PE4302_shiftOut(uint8_t val)
{
     uint8_t i;
     SI4432_log(SI4432_Sel);
     SI4432_log(val);
     for (i = 0; i < 8; i++)  {
           if (val & (1 << (7 - i)))
             SPI1_SDI_HIGH;
           else
             SPI1_SDI_LOW;
//           my_microsecond_delay(PE4302_DELAY);
           SPI1_CLK_HIGH;
//           my_microsecond_delay(PE4302_DELAY);
           SPI1_CLK_LOW;
//           my_microsecond_delay(PE4302_DELAY);
     }
}
#endif

static unsigned char old_attenuation = 255;
bool PE4302_Write_Byte(unsigned char DATA )
{
  if (old_attenuation == DATA)
    return false;
//  my_microsecond_delay(PE4302_DELAY);
//  SPI1_CLK_LOW;
//  my_microsecond_delay(PE4302_DELAY);
//  PE4302_shiftOut(DATA);

  shiftOut(DATA);
//  my_microsecond_delay(PE4302_DELAY);
  CS_PE_HIGH;
//  my_microsecond_delay(PE4302_DELAY);
  CS_PE_LOW;
//  my_microsecond_delay(PE4302_DELAY);
  return true;
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

#ifdef __ADF4351__

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CS_ADF0_HIGH     palSetPad(GPIOB, 10)
#define CS_ADF1_HIGH     palSetPad(GPIOB, 10)

#define CS_ADF0_LOW     palClearPad(GPIOB, 10)
#define CS_ADF1_LOW     palClearPad(GPIOB, 10)

//uint32_t registers[6] =  {0x320000, 0x8008011, 0x4E42, 0x4B3,0x8C803C , 0x580005} ;         //25 MHz ref

uint32_t registers[6] =  {0xA00000, 0x8000011, 0x4E42, 0x4B3,0xDC003C , 0x580005} ;         //10 MHz ref

int debug = 0;
int ADF4351_LE[2] = { 10, 11};
int ADF4351_Mux = 7;

int ADF4351_frequency_changed = false;

//#define DEBUG(X) // Serial.print( X )
//#define DEBUGLN(X) Serial.println( X )
//#define DEBUGFLN(X,Y) Serial.println( X,Y )
//#define DEBUGF(X,Y) Serial.print( X,Y )
#define DEBUG(X)
#define DEBUGLN(X)


#define XTAL    26.0
double RFout, //Output freq in MHz
  PFDRFout[6] = {XTAL,XTAL,XTAL,10.0,10.0,10.0}, //Reference freq in MHz
  Chrystal[6] = {XTAL,XTAL,XTAL,10.0,10.0,10.0},

  OutputChannelSpacing = 0.001, // = 0.01
  FRACF; // Temp

unsigned int long RFint,  // Output freq/10Hz
  INTA,         // Temp
  RFcalc, //UI
  MOD, //Temp
  FRAC; //Temp

uint8_t OutputDivider; // Temp
uint8_t lock=2; //Not used

// Lock = A4

void ADF4351_Setup(void)
{
//  palSetPadMode(GPIOA, 1, PAL_MODE_OUTPUT_PUSHPULL );
//  palSetPadMode(GPIOA, 2, PAL_MODE_OUTPUT_PUSHPULL );

//  SPI3_CLK_HIGH;
//  SPI3_SDI_HIGH;
  CS_ADF0_HIGH;
//  CS_ADF1_HIGH;

  //  bitSet (registers[2], 17); // R set to 8
//  bitClear (registers[2], 14); // R set to 8

//  while(1) {
//

  ADF4351_R_counter(1);

  ADF4351_set_frequency(0,2000000000,0);


//  ADF4351_set_frequency(1,150000000,0);
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
  for (int i = 3; i >= 0; i--) shiftOut((value >> (8 * i)) & 0xFF);
//  my_microsecond_delay(1);
  palSetPad(GPIOB, ADF4351_LE[channel]);
  my_microsecond_delay(1);        // Must
  palClearPad(GPIOB, ADF4351_LE[channel]);
//  my_microsecond_delay(1);      // Not needed
}

void ADF4351_Set(int channel)
{
  set_SPI_mode(SPI_MODE_SI);
  my_microsecond_delay(1);
  palClearPad(GPIOB, ADF4351_LE[channel]);
  my_microsecond_delay(1);

  for (int i = 5; i >= 0; i--) {
    ADF4351_WriteRegister32(channel, registers[i]);

//    if (debug)  Serial.println(registers[i],HEX);
}
}

void ADF4351_disable_output(void)
{
    bitClear (registers[4], 5); // digital lock
    ADF4351_Set(0);
}

void ADF4351_enable_output(void)
{
    bitSet (registers[4], 5); // digital lock
    ADF4351_Set(0);
}

uint32_t ADF4351_set_frequency(int channel, uint32_t freq, int drive)  // freq / 10Hz
{
//  freq -= 71000;

//  uint32_t offs = ((freq / 1000)* ( 0) )/ 1000;
  uint32_t offs = 0;
  uint32_t actual_freq = ADF4351_prep_frequency(channel,freq + offs, drive);
//START_PROFILE;
  ADF4351_frequency_changed = true;
  ADF4351_Set(channel);
//STOP_PROFILE;
  return actual_freq;
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
static int old_R;
  if (R == old_R)
    return;
  old_R = R;
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

uint32_t ADF4351_prep_frequency(int channel, unsigned long freq, int drive)  // freq / 10Hz
{
//  START_PROFILE;
//  if (channel == 0)
    RFout=freq/1000000.0; // config.setting_frequency_10mhz;  // To MHz
//  else
//    RFout=freq/1000210;  // To MHz

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
#if 0       // does not work on ADF4350
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
#endif
    }

    INTA = (RFout * OutputDivider) / PFDRFout[channel];
//    MOD = (PFDRFout[channel] / OutputChannelSpacing) + 0.01;
    MOD = 3125;
    FRACF = (((RFout * OutputDivider) / PFDRFout[channel]) - INTA) * MOD;
    FRAC = round(FRACF);

  while (FRAC > 4095 || MOD > 4095) {
    FRAC = FRAC >> 1;
    MOD = MOD >> 1;
 //   Serial.println( "MOD/FRAC reduced");
  }
#if 0               // Don't reduce for now <-----------------------------------------------------
    int32_t k = gcd(FRAC, MOD);
    if (k > 1) {
      FRAC /= k;
      MOD /= k;
//      Serial.print( "MOD/FRAC gcd reduced");
    }
#endif
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
    uint32_t actual_freq = 1000000 * PFDRFout[channel] *(INTA + ((double)FRAC)/MOD)/OutputDivider;
    registers[0] = 0;
    registers[0] = INTA << 15; // OK
    FRAC = FRAC << 3;
    registers[0] = registers[0] + FRAC;
    if (MOD == 1) MOD = 2;
    registers[1] = 0;
    registers[1] = MOD << 3;
    registers[1] = registers[1] + 1 ; // restore address "001"
    bitSet (registers[1], 27); // Prescaler at 8/9

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
//    STOP_PROFILE;
    return actual_freq;
}

#endif

// ------------------------------ SI4463 -------------------------------------


int SI4463_frequency_changed = false;
static int SI4463_band = -1;
static int SI4463_outdiv = -1;
static uint32_t SI4463_prev_freq = 0;
float SI4463_step_size = 0;
uint8_t SI4463_channel = 0;
int SI4463_R = 5;


#define MIN_DELAY   2

#include <string.h>

#define SI4463_READ_CTS       ((palReadPort(GPIOC)>>14)&1)

int SI4463_wait_for_cts(void)
{
  if (SI4463_READ_CTS)
    return 1;
  while (!SI4463_READ_CTS) {
//    ili9341_drawstring_7x13("Waiting               ", 50, 200);
    my_microsecond_delay(10);
  }
//  ili9341_drawstring_7x13("Proceed                 ", 50, 200);
  return 1;
}


void SI4463_write_byte(uint8_t ADR, uint8_t DATA)
{
  set_SPI_mode(SPI_MODE_SI);
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI1_CLK_LOW;
  palClearPad(GPIOB, GPIOB_RX_SEL);
  my_microsecond_delay(MIN_DELAY);
//  my_microsecond_delay(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  shiftOut( DATA );
  my_microsecond_delay(MIN_DELAY);
  palSetPad(GPIOB, GPIOB_RX_SEL);
  my_microsecond_delay(MIN_DELAY);
//  SI4432_guard = 0;
}

void SI4463_write_buffer(uint8_t ADR, uint8_t *DATA, int len)
{
  set_SPI_mode(SPI_MODE_SI);
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI1_CLK_LOW;
  palClearPad(GPIOB, GPIOB_RX_SEL);
  my_microsecond_delay(MIN_DELAY);
//  my_microsecond_delay(SELECT_DELAY);
  ADR |= 0x80 ; // RW = 1
  shiftOut( ADR );
  while (len-- > 0)
    shiftOut( *(DATA++) );
  my_microsecond_delay(MIN_DELAY);
  palSetPad(GPIOB, GPIOB_RX_SEL);
  my_microsecond_delay(MIN_DELAY);
//  SI4432_guard = 0;
}


uint8_t SI4463_read_byte( uint8_t ADR )
{
//  set_SPI_mode(SPI_MODE_SI);
  uint8_t DATA ;
//  if (SI4432_guard)
//    while(1) ;
//  SI4432_guard = 1;
//  SPI1_CLK_LOW;
  set_SPI_mode(SPI_MODE_SI);
  palClearPad(GPIOB, GPIOB_RX_SEL);
  my_microsecond_delay(MIN_DELAY);
  shiftOut( ADR );
  my_microsecond_delay(MIN_DELAY);
  DATA = shiftIn();
  my_microsecond_delay(MIN_DELAY);
  palSetPad(GPIOB, GPIOB_RX_SEL);
  my_microsecond_delay(MIN_DELAY);
//  SI4432_guard = 0;
  return DATA ;
}

uint8_t SI4463_get_response(void* buff, uint8_t len)
{
    uint8_t cts = 0;
#if 1
    cts = SI4463_READ_CTS;
    if (!cts) {
      return false;
    }
//    if (len == 0)
//      return true;
#endif
    set_SPI_mode(SPI_MODE_SI);
  //  if (SI4432_guard)
  //    while(1) ;
  //  SI4432_guard = 1;
  //  SPI1_CLK_LOW;
    palClearPad(GPIOB, GPIOB_RX_SEL);
    my_microsecond_delay(MIN_DELAY);
    shiftOut( SI446X_CMD_READ_CMD_BUFF );
    my_microsecond_delay(MIN_DELAY);
    cts = (shiftIn() == 0xFF);
    my_microsecond_delay(MIN_DELAY);
    if (cts)
    {
        // Get response data
        for(uint8_t i=0;i<len;i++) {
            ((uint8_t*)buff)[i] = shiftIn();
            my_microsecond_delay(MIN_DELAY);
        }
    }
    palSetPad(GPIOB, GPIOB_RX_SEL);
    my_microsecond_delay(MIN_DELAY);
    return cts;
}

uint8_t SI4463_wait_response(void* buff, uint8_t len, uint8_t use_timeout)
{
  uint16_t timeout = 40000;
  while(!SI4463_get_response(buff, len))
  {
    my_microsecond_delay(10);
    if(use_timeout && !--timeout)
    {
      ((char *)buff)[0] = 1;
      return 0;
    }
  }
  return 1;
}

void SI4463_do_api(void* data, uint8_t len, void* out, uint8_t outLen)
{
#if 0
  if(SI4463_wait_response(NULL, 0, true)) // Make sure it's ok to send a command
#else
  if (SI4463_wait_for_cts())
#endif
    {
//    set_SPI_mode(SPI_MODE_SI);
    palClearPad(GPIOB, GPIOB_RX_SEL);
    my_microsecond_delay(MIN_DELAY);
    for(uint8_t i=0;i<len;i++) {
      shiftOut(((uint8_t*)data)[i]); // (pgm_read_byte(&((uint8_t*)data)[i]));
      my_microsecond_delay(MIN_DELAY);
    }
//    my_microsecond_delay(MIN_DELAY);
    palSetPad(GPIOB, GPIOB_RX_SEL);
    my_microsecond_delay(MIN_DELAY);
#if 0
    if(((uint8_t*)data)[0] == SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
#if 0
      SI4463_wait_response(NULL, 0, false);
#else
      SI4463_wait_for_cts();
#endif
    else
#endif
#if 1
    SI4463_wait_for_cts();
#endif
    if(out != NULL) { // If we have an output buffer then read command response into it
//      if (((uint8_t*)data)[0] == SI446X_CMD_GET_MODEM_STATUS)
//        my_microsecond_delay(18);   // Prevent extra wait cycles
      SI4463_wait_response(out, outLen, true);
    }
  }
}

static void SI4463_set_properties(uint16_t prop, void* values, uint8_t len)
{
    // len must not be greater than 12

    uint8_t data[16] = {
        SI446X_CMD_SET_PROPERTY,
        (uint8_t)(prop>>8),
        len,
        (uint8_t)prop
    };

    // Copy values into data, starting at index 4
    memcpy(data + 4, values, len);

    SI4463_do_api(data, len + 4, NULL, 0);
}

#include "SI446x_cmd.h"

#if 0
#include "SI4463_radio_config.h"
static const uint8_t SI4463_config[] = RADIO_CONFIGURATION_DATA_ARRAY;
#endif

#ifdef __SI4468__
#undef RADIO_CONFIG_H_
#undef RADIO_CONFIGURATION_DATA_ARRAY
#include "radio_config_Si4468_default.h"

#define GLOBAL_GPIO_PIN_CFG 0x13, 0x07, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00
#define GLOBAL_CLK_CFG 0x11, 0x00, 0x01, 0x01, 0x00

//#undef RF_MODEM_RAW_CONTROL_10                      // Override RSSI averaging
//#define RF_MODEM_RAW_CONTROL_10 0x11, 0x20, 0x0A, 0x45, 0x03, 0x00, 0x00, 0x01, 0x00, 0xFF, 0x06, 0x18, 0x10, 0x40

//#undef RF_MODEM_AGC_CONTROL_1
//#define RF_MODEM_AGC_CONTROL_1 0x11, 0x20, 0x01, 0x35, 0x92             // Override AGC gain increase
#define RF_MODEM_AGC_CONTROL_1 0x11, 0x20, 0x01, 0x35, 0xE0 + 0x10 + 0x08 // slow AGC
//#undef RF_MODEM_RSSI_JUMP_THRESH_4
//#define RF_MODEM_RSSI_JUMP_THRESH_4 0x11, 0x20, 0x04, 0x4B, 0x06, 0x09, 0x10, 0x45  // Increase RSSI reported value with 2.5dB

#undef RF_GPIO_PIN_CFG
#define RF_GPIO_PIN_CFG GLOBAL_GPIO_PIN_CFG
#undef RF_GLOBAL_CLK_CFG_1
#define RF_GLOBAL_CLK_CFG_1 GLOBAL_CLK_CFG

static const uint8_t SI4468_config[] = RADIO_CONFIGURATION_DATA_ARRAY;
#endif

// Set new state
static void SI4463_set_state(si446x_state_t newState)
{
  uint8_t data[] = {
        SI446X_CMD_CHANGE_STATE,
        newState
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);
}

static void SI4463_clear_FIFO(void)
{
    // 'static const' saves 20 bytes of flash here, but uses 2 bytes of RAM
  static const uint8_t clearFifo[] = {
        SI446X_CMD_FIFO_INFO,
        SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX
  };
  SI4463_do_api((uint8_t*)clearFifo, sizeof(clearFifo), NULL, 0);
}

void SI4463_start_rx(uint8_t CHANNEL)
{
  uint8_t data[] = {
    SI446X_CMD_ID_START_RX,
    CHANNEL,
    0,
    0,
    0,
    8,
    SI446X_CMD_START_RX_ARG_NEXT_STATE2_RXVALID_STATE_ENUM_RX,
    SI446X_CMD_START_RX_ARG_NEXT_STATE3_RXINVALID_STATE_ENUM_RX
  };
retry:
  SI4463_do_api(data, sizeof(data), NULL, 0);
//  my_microsecond_delay(SI4432_offset_delay);

#if 0
  //  my_microsecond_delay(15000);
  si446x_state_t s = getState();
  if (s != SI446X_STATE_RX) {
    my_microsecond_delay(10);
    goto retry;
  }
#endif
}

void SI4463_clear_int_status()
{
  uint8_t data[9] = {
    SI446X_CMD_ID_GET_INT_STATUS
  };
  SI4463_do_api(data, 1, data, SI446X_CMD_REPLY_COUNT_GET_INT_STATUS);

}

void Si4463_set_refer(int ref)
{

  ref = 0;   // <--------------------- DISABLED FOR PROTOTYPE!!!!!!!!!!!!!!!!!!!!!!!!!


    if (ref >= 0) {
    uint8_t data[8] = {
      0x13, 0x07, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00    // GPIO_PIN_CFG GPIO 0 input ,1 CTS, clock div out
    };
    SI4463_do_api(data, 8, NULL, 0);

    uint8_t data2[5] = {
      0x11, 0x00, 0x01, 0x01, 0x40                      // GLOBAL_CLK_CFG Clock config
    };
    data2[4] |= ref<<3;
    SI4463_do_api(data2, 5, NULL, 0);
    } else {
      uint8_t data[8] = {
        0x13, 0x01, 0x08, 0x01, 0x12, 0x00, 0x00, 0x00    //  GPIO_PIN_CFG GLOBAL_CLK_CFG  GPIO 0,1 CTS, GPIO 2 clock div out
      };
      SI4463_do_api(data, 8, NULL, 0);

    }
}

si446x_info_t SI4463_info;

void Si446x_getInfo(si446x_info_t* info)
{
    uint8_t data[8] = {
        SI446X_CMD_PART_INFO
    };
    SI4463_do_api(data, 1, data, 8);

    info->chipRev   = data[0];
    info->part      = (data[1]<<8) | data[2];
    info->partBuild = data[3];
    info->id        = (data[4]<<8) | data[5];
    info->customer  = data[6];
    info->romId     = data[7];


    data[0] = SI446X_CMD_FUNC_INFO;
    SI4463_do_api(data, 1, data, 6);

    info->revExternal   = data[0];
    info->revBranch     = data[1];
    info->revInternal   = data[2];
    info->patch         = (data[3]<<8) | data[4];
    info->func          = data[5];
}

uint8_t SI4463_get_device_status()
{
  uint8_t data[2] =
  {
    SI446X_CMD_ID_REQUEST_DEVICE_STATE, 0, 0
  };
  SI4463_do_api(data, 1, data, SI446X_CMD_REPLY_COUNT_REQUEST_DEVICE_STATE);
  return(data[0]);
}



// Read a fast response register
uint8_t getFRR(uint8_t reg)
{
  set_SPI_mode(SPI_MODE_SI);
  return SI4463_read_byte(reg);
}

// Get current radio state
 si446x_state_t getState(void)
{
   SI4463_wait_for_cts();
   uint8_t state = getFRR(SI446X_CMD_READ_FRR_B);
    if(state == SI446X_STATE_TX_TUNE)
        state = SI446X_STATE_TX;
    else if(state == SI446X_STATE_RX_TUNE)
        state = SI446X_STATE_RX;
    else if(state == SI446X_STATE_READY2)
        state = SI446X_STATE_READY;
    return (si446x_state_t)state;
}

// Set new state
void setState(si446x_state_t newState)
{
    uint8_t data[] = {
        SI446X_CMD_CHANGE_STATE,
        newState
    };
    SI4463_do_api(data, sizeof(data), NULL, 0);
}

void set_RSSI_comp(void)
{
  // Set properties:           RF_MODEM_RSSI_COMP_1
  // Number of properties:     1
  // Group ID:                 0x20
  // Start ID:                 0x4E
  // Default values:           0x40,
  // Descriptions:
  //   MODEM_RSSI_JUMP_THRESH - Configures the RSSI Jump Detection threshold.
  //   MODEM_RSSI_CONTROL - Control of the averaging modes and latching time for reporting RSSI value(s).
  //   MODEM_RSSI_CONTROL2 - RSSI Jump Detection control.
  //   MODEM_RSSI_COMP - RSSI compensation value.
  //
  // #define RF_MODEM_RSSI_COMP_1 0x11, 0x20, 0x01, 0x4E, 0x40

  uint8_t data[5] = {
      0x11,
      0x20,
      0x01,
      0x4E,
      0x40
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);

}


int16_t Si446x_RSSI(void)
{

//  int s = Si446x_getInfo(&SI4463_info);

//  if (s != SI446X_STATE_RX)
//    SI4463_start_rx(0);

  volatile uint8_t data[3] = {
        SI446X_CMD_GET_MODEM_STATUS,
        0xFF
  };
//  volatile si446x_state_t s = getState();
//START_PROFILE;
  if (SI4432_step_delay && (ADF4351_frequency_changed || SI4463_frequency_changed) ) {
    my_microsecond_delay(SI4432_step_delay);
    ADF4351_frequency_changed = false;
    SI4463_frequency_changed = false;
  }

  int i = 3; //setting.repeat;
  int RSSI_RAW[3];
  do{



    again:
    data[0] = SI446X_CMD_GET_MODEM_STATUS;
    data[1] = 0xFF;

    SI4463_do_api(data, 2, data, 3);
    if (data[2] == 255)
      goto again;
//    if (data[2] == 0){
//      data[2] += 1;
//      goto again;
//    }
//    if (data[2] > 150)
//      data[2] = data[2]+1;
    RSSI_RAW[--i] = data[2] - 120 * 2;
    //  if (rssi < -238)
    //    while(1)
    //      rssi = rssi;
    //STOP_PROFILE;

    if (i == 0) break;
    my_microsecond_delay(100);
  }while(1);
#if 1
  int t;
  if (RSSI_RAW[0] > RSSI_RAW[1]) {
    t = RSSI_RAW[1];
    RSSI_RAW[1] = RSSI_RAW[0];
    RSSI_RAW[0] = t;
  }
  if (RSSI_RAW[1] > RSSI_RAW[2]) {
    t = RSSI_RAW[2];
    RSSI_RAW[2] = RSSI_RAW[1];
    RSSI_RAW[1] = t;
  }
  if (RSSI_RAW[0] > RSSI_RAW[1]) {
    t = RSSI_RAW[1];
    RSSI_RAW[1] = RSSI_RAW[0];
    RSSI_RAW[0] = t;
  }

  return DEVICE_TO_PURE_RSSI(RSSI_RAW[1]);
#else
  return DEVICE_TO_PURE_RSSI(RSSI_RAW[0]);
#endif

}

void SI446x_set_AGC_LNA(uint8_t v)
{

  uint8_t data[2] = {
      0xd0,                 // AGC_OVERRIDE
      v
  };
  SI4463_do_api(data, sizeof(data), NULL, 0);
#if 0
  if (v == 0) {
    data[0] = 0xd1;     // Read AGC?????? NO
    SI4463_do_api(data, 1, data, 1);
  }
#endif
}



// Do an ADC conversion
static uint16_t getADC(uint8_t adc_en, uint8_t adc_cfg, uint8_t part)
{
    uint8_t data[6] = {
        SI446X_CMD_GET_ADC_READING,
        adc_en,
        adc_cfg
    };
    SI4463_do_api(data, 3, data, 6);
    return (data[part]<<8 | data[part + 1]);
}

#ifndef __SI4468__
// -------------- 0.2 kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_12_1 0x11, 0x20, 0x0C, 0x18, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0xF0, 0x10, 0x74, 0xE8, 0x00, 0xA9
// #define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0xF0, 0x11
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0x0C, 0x01, 0xE4, 0xB9, 0x86, 0x55, 0x2B, 0x0B, 0xF8, 0xEF, 0xEF, 0xF2
//#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xC6, 0xC1, 0xB2, 0x9C, 0x80, 0x63, 0x47, 0x2F, 0x1B, 0x0E, 0x05, 0x00
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0xF8, 0xFC, 0x05, 0x00, 0xFF, 0x0F, 0x0C, 0x01, 0xE4, 0xB9, 0x86, 0x55
//#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x0F, 0xC6, 0xC1, 0xB2, 0x9C, 0x80, 0x63
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0x2B, 0x0B, 0xF8, 0xEF, 0xEF, 0xF2, 0xF8, 0xFC, 0x05, 0x00, 0xFF, 0x0F
//#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0x47, 0x2F, 0x1B, 0x0E, 0x05, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x0F

#define RF_MODEM_RAW_SEARCH2_2_1 0x11, 0x20, 0x02, 0x50, 0x94, 0x0A

#define RF_GLOBAL_CONFIG_1_1 0x11, 0x00, 0x01, 0x03, 0x20

uint8_t SI4463_RBW_02kHz[] =
{
 6, RF_MODEM_RAW_SEARCH2_2_1, \
 5, RF_GLOBAL_CONFIG_1_1, \
 0x0C, RF_MODEM_TX_RAMP_DELAY_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};
// -------------- 1kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0xF0, 0x11
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xC6, 0xC1, 0xB2, 0x9C, 0x80, 0x63, 0x47, 0x2F, 0x1B, 0x0E, 0x05, 0x00
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x0F, 0xC6, 0xC1, 0xB2, 0x9C, 0x80, 0x63
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0x47, 0x2F, 0x1B, 0x0E, 0x05, 0x00, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x0F

uint8_t SI4463_RBW_1kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};

// -------------- 3 kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0xF0, 0x11
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00

uint8_t SI4463_RBW_3kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};

// -------------- 10 kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0xB0, 0x20
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00


uint8_t SI4463_RBW_10kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};

// -------------- 30 kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x30, 0x10
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F

uint8_t SI4463_RBW_30kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};

// -------------- 100kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x20, 0x20
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F

uint8_t SI4463_RBW_100kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};

// -------------- 300kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x80, 0x08, 0x03, 0x80, 0x00, 0x00, 0x20
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00

uint8_t SI4463_RBW_300kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};



// -------------- 850kHz ----------------------------

#undef RF_MODEM_TX_RAMP_DELAY_8_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
#undef RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
#undef RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1

#define RF_MODEM_TX_RAMP_DELAY_8_1 0x11, 0x20, 0x08, 0x18, 0x01, 0x00, 0x08, 0x03, 0x80, 0x00, 0x00, 0x30
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01, 0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F

uint8_t SI4463_RBW_850kHz[] =
{
 0x0C, RF_MODEM_TX_RAMP_DELAY_8_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1, \
 0x10, RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1, \
 0x00
};
#else
// -------------- 0.2 kHz ----------------------------

#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_200Hz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_02kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 1kHz ----------------------------

#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_1kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_1kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 3 kHz ----------------------------
#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_3kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_3kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 10 kHz ----------------------------
#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_10kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_10kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 30 kHz ----------------------------
#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_30kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_30kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 100kHz ----------------------------
#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_100kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_100kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 300kHz ----------------------------

#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_300kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_300kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

// -------------- 850kHz ----------------------------

#undef RADIO_CONFIG_H_
#include "radio_config_Si4468_850kHz.h"
#include "radio_config_Si4468_short.h"

static const uint8_t SI4463_RBW_850kHz[] =
    RADIO_CONFIGURATION_DATA_ARRAY;

#endif
// User asks for an RBW of WISH, go through table finding the last triple
// for which WISH is greater than the first entry, use those values,
// Return the first entry of the following triple for the RBW actually achieved
#define IF_BW(dwn3, ndec, filset) (((dwn3)<<7)|((ndec)<<4)|(filset))
typedef struct {
  uint8_t  *reg;                   // IF_BW(dwn3, ndec, filset)
  int16_t   RSSI_correction_x_10;  // Correction * 10
  int16_t RBWx10;                // RBW in kHz
}RBW_t; // sizeof(RBW_t) = 8 bytes

static const RBW_t RBW_choices[] =
{
// BW register    corr  freq
 {SI4463_RBW_02kHz, 0,3},
 {SI4463_RBW_1kHz,  0,10},
 {SI4463_RBW_3kHz,  0,30},
 {SI4463_RBW_10kHz, 0,100},
 {SI4463_RBW_30kHz, 0,300},
 {SI4463_RBW_100kHz,0,1000},
 {SI4463_RBW_300kHz,0,3000},
 {SI4463_RBW_850kHz,0,8500},
};

const int SI4432_RBW_count = ((int)(sizeof(RBW_choices)/sizeof(RBW_t)));

static pureRSSI_t SI4463_RSSI_correction = float_TO_PURE_RSSI(-120);

uint16_t SI4463_force_RBW(int f)
{
#if 0
  SI_SDN_LOW;
  my_microsecond_delay(1000);
  SI_SDN_HIGH;
  my_microsecond_delay(1000);
  SI_SDN_LOW;
  my_microsecond_delay(14000);
#else
  setState(SI446X_STATE_READY);
  my_microsecond_delay(200);
#endif
  uint8_t *config = RBW_choices[f].reg;
  uint16_t i=0;
  while(config[i] != 0)
  {
    SI4463_do_api((void *)&config[i+1], config[i], NULL, 0);
    i += config[i]+1;
    my_microsecond_delay(200);
  }
  SI4463_clear_int_status();
retry:
  SI4463_start_rx(SI4463_channel);
  my_microsecond_delay(15000);
  si446x_state_t s = getState();
  if (s != SI446X_STATE_RX) {

    SI4463_start_rx(SI4463_channel);
    osalThreadSleepMilliseconds(1000);
    si446x_state_t s = getState();
    if (s != SI446X_STATE_RX) {
      ili9341_drawstring_7x13("Waiting for RX", 50, 200);
      osalThreadSleepMilliseconds(3000);
      goto retry;
    }
    ili9341_drawstring_7x13("Waiting done     ", 50, 200);
  }
  set_RSSI_comp();
  SI4463_RSSI_correction = float_TO_PURE_RSSI(RBW_choices[f].RSSI_correction_x_10 - 1200)/10;  // Set RSSI correction
  return RBW_choices[f].RBWx10;                                                   // RBW achieved by SI4463 in kHz * 10
}

uint16_t SI4463_SET_RBW(uint16_t WISH)  {
  int i;
  for (i=0; i < (int)(sizeof(RBW_choices)/sizeof(RBW_t)) - 1; i++)
    if (WISH <= RBW_choices[i].RBWx10 * 15/10)
      break;
  return SI4463_force_RBW(i);
}

#define freq_xco    26.0

#define Npresc 1    // 0=low / 1=High performance mode

static int prev_band = -1;

void SI4463_set_freq(uint32_t freq, uint32_t step_size)
{
//  uint32_t offs = ((freq / 1000)* 147) / 1000;
  uint32_t offs = 0;
  float RFout=(freq+offs)/1000000.0;  // To MHz
  if (RFout >= 822 && RFout <= 1140)         {       // till 1140MHz
    SI4463_band = 0;
    SI4463_outdiv = 4;
#if 0       // band 4 does not function
  } else if (RFout >= 568 && RFout <= 758 ) {    // works till 758MHz
    SI4463_band = 4;
    SI4463_outdiv = 6;
#endif
  } else if (RFout >= 420 && RFout <= 568) {    // works till 568MHz
    SI4463_band = 2;
    SI4463_outdiv = 8;
  } else if (RFout >= 329 && RFout <= 454) {    // works till 454MHz
    SI4463_band = 1;
    SI4463_outdiv = 10;
  } else if (RFout >= 274 && RFout <= 339) {    // to 339
    SI4463_band = 3;
    SI4463_outdiv = 12;
  } else if (RFout >= 136 && RFout <= 190){ // 136 { // To 190
    SI4463_band = 5;
    SI4463_outdiv = 24;
 }
  if (SI4463_band == -1)
    return;

  int S = 4 ;               // Aprox 100 Hz channels
  SI4463_step_size = S * (Npresc ? 2000000*freq_xco : 4000000*freq_xco) /  ((2<<18) * SI4463_outdiv);


  if (freq < SI4463_prev_freq || freq > SI4463_prev_freq + 255 * SI4463_step_size ) {

    SI4463_channel = 0;         // 128 <--------------------------BUG!!!!!!!!!!!!!!!!!!!!!!!!!!
    SI4463_prev_freq = freq - SI4463_channel * SI4463_step_size;

    RFout -=  SI4463_channel * SI4463_step_size / 1000000.0;       // shift for channel 128

    int32_t R = (RFout * SI4463_outdiv) / (Npresc ? 2*freq_xco : 4*freq_xco) - 1;        // R between 0x00 and 0x7f (127)
    float MOD = 524288.0;
    int32_t  F = (((RFout * SI4463_outdiv) / (Npresc ? 2*freq_xco : 4*freq_xco)) - R) * MOD;

    setState(SI446X_STATE_READY);
    my_microsecond_delay(100);

    /*
  // Set properties:           RF_FREQ_CONTROL_INTE_8
  // Number of properties:     8
  // Group ID:                 0x40
  // Start ID:                 0x00
  // Default values:           0x3C, 0x08, 0x00, 0x00, 0x00, 0x00, 0x20, 0xFF,
  // Descriptions:
  //   FREQ_CONTROL_INTE - Frac-N PLL Synthesizer integer divide number.
  //   FREQ_CONTROL_FRAC_2 - Frac-N PLL fraction number.
  //   FREQ_CONTROL_FRAC_1 - Frac-N PLL fraction number.
  //   FREQ_CONTROL_FRAC_0 - Frac-N PLL fraction number.
  //   FREQ_CONTROL_CHANNEL_STEP_SIZE_1 - EZ Frequency Programming channel step size.
  //   FREQ_CONTROL_CHANNEL_STEP_SIZE_0 - EZ Frequency Programming channel step size.
  //   FREQ_CONTROL_W_SIZE - Set window gating period (in number of crystal reference clock cycles) for counting VCO frequency during calibration.
  //   FREQ_CONTROL_VCOCNT_RX_ADJ - Adjust target count for VCO calibration in RX mode.
     */
    // #define RF_FREQ_CONTROL_INTE_8_1 0x11, 0x40, 0x08, 0x00, 0x41, 0x0D, 0xA9, 0x5A, 0x4E, 0xC5, 0x20, 0xFE
    uint8_t data[] = {
                      0x11, 0x40, 0x08, 0x00,
                      (uint8_t) R,                   //  R data[4]
                      (uint8_t) ((F>>16) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F>> 8) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F    ) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((S>> 8) & 255),     //  Step size data[8] .. data[9]
                      (uint8_t) ((S    ) & 255),     //  Step size data[8] .. data[9]
                      0x20,                   // Window gate
                      0xFF                    // Adj count
    };



    SI4463_do_api(data, sizeof(data), NULL, 0);

    if (SI4463_band != prev_band) {
    /*
  // Set properties:           RF_MODEM_CLKGEN_BAND_1
  // Number of properties:     1
  // Group ID:                 0x20
  // Start ID:                 0x51
  // Default values:           0x08,
  // Descriptions:
  //   MODEM_CLKGEN_BAND - Select PLL Synthesizer output divider ratio as a function of frequency band.
     */
    // #define RF_MODEM_CLKGEN_BAND_1 0x11, 0x20, 0x01, 0x51, 0x0A
    uint8_t data2[] = {
                       0x11, 0x20, 0x01, 0x51,
                       0x10 + (uint8_t)(SI4463_band + (Npresc ? 0x08 : 0))           // 0x08 for high performance mode, 0x10 to skip recal
    };
    SI4463_do_api(data2, sizeof(data2), NULL, 0);
    my_microsecond_delay(30000);
    prev_band = SI4463_band;
    }
    //  SI4463_clear_int_status();
    retry:
    SI4463_start_rx(SI4463_channel);
    my_microsecond_delay(200);
    si446x_state_t s = getState();
    if (s != SI446X_STATE_RX) {
      SI4463_start_rx(SI4463_channel);
      osalThreadSleepMilliseconds(1000);
      si446x_state_t s = getState();
      if (s != SI446X_STATE_RX) {
        osalThreadSleepMilliseconds(3000);
        goto retry;
      }
    }
  } else
  {
    SI4463_channel = (freq - SI4463_prev_freq)/SI4463_step_size;
    SI4463_start_rx(SI4463_channel);

/*
    uint8_t data[] = {
                      0x36,
                      (uint8_t) R,                   //  R data[4]
                      (uint8_t) ((F>>16) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F>> 8) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      (uint8_t) ((F    ) & 255),     //  F2,F1,F0 data[5] .. data[7]
                      0x00,
                      0xFF
    };
    SI4463_do_api(data, sizeof(data), NULL, 0);
    my_microsecond_delay(200);
    */
  }

  SI4463_frequency_changed = true;
}


void SI4463_init(void)
{
reset:
  SI_SDN_LOW;
  my_microsecond_delay(100);
  SI_SDN_HIGH;
  my_microsecond_delay(1000);
  SI_SDN_LOW;
  my_microsecond_delay(1000);
  ili9341_set_foreground(LCD_BRIGHT_COLOR_GREEN);
  while (!SI4463_READ_CTS) {
    ili9341_drawstring_7x13("Waiting", 50, 200);
    my_microsecond_delay(100);
  }
  ili9341_drawstring_7x13("Proceed", 50, 200);


#if 0
for(uint16_t i=0;i<sizeof(SI4463_config);i++)
{
  SI4463_do_api((void *)&SI4463_config[i+1], SI4463_config[i], NULL, 0);
  i += SI4463_config[i];
  my_microsecond_delay(200);

}
#endif
#ifdef __SI4468__
uint16_t i=0;
SI4463_do_api((void *)&SI4468_config[i+1], SI4468_config[i], NULL, 0);
i += SI4468_config[i]+1;
my_microsecond_delay(2000000);
#if 0
SI4463_do_api((void *)&SI4468_config[i+1], SI4468_config[i], NULL, 0);
i += SI4468_config[i]+1;
//my_microsecond_delay(2000000);
SI4463_do_api((void *)&SI4468_config[i+1], SI4468_config[i], NULL, 0);
i += SI4468_config[i]+1;
//my_microsecond_delay(1000000);
#endif
for(;i<sizeof(SI4468_config);i++)
{
  SI4463_do_api((void *)&SI4468_config[i+1], SI4468_config[i], NULL, 0);
  i += SI4468_config[i];
  my_microsecond_delay(2000);
}
#endif
//  SI4463_do_api((void *)&SI4463_config[1], SI4463_config[0], NULL, 0);

//  SI4463_set_freq(433800000,1000);

//#define SI446X_ADC_SPEED 10
//  RSSI = getADC(SI446X_ADC_CONV_BATT, (SI446X_ADC_SPEED<<4), 2);
volatile si446x_state_t s ;
again:
  Si446x_getInfo(&SI4463_info);
// s = getState();
//  SI4463_clear_int_status();
  SI4463_start_rx(SI4463_channel);
  my_microsecond_delay(15000);
  s = getState();
  if (s != SI446X_STATE_RX) {
    ili9341_drawstring_7x13("Waiting for RX", 50, 200);
    osalThreadSleepMilliseconds(3000);
    goto reset;
  }
  ili9341_drawstring_7x13("Waiting ready     ", 50, 200);
  // Si446x_RSSI();
// goto again;

  prev_band = -1; // 433MHz
}


#pragma GCC pop_options
