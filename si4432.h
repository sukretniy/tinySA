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


#ifndef __SI4432_H__

#define __SI4432_H__

#define byte uint8_t
extern volatile int SI4432_Sel;         // currently selected SI4432
void SI4432_Write_Byte(byte ADR, byte DATA );
byte SI4432_Read_Byte( byte ADR );

void SI4432_Init(void);
float SI4432_RSSI(uint32_t i, int s);
#ifdef __SIMULATION__
float Simulated_SI4432_RSSI(uint32_t i, int s);
#endif
void SI4432_Set_Frequency ( long Freq );
void SI4432_Transmit(int d);
void SI4432_Receive(void);
float SI4432_SET_RBW(float WISH);
void PE4302_Write_Byte(unsigned char DATA );
void PE4302_init(void);

#ifdef __ULTRA_SA__
extern int ADF4351_LE[];
extern int debug;
void   ADF4351_Setup(void);


void ADF4351_WriteRegister32(int channel, const uint32_t value);
void ADF4351_set_frequency(int channel, uint32_t freq, int drive_strength);
void ADF4351_prep_frequency(int channel, uint32_t freq, int drive_strength);
//int ADF4351_set_frequency_with_offset(uint32_t freq, int offset, uint8_t drive_strength);
void ADF4351_Set(int channel);
void ADF4351_enable_output(void);
void ADF4351_disable_output(void);
void ADF4351_spur_mode(int S);
void ADF4351_R_counter(int R);
void ADF4351_channel_spacing(int spacing);
void ADF4351_CP(int p);
void ADF4351_level(int p);
int ADF4351_locked(void);
#endif


#endif //__SI4432_H__
