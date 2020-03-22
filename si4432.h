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

#endif //__SI4432_H__
