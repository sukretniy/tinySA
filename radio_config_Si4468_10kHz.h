/*! @file radio_config.h
 * @brief This file contains the automatically generated
// # WB filter 3 (BW =  10.03 kHz);  NB-filter 3 (BW = 10.03 kHz)

/*
// Set properties:           RF_MODEM_TX_RAMP_DELAY_12_1
// Number of properties:     12
// Group ID:                 0x20
// Start ID:                 0x18
// Default values:           0x01, 0x00, 0x08, 0x03, 0xC0, 0x00, 0x10, 0x20, 0x00, 0x00, 0x00, 0x4B, 
// Descriptions:
//   MODEM_TX_RAMP_DELAY - TX ramp-down delay setting.
//   MODEM_MDM_CTRL - MDM control.
//   MODEM_IF_CONTROL - Selects Fixed-IF, Scaled-IF, or Zero-IF mode of RX Modem operation.
//   MODEM_IF_FREQ_2 - the IF frequency setting (an 18-bit signed number).
//   MODEM_IF_FREQ_1 - the IF frequency setting (an 18-bit signed number).
//   MODEM_IF_FREQ_0 - the IF frequency setting (an 18-bit signed number).
//   MODEM_DECIMATION_CFG1 - Specifies three decimator ratios for the Cascaded Integrator Comb (CIC) filter.
//   MODEM_DECIMATION_CFG0 - Specifies miscellaneous parameters and decimator ratios for the Cascaded Integrator Comb (CIC) filter.
//   MODEM_DECIMATION_CFG2 - Specifies miscellaneous decimator filter selections.
//   MODEM_IFPKD_THRESHOLDS - 
//   MODEM_BCR_OSR_1 - RX BCR/Slicer oversampling rate (12-bit unsigned number).
//   MODEM_BCR_OSR_0 - RX BCR/Slicer oversampling rate (12-bit unsigned number).
*/
#define RF_MODEM_TX_RAMP_DELAY_12_1 0x11, 0x20, 0x0C, 0x18, 0x01, 0x00, 0x0A, 0x03, 0x80, 0x00, 0xB0, 0x20, 0x14, 0xF9, 0x0F, 0xDF

/*
// Set properties:           RF_MODEM_BCR_NCO_OFFSET_2_12_1
// Number of properties:     12
// Group ID:                 0x20
// Start ID:                 0x24
// Default values:           0x06, 0xD3, 0xA0, 0x06, 0xD3, 0x02, 0xC0, 0x00, 0x00, 0x23, 0x83, 0x69, 
// Descriptions:
//   MODEM_BCR_NCO_OFFSET_2 - RX BCR NCO offset value (an unsigned 22-bit number).
//   MODEM_BCR_NCO_OFFSET_1 - RX BCR NCO offset value (an unsigned 22-bit number).
//   MODEM_BCR_NCO_OFFSET_0 - RX BCR NCO offset value (an unsigned 22-bit number).
//   MODEM_BCR_GAIN_1 - The unsigned 11-bit RX BCR loop gain value.
//   MODEM_BCR_GAIN_0 - The unsigned 11-bit RX BCR loop gain value.
//   MODEM_BCR_GEAR - RX BCR loop gear control.
//   MODEM_BCR_MISC1 - Miscellaneous control bits for the RX BCR loop.
//   MODEM_BCR_MISC0 - Miscellaneous RX BCR loop controls.
//   MODEM_AFC_GEAR - RX AFC loop gear control.
//   MODEM_AFC_WAIT - RX AFC loop wait time control.
//   MODEM_AFC_GAIN_1 - Sets the gain of the PLL-based AFC acquisition loop, and provides miscellaneous control bits for AFC functionality.
//   MODEM_AFC_GAIN_0 - Sets the gain of the PLL-based AFC acquisition loop, and provides miscellaneous control bits for AFC functionality.
*/
#define RF_MODEM_BCR_NCO_OFFSET_2_12_1 0x11, 0x20, 0x0C, 0x24, 0x00, 0x20, 0x44, 0x06, 0x4D, 0x02, 0xC0, 0x08, 0x00, 0x12, 0x00, 0x01

/*
// Set properties:           RF_MODEM_AFC_LIMITER_1_3_1
// Number of properties:     3
// Group ID:                 0x20
// Start ID:                 0x30
// Default values:           0x00, 0x40, 0xA0, 
// Descriptions:
//   MODEM_AFC_LIMITER_1 - Set the AFC limiter value.
//   MODEM_AFC_LIMITER_0 - Set the AFC limiter value.
//   MODEM_AFC_MISC - Specifies miscellaneous AFC control bits.
*/
#define RF_MODEM_AFC_LIMITER_1_3_1 0x11, 0x20, 0x03, 0x30, 0x4B, 0x1E, 0xA0

/*
// Set properties:           RF_MODEM_AGC_WINDOW_SIZE_12_1
// Number of properties:     12
// Group ID:                 0x20
// Start ID:                 0x38
// Default values:           0x11, 0x10, 0x10, 0x0B, 0x1C, 0x40, 0x00, 0x00, 0x2B, 0x0C, 0xA4, 0x03, 
// Descriptions:
//   MODEM_AGC_WINDOW_SIZE - Specifies the size of the measurement and settling windows for the AGC algorithm.
//   MODEM_AGC_RFPD_DECAY - Sets the decay time of the RF peak detectors.
//   MODEM_AGC_IFPD_DECAY - Sets the decay time of the IF peak detectors.
//   MODEM_FSK4_GAIN1 - Specifies the gain factor of the secondary branch in 4(G)FSK ISI-suppression.
//   MODEM_FSK4_GAIN0 - Specifies the gain factor of the primary branch in 4(G)FSK ISI-suppression.
//   MODEM_FSK4_TH1 - 16 bit 4(G)FSK slicer threshold.
//   MODEM_FSK4_TH0 - 16 bit 4(G)FSK slicer threshold.
//   MODEM_FSK4_MAP - 4(G)FSK symbol mapping code.
//   MODEM_OOK_PDTC - Configures the attack and decay times of the OOK Peak Detector.
//   MODEM_OOK_BLOPK - Configures the slicing reference level of the OOK Peak Detector.
//   MODEM_OOK_CNT1 - OOK control.
//   MODEM_OOK_MISC - Selects the detector(s) used for demodulation of an OOK signal, or for demodulation of a (G)FSK signal when using the asynchronous demodulator.
*/
#define RF_MODEM_AGC_WINDOW_SIZE_12_1 0x11, 0x20, 0x0C, 0x38, 0x11, 0xFF, 0xFF, 0x80, 0x02, 0x01, 0x48, 0x00, 0x2B, 0x0C, 0xA4, 0x23

/*
// Set properties:           RF_MODEM_RAW_CONTROL_10
// Number of properties:     10
// Group ID:                 0x20
// Start ID:                 0x45
// Default values:           0x02, 0x00, 0xA3, 0x02, 0x80, 0xFF, 0x0C, 0x01, 0x00, 0x40, 
// Descriptions:
//   MODEM_RAW_CONTROL - Defines gain and enable controls for raw / nonstandard mode.
//   MODEM_RAW_EYE_1 - 11 bit eye-open detector threshold.
//   MODEM_RAW_EYE_0 - 11 bit eye-open detector threshold.
//   MODEM_ANT_DIV_MODE - Antenna diversity mode settings.
//   MODEM_ANT_DIV_CONTROL - Specifies controls for the Antenna Diversity algorithm.
//   MODEM_RSSI_THRESH - Configures the RSSI threshold.
//   MODEM_RSSI_JUMP_THRESH - Configures the RSSI Jump Detection threshold.
//   MODEM_RSSI_CONTROL - Control of the averaging modes and latching time for reporting RSSI value(s).
//   MODEM_RSSI_CONTROL2 - RSSI Jump Detection control.
//   MODEM_RSSI_COMP - RSSI compensation value.
*/
#define RF_MODEM_RAW_CONTROL_10 0x11, 0x20, 0x0A, 0x45, 0x03, 0x00, 0x00, 0x01, 0x00, 0xFF, 0x06, 0x10, 0x10, 0x40

/*
// Set properties:           RF_MODEM_DSA_CTRL1_5_1
// Number of properties:     5
// Group ID:                 0x20
// Start ID:                 0x5B
// Default values:           0x00, 0x00, 0x00, 0x00, 0x00, 
// Descriptions:
//   MODEM_DSA_CTRL1 - Configures parameters for the Signal Arrival Detection circuit block and algorithm.
//   MODEM_DSA_CTRL2 - Configures parameters for the Signal Arrival Detection circuit block and algorithm.
//   MODEM_DSA_QUAL - Configures parameters for the Eye Opening qualification m ethod of the Signal Arrival Detection algorithm.
//   MODEM_DSA_RSSI - Signal Arrival Detect RSSI Qualifier Config
//   MODEM_DSA_MISC - Miscellaneous detection of signal arrival bits.
*/
#define RF_MODEM_DSA_CTRL1_5_1 0x11, 0x20, 0x05, 0x5B, 0x40, 0x04, 0x0A, 0x78, 0x20

/*
// Set properties:           RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1
// Number of properties:     12
// Group ID:                 0x21
// Start ID:                 0x00
// Default values:           0xFF, 0xBA, 0x0F, 0x51, 0xCF, 0xA9, 0xC9, 0xFC, 0x1B, 0x1E, 0x0F, 0x01, 
// Descriptions:
//   MODEM_CHFLT_RX1_CHFLT_COE13_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE12_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE11_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE10_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE9_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE8_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE7_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE6_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE5_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE4_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE3_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE2_7_0 - Filter coefficients for the first set of RX filter coefficients.
*/
#define RF_MODEM_CHFLT_RX1_CHFLT_COE13_7_0_12_1 0x11, 0x21, 0x0C, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11

/*
// Set properties:           RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1
// Number of properties:     12
// Group ID:                 0x21
// Start ID:                 0x0C
// Default values:           0xFC, 0xFD, 0x15, 0xFF, 0x00, 0x0F, 0xFF, 0xC4, 0x30, 0x7F, 0xF5, 0xB5, 
// Descriptions:
//   MODEM_CHFLT_RX1_CHFLT_COE1_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COE0_7_0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COEM0 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COEM1 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COEM2 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX1_CHFLT_COEM3 - Filter coefficients for the first set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE13_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE12_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE11_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE10_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE9_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE8_7_0 - Filter coefficients for the second set of RX filter coefficients.
*/
#define RF_MODEM_CHFLT_RX1_CHFLT_COE1_7_0_12_1 0x11, 0x21, 0x0C, 0x0C, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00, 0xCC, 0xA1, 0x30, 0xA0, 0x21, 0xD1

/*
// Set properties:           RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1
// Number of properties:     12
// Group ID:                 0x21
// Start ID:                 0x18
// Default values:           0xB8, 0xDE, 0x05, 0x17, 0x16, 0x0C, 0x03, 0x00, 0x15, 0xFF, 0x00, 0x00, 
// Descriptions:
//   MODEM_CHFLT_RX2_CHFLT_COE7_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE6_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE5_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE4_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE3_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE2_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE1_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COE0_7_0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COEM0 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COEM1 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COEM2 - Filter coefficients for the second set of RX filter coefficients.
//   MODEM_CHFLT_RX2_CHFLT_COEM3 - Filter coefficients for the second set of RX filter coefficients.
*/
#define RF_MODEM_CHFLT_RX2_CHFLT_COE7_7_0_12_1 0x11, 0x21, 0x0C, 0x18, 0xB9, 0xC9, 0xEA, 0x05, 0x12, 0x11, 0x0A, 0x04, 0x15, 0xFC, 0x03, 0x00
