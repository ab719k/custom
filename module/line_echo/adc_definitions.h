/*******************************************************************************
 * adc_definitions.h - This file defines the ADS131A04 register settings
 * and other items to simplify communication to the ADS131A04
 *
 *  Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *  ALL RIGHTS RESERVED
 *
*/

#ifndef ADC_DEFINITIONS_H_
#define ADC_DEFINITIONS_H_

#include "stdint.h"
#include <inttypes.h>

//Function Prototypes and variables for communication to delta sigma ADC
void setup_ADS(void);
void setup_sar_adc(void);
int verifyCommand(int16_t command);
void sendCommand(int16_t command);
int verifyCommand_6words(int16_t command);
void sendCommand_6words(int16_t command);
void Trigger_SAR(void);

extern int32_t channel_data[8];
extern uint8_t controlTable[256];
extern  uint8_t recBuffer[36];
extern  uint8_t dummy_send[36];
extern uint8_t sar_recBuffer[10];





/* Definition of standard bits*/
#define BIT0                                        (uint16_t)(0x0001)
#define BIT1                                        (uint16_t)(0x0002)
#define BIT2                                        (uint16_t)(0x0004)
#define BIT3                                        (uint16_t)(0x0008)
#define BIT4                                        (uint16_t)(0x0010)
#define BIT5                                        (uint16_t)(0x0020)
#define BIT6                                        (uint16_t)(0x0040)
#define BIT7                                        (uint16_t)(0x0080)
#define BIT8                                        (uint16_t)(0x0100)
#define BIT9                                        (uint16_t)(0x0200)
#define BITA                                        (uint16_t)(0x0400)
#define BITB                                        (uint16_t)(0x0800)
#define BITC                                        (uint16_t)(0x1000)
#define BITD                                        (uint16_t)(0x2000)
#define BITE                                        (uint16_t)(0x4000)
#define BITF                                        (uint16_t)(0x8000)
#define BIT(x)                                      ((uint16_t)1 << (x))


//This determines if the references are valid based on the output of the corresponding reference comparators
#define REF1_OK                                     (P2IN & BIT7)>>7
#define REF2_OK                                     (P2IN & BIT6)>>6


/*ADC104S021 SAR Definitions*/

//Command to select which channels to convert next time.  This must be followed by a 0x00 byte.
#define SAR_ADC_CONVERT_CH1						    0x00
#define SAR_ADC_CONVERT_CH2						    0x08
#define SAR_ADC_CONVERT_CH3						    0x10
#define SAR_ADC_CONVERT_CH4						    0x18

//adc_value should be defined as signed 16-bit int but response_MSBbyte
//and response_LSBbyte as unsigned 8-bit numbers
#define SAR_CONVERSION_VALUE_PARSE(response_MSBbyte, response_LSBbyte, adc_value) \
{  adc_value= ((unsigned short)(((response_MSBbyte<<8) | response_LSBbyte)<<4 ))-((signed short) 32768);\
}

#define SAR_ADC0_CHANNEL_NUMBER_OFFSET              0
#define SAR_ADC1_CHANNEL_NUMBER_OFFSET              4
#define SWITCH_SAR_ADCs                             P5OUT^=BIT0|BIT1
#define IS_SAR_ADC0_ON                              !(P5OUT & BIT0)
#define IS_SAR_ADC1_ON                              !((P5OUT & BIT1)>>1)
#define TURN_SAR_ADC0_OFF                           P5OUT|=BIT0
#define TURN_SAR_ADC1_OFF                           P5OUT|=BIT1
#define TURN_SAR_ADC0_ON                            P5OUT&=~BIT0
#define TURN_SAR_ADC1_ON                            P5OUT&=~BIT1

/*End of ADC104S021 SAR Definitions*/

/*ADS131A04 Definitions*/


//Delta Sigma Interface Commands
#define CS_ENABLE   P5->OUT&=~BIT2;
#define CS_DISABLE  P5->OUT|=BIT2;
#define start_ads131a04_reset()  P4->OUT&=~BIT5;
#define end_ads131a04_reset()   P4->OUT|=BIT5;

//Read/Write Registers OPCODES
#define ADS131A04_READ_REGISTER_COMMAND_OPCODE							0x2000
#define ADS131A04_READ_MULTIPLE_REGISTERS_COMMAND_OPCODE				0x2000
#define ADS131A04_WRITE_REGISTER_COMMAND_OPCODE							0x4000
#define ADS131A04_WRITE_MULTIPLE_REGISTERS_COMMAND_OPCODE				0x6000

#define ADS131A04_READ_REGISTER_RESPONSE_OPCODE							0x2000
#define ADS131A04_READ_MULTIPLE_REGISTERS_RESPONSE_OPCODE				0x6000
#define ADS131A04_WRITE_REGISTER_RESPONSE_OPCODE						0x2000
#define ADS131A04_WRITE_MULTIPLE_REGISTERS_RESPONSE_OPCODE				0x4000

//System Commands
#define ADS131A04_NULL_COMMAND											0X0000
#define ADS131A04_RESET_COMMAND											0X0011
#define ADS131A04_STANDBY_COMMAND										0X0022
#define ADS131A04_WAKEUP_COMMAND										0X0033
#define ADS131A04_LOCK_COMMAND											0X0555
#define ADS131A04_UNCLOCK_COMMAND										0X0655
#define READ_REGISTER_COMMAND(address, register_count) 					(ADS131A04_READ_REGISTER_COMMAND_OPCODE | (address<<8) | register_count)
//Please note that register_count tells A04 device register_count+1 registers to read
#define READ_MULTIPLE_REGISTERS_COMMAND(address, register_count) 		(ADS131A04_READ_MULTIPLE_REGISTERS_COMMAND_OPCODE | (address<<8) | register_count)
#define WRITE_REGISTER_COMMAND(address, data) 							(ADS131A04_WRITE_REGISTER_COMMAND_OPCODE | (address<<8) | data)
#define WRITE_MULTIPLE_REGISTERS_COMMAND(address, register_count) 		(ADS131A04_WRITE_MULTIPLE_REGISTERS_COMMAND_OPCODE | (address<<8) | register_count)
#define WRITE_MULTIPLE_REGISTERS_ADDITIONAL_WORD(data1, data2)			((data1<<8) | data2)
//data1 is written to a register at adress N.  data2 is written to next register at address N+1.

//system responses
#define ADS131A04_READY													0xFF04
#define ADS131A04_STANDBY_ACK											0X0022
#define ADS131A04_WAKEUP_ACK											0X0033
#define ADS131A04_LOCK_ACK												0X0555
#define ADS131A04_UNCLOCK_ACK											0X0655
#define WRITE_MULTIPLE_REGISTERS_ACK(address, register_count) 			(ADS131A04_WRITE_MULTIPLE_REGISTERS_RESPONSE_OPCODE | (address<<8) | register_count)
#define WRITE_REGISTER_ACK(address, data) 								(ADS131A04_WRITE_REGISTER_RESPONSE_OPCODE | (address<<8) | data)
#define READ_REGISTER_RESPONSE_PARSE(response, op_code, address, data) \
{ \
	op_code=(response & 0xE000); \
	address = ((response & 0x1F00)>>8); \
	data=(response & 0x00FF);\
}
#define READ_MULTIPLE_REGISTERS_ACK(address, register_count) 			(ADS131A04_READ_MULTIPLE_REGISTERS_RESPONSE_OPCODE | (address<<8) | register_count)
#define READ_MULTIPLE_REGISTER_RESPONSE_PARSE(response, data1, data2) \
{ \
	data1=((response & 0xFF00)>>8);\
	data2=(response & 0x00FF);\
}

//Register Addresses
#define ID_MSB															0x00 //ID Control Register MSB
#define ID_LSB															0x01 //ID Control Register LSB
#define STAT_1															0x02 //Status 1 Register
#define STAT_P															0x03 //Positive Input Fault Detect Status
#define STAT_N															0x04 //Negative Input Fault Detect Status
#define STAT_S															0x05 //SPI Status Register
#define ERROR_CNT														0x06 //Error Count Register
#define STAT_M2															0x07 //Hardware Mode Pin Status Register
//#define RESERVED														0x08
//#define RESERVED														0x09
//#define RESERVED														0x0A
#define A_SYS_CFG														0x0B //Analog System Configuration Register
#define D_SYS_CFG														0x0C //Digital System Configuration Register
#define CLK1															0x0D //Clock Configuration 1 Register
#define CLK2															0x0E //Clock Configuration 2 Register
#define ADC_ENA															0x0F //ADC Channel Enable Register
//#define RESERVED														0x10
#define ADC1															0x11 //ADC Channel 1 Digital Gain Configuration
#define ADC2															0x12 //ADC Channel 2 Digital Gain Configuration
#define ADC3															0x13 //ADC Channel 3 Digital Gain Configuration
#define ADC4															0x14 //ADC Channel 4 Digital Gain Configuration

//ID_MSB Bits
#define DEV_ID_ADS131A02												0 //Device ID
#define DEV_ID_ADS131A04												BIT4
#define DEV_ID_MASK														BIT4 | BIT5 | BIT6 | BIT7
#define NU_CH_2CHANNELS													BIT1 //Channel count identification bits.
#define NU_CH_4CHANNELS													BIT2
#define NU_CH_MASK														BIT0 | BIT1 | BIT2 | BIT3

//STAT_1 Bits
//#define RESERVED														BIT7
#define F_OPC															BIT6 //Fault Command
#define F_SPI															BIT5 //Fault SPI
#define F_ADCIN															BIT4 //Fault ADC input
#define F_WDT															BIT3 //Watchdog timer timeout
#define F_RESYNC														BIT2 //Fault resynchronization
#define F_DRDY															BIT1 //Fault Data Ready
#define F_CHECK															BIT0 //Fault DIN Check

//STAT_P Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
#define F_IN4P															BIT3 //AIN4P threshold detect
#define F_IN3P															BIT2 //AIN3P threshold detect
#define F_IN2P															BIT1 //AIN2P threshold detect
#define F_IN1P															BIT0 //AIN1P threshold detect

//STAT_N Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
#define F_IN4N															BIT3 //AIN4N threshold detect
#define F_IN3N															BIT2 //AIN3N threshold detect
#define F_IN2N															BIT1 //AIN2N threshold detect
#define F_IN1N															BIT0 //AIN1N threshold detect

//STAT_S Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
//#define RESERVED														BIT3
#define F_STARTUP														BIT2 //ADC startup fault
#define F_CS															BIT1 //Chip-select fault
#define F_FRAME															BIT0 //Fame fault

//ERROR_CNT
#define ER_MASK															0xFF

//STAT_M2 Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
//#define RESERVED														BIT3
#define F_STARTUP														BIT2 //ADC startup fault
#define F_CS															BIT1 //Chip-select fault
#define F_FRAME															BIT0 //Fame fault


//STAT_M2 Bits
#define M2PIN_GND														0x00 //M2 Captured State
#define HAMMING_CODE_OFF												0x00
#define M2PIN_IOVDD														BIT4
#define HAMMING_CODE_ON													BIT4
#define M2PIN_NO_CONNECTION												BIT5
#define M2PIN_MASK														BIT4 | BIT5
#define M1PIN_GND														0x00 //M1 Captured State
#define TWENTY_FOUR_BIT_WORDS											0x00
#define M1PIN_IOVDD														BIT2
#define THIRTY_TWO_BIT_WORDS											BIT2
#define M1PIN_NO_CONNECTION												BIT3
#define SIXTEEN_BIT_WORDS												BIT3
#define M1PIN_MASK														BIT2 | BIT3
#define M0PIN_GND														0x00 //M0 Captured State
#define SYNC_MASTER_MODE												0x00
#define M0PIN_IOVDD														BIT0
#define ASYNC_SLAVE_MODE												BIT0
#define M0PIN_NO_CONNECTION												BIT1
#define SYNC_SLAVE_MODE													BIT1
#define M0PIN_MASK														BIT1 | BIT0

//A_SYS_CFG Bits
#define VNCPEN															BIT7 //Negative charge pump enable
#define HRM																BIT6 //High Resolution MOde
//#define RESERVED														BIT5 //Fault SPI
#define VREF_4V															BIT4 //Fault ADC input
#define INT_REFEN														BIT3 //Watchdog timer timeout
#define COMP_TH_0														0
#define COMP_TH_1														BIT0 //See Table in User Guide for exact values
#define COMP_TH_2														BIT1
#define COMP_TH_3														BIT0 | BIT1
#define COMP_TH_4														BIT2
#define COMP_TH_5														BIT2 | BIT0
#define COMP_TH_6														BIT2 | BIT1
#define COMP_TH_7														BIT0 | BIT1 | BIT2
#define COMP_TH_MASK													BIT0 | BIT1 | BIT2

//D_SYS_CFG Bits
#define WDT_EN															BIT7 //Watchdog timer enable
#define CRC_MODE														BIT6 //CRC mode select;
//0: CRC is valid on only the device words being sent and received 1: CRC is valid on all bits received and transmitted
#define DNDLY_6ns														0 //Done bar Delay
#define DNDLY_8ns														BIT4
#define DNDLY_10ns														BIT5
#define DNDLY_12ns														BIT4 | BIT5
#define DNDLY_MASK														BIT4 | BIT5
#define HIZDLY_6ns														0  //Hi-Z Delay(for DOUT)
#define HIZDLY_8ns														BIT2
#define HIZDLY_10ns														BIT3
#define HIZDLY_12ns														BIT2| BIT3
#define HIZDLY_MASK														BIT2 | BIT3
#define FIXED															BIT1 //Fixed word size enable
#define CRC_EN															BIT0 //Cyclic redundancy check enable

//CLK1 Bits
#define CLKSRC															BIT7
//ADC clock source; 0: XTAL1/CLKIN pin or XTAL1/CLKIN and XTAL2 pins, 1: SCLK pin
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
#define CLK_DIV_2														BIT1 //CLIKIN divider ratio
#define CLK_DIV_4														BIT2
#define CLK_DIV_6														BIT1 | BIT2
#define CLK_DIV_8														BIT3
#define CLK_DIV_10														BIT3 | BIT1
#define CLK_DIV_12														BIT3 | BIT2
#define CLK_DIV_14														BIT1 | BIT2 | BIT3
#define CLK_DIV_MASK													BIT1 | BIT2 | BIT3

//CLK2 Bits
#define ICLK_DIV_2														BIT5 //ICLK divide ratio
#define ICLK_DIV_4														BIT6
#define ICLK_DIV_6														BIT5 | BIT6
#define ICLK_DIV_8														BIT7
#define ICLK_DIV_10														BIT7 | BIT5
#define ICLK_DIV_12														BIT7 | BIT6
#define ICLK_DIV_14														BIT5 | BIT6 | BIT7
#define ICLK_DIV_MASK													BIT5 | BIT6 | BIT7
//#define RESERVED														BIT4
#define OSR_4096														0    //Oversampling ratio
#define OSR_2048														BIT0
#define OSR_1024														BIT1
#define OSR_800															BIT0 | BIT1
#define OSR_768															BIT2
#define OSR_512															BIT2 | BIT0
#define OSR_400															BIT2 | BIT1
#define OSR_384															BIT2 | BIT0 | BIT1
#define OSR_256															BIT3 //Oversampling ratio
#define OSR_200															BIT0 | BIT3
#define OSR_192															BIT1| BIT3
#define OSR_128															BIT0 | BIT1| BIT3
#define OSR_96															BIT2| BIT3
#define OSR_64															BIT2 | BIT0| BIT3
#define OSR_48															BIT2 | BIT1| BIT3
#define OSR_32															BIT2 | BIT0 | BIT1| BIT3
#define OSR_MASK														BIT0 | BIT1 | BIT2 | BIT3

//ADC_ENA Bits
#define ADC_ENA_DISABLE_ALL_CHANNELS									0
#define ADC_ENA_ENABLE_ALL_CHANNELS										BIT0 | BIT1 | BIT2 | BIT3
#define ADC_ENA_MASK													BIT0 | BIT1 | BIT2 | BIT3

//ADC1 Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
//#define RESERVED														BIT3
#define GAIN_CH1_GAIN1													0 //CLIKIN divider ratio
#define GAIN_CH1_GAIN2													BIT0
#define GAIN_CH1_GAIN4													BIT1
#define GAIN_CH1_GAIN8													BIT0 | BIT1
#define GAIN_CH1_GAIN16													BIT2
#define GAIN_CH1_MASK													BIT0 | BIT1 | BIT2

//ADC2 Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
//#define RESERVED														BIT3
#define GAIN_CH2_GAIN1													0 //CLIKIN divider ratio
#define GAIN_CH2_GAIN2													BIT0
#define GAIN_CH2_GAIN4													BIT1
#define GAIN_CH2_GAIN8													BIT0 | BIT1
#define GAIN_CH2_GAIN16													BIT2
#define GAIN_CH2_MASK													BIT0 | BIT1 | BIT2

//ADC3 Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
//#define RESERVED														BIT3
#define GAIN_CH3_GAIN1													0 //CLIKIN divider ratio
#define GAIN_CH3_GAIN2													BIT0
#define GAIN_CH3_GAIN4													BIT1
#define GAIN_CH3_GAIN8													BIT0 | BIT1
#define GAIN_CH3_GAIN16													BIT2
#define GAIN_CH3_MASK													BIT0 | BIT1 | BIT2

//ADC4 Bits
//#define RESERVED														BIT7
//#define RESERVED														BIT6
//#define RESERVED														BIT5
//#define RESERVED														BIT4
//#define RESERVED														BIT3
#define GAIN_CH4_GAIN1													0 //CLIKIN divider ratio
#define GAIN_CH4_GAIN2													BIT0
#define GAIN_CH4_GAIN4													BIT1
#define GAIN_CH4_GAIN8													BIT0 | BIT1
#define GAIN_CH4_GAIN16													BIT2
#define GAIN_CH4_MASK													BIT0 | BIT1 | BIT2

/* End of ADS131A04 Definitions*/

#endif /* ADC_DEFINITIONS_H_ */


