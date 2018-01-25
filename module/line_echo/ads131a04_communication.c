/*******************************************************************************
 * ads131a04_communication.c - Defines functions for communication to the ADS131A04
 *
 *  Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *  ALL RIGHTS RESERVED
 *
*/

#include "adc_definitions.h"
#include "msp.h"
#include "stdint.h"
#include <inttypes.h>
#include "driverlib.h"
#include "settings.h"

int32_t channel_data[8];
volatile int prev_bad=0;
 uint8_t recBuffer[36];
 uint8_t dummy_send[36]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x4E,
        0xC3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x4E, 0xC3, 0};

//#pragma DATA_ALIGN(controlTable, 256)
extern uint8_t controlTable[256];


//Setup the ADS131A04 port on the MSP432 and send the ADS131A04 commands to configure it.
void setup_ADS(void)
{

    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;                     // **Put state machine in reset**
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MST| EUSCI_B_CTLW0_SYNC|0|EUSCI_B_CTLW0_MSB;         // 3-pin, 8-bit SPI slave
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SSEL__SMCLK;                // ACLK
    UCB0BR0 = 7;                           // /2,fBitClock = fBRCLK/(UCBRx+1).
    UCB0BR1 = 0;                              //                           // No modulation
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
    //EUSCI_B0->IFG&=~EUSCI_B_IFG_TXIFG;

//    MAP_DMA_enableModule();  //MM Enables DMA
//    MAP_DMA_setControlBase(controlTable); //MM Table holds control information for DMA.


    MAP_DMA_assignChannel(DMA_CH0_EUSCIB0TX0);  //Select a source for one of the DMA channels
    MAP_DMA_assignChannel(DMA_CH1_EUSCIB0RX0); //Select a source for one of the DMA channels.  This case channel 1 is set to B0RX so transfer occurs when RX flag set


    //These functions just set up DMA.  It is not enabled until port interrupt which tells it to do a read.
    MAP_DMA_disableChannelAttribute(DMA_CH0_EUSCIB0TX0,
                                     UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                     UDMA_ATTR_HIGH_PRIORITY |
                                     UDMA_ATTR_REQMASK); //Disables all Channel attributes so can be set
    MAP_DMA_disableChannelAttribute(DMA_CH1_EUSCIB0RX0,
                                     UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST | UDMA_ATTR_HIGH_PRIORITY|
                                     UDMA_ATTR_REQMASK );
//    MAP_DMA_enableChannelAttribute(DMA_CH1_EUSCIB0RX0,
//                                     UDMA_ATTR_HIGH_PRIORITY); //Sets receive  channel to high priority

    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH0_EUSCIB0TX0,
            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH1_EUSCIB0RX0,
           UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1); //Sets control parameters for a DMA channel control structure.  1 item transferred before
    //rearbitrates bus.  After every transfer, increment destination because loading from SPI RX buffer and want to store all results.  Address increment of 8-bit bytes
    //because send char at one time.


    //Send Unlock Command
    while(!verifyCommand(ADS131A04_UNCLOCK_ACK))
    {
        sendCommand(ADS131A04_UNCLOCK_COMMAND);
    }

    //Send Wake-up Command
    while(!verifyCommand(ADS131A04_WAKEUP_ACK))
    {
        sendCommand(ADS131A04_WAKEUP_COMMAND);
    }


    //Set Iclk divider to 2
    while(!verifyCommand(WRITE_REGISTER_ACK(CLK1, CLK_DIV_2) ))
        {
            sendCommand(WRITE_REGISTER_COMMAND(CLK1, CLK_DIV_2));
        }

#if SAMPLES_PER_10_SECONDS == 40000
    //Sent Modclk divider to 8 and OSR=256
    while(!verifyCommand(WRITE_REGISTER_ACK(CLK2, ICLK_DIV_8 | OSR_256)))
    {
        sendCommand(WRITE_REGISTER_COMMAND(CLK2, ICLK_DIV_8 | OSR_256));
    }
#elif SAMPLES_PER_10_SECONDS == 53333
   //Sent Modclk divider to 6 and OSR=256
    while(!verifyCommand(WRITE_REGISTER_ACK(CLK2, ICLK_DIV_6 | OSR_256)))
    {
        sendCommand(WRITE_REGISTER_COMMAND(CLK2, ICLK_DIV_6 | OSR_256));
    }
#endif




    //high resolution mode, NO negative charge pump, internal reference; BIT5 is set because the User Guide mentions to always write 1 when writing to this register.
    while(!verifyCommand(WRITE_REGISTER_ACK(A_SYS_CFG, HRM | INT_REFEN | BIT5)))
    {
        sendCommand(WRITE_REGISTER_COMMAND(A_SYS_CFG, HRM | INT_REFEN| BIT5));
    }


    //This is mainly to have everything have a fixed frame size from now on.
    while(!verifyCommand_6words(WRITE_REGISTER_ACK(D_SYS_CFG, HIZDLY_12ns | DNDLY_12ns | FIXED)))
    {
        sendCommand(WRITE_REGISTER_COMMAND(D_SYS_CFG, HIZDLY_12ns | DNDLY_12ns | FIXED));
    }

    //Enable All ADCs
    while(!verifyCommand_6words(WRITE_REGISTER_ACK(ADC_ENA, ADC_ENA_ENABLE_ALL_CHANNELS)))
    {
        sendCommand_6words(WRITE_REGISTER_COMMAND(ADC_ENA, ADC_ENA_ENABLE_ALL_CHANNELS));
    }

    //CRC is valid on all bits received and transmitted, 12 ns after assert DONE when LSB shifted out, CRC enabled
    //12 ns time that the device asserts Hi-Z on DOUT after the LSB of the data frame is shifted out.
    while(!verifyCommand_6words(WRITE_REGISTER_ACK(D_SYS_CFG, HIZDLY_12ns | DNDLY_12ns | CRC_MODE | CRC_EN | FIXED)))
    {
        sendCommand_6words(WRITE_REGISTER_COMMAND(D_SYS_CFG, HIZDLY_12ns | DNDLY_12ns | CRC_MODE | CRC_EN | FIXED ));
    }


    //MM This may be duplicated in the main loop also.
//    MAP_DMA_enableModule();
//    MAP_DMA_setControlBase(controlTable);


    MAP_DMA_assignChannel(DMA_CH0_EUSCIB0TX0);
    MAP_DMA_assignChannel(DMA_CH1_EUSCIB0RX0);


    MAP_DMA_disableChannelAttribute(DMA_CH0_EUSCIB0TX0,
                                     UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                     UDMA_ATTR_HIGH_PRIORITY |
                                     UDMA_ATTR_REQMASK);
    MAP_DMA_disableChannelAttribute(DMA_CH1_EUSCIB0RX0,
                                     UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                     UDMA_ATTR_REQMASK);
    MAP_DMA_enableChannelAttribute(DMA_CH1_EUSCIB0RX0,
                                     UDMA_ATTR_HIGH_PRIORITY);

    MAP_DMA_assignInterrupt(DMA_INT1, 1);
    MAP_Interrupt_enableInterrupt(INT_DMA_INT1);
    Interrupt_setPriority(INT_DMA_INT1, 0x20);
    //MM There is an interrupt when done receiving.  There you put everything together.

    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH0_EUSCIB0TX0,
            UDMA_SIZE_8 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE | UDMA_ARB_1);
    MAP_DMA_setChannelControl(UDMA_PRI_SELECT | DMA_CH1_EUSCIB0RX0,
           UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_1);

}


//Send a command to both chained A04 devices when the word length is 1 with each word having a size of 3 bytes.
void sendCommand(int16_t command)
{
    char response;
    CS_ENABLE;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF00)>>8;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF);                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

//Send command to second A04 device
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF00)>>8;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF);                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    CS_DISABLE;


}

//Send a command to both chained A04 devices when the word length is 6 with each word having a size of 3-bytes.
void sendCommand_6words(int16_t command)
{
    char response;
    CS_ENABLE;
    int i=0;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF00)>>8;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF);                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    for(i=0; i<16; i++)
    {
        while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
        EUSCI_B0->TXBUF = 0;                  // Transmit characters
        while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
        response= EUSCI_B0->RXBUF;
        EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;
    }


    //Device 2

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF00)>>8;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (command&0xFF);                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    response= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;


    for(i=0; i<16; i++)
    {
        while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
        EUSCI_B0->TXBUF = 0;                  // Transmit characters
        while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
        response= EUSCI_B0->RXBUF;
        EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;
    }

    CS_DISABLE;


}



//Read the response from the A04 fir both chained A04 devices when the word length is 1 with each word having a size of 3 bytes.
int verifyCommand(int16_t command)
{
    int32_t test_command, received_command1, received_command2;
     test_command= (((int32_t) command)<<8) | 0x00;

    CS_ENABLE;

    //First A04 device
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command1= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command1<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                    // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command1|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command1<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command1|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    //Second A04 device
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command2= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command2<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                    // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command2|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command2<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command2|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;


    CS_DISABLE;

    if(received_command1==test_command && received_command2==test_command) return 1;
    else return 0;

}
//Read the response from the A04 fir both chained A04 devices when the word length is 6 with each word having a size of 3 bytes.
int verifyCommand_6words(int16_t command)
{
    int32_t test_command, received_command1, received_command2, i;
     test_command= (((int32_t) command)<<8) | 0x00;

    CS_ENABLE;

    //First A04 device
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command1= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command1<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                    // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command1|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command1<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command1|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

  for(i=0; i<15; i++)
  {
        while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
        EUSCI_B0->TXBUF = 0;                  // Transmit characters
        while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
        EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;
  }


    //Second A04 device
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command2= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command2<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                    // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command2|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    received_command2<<=8;
    while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = 0;                  // Transmit characters
    while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
    received_command2|= EUSCI_B0->RXBUF;
    EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    for(i=0; i<15; i++)
    {
        while (!(EUSCI_B0->IFG&EUSCI_B_IFG_TXIFG));
        EUSCI_B0->TXBUF = 0;                  // Transmit characters
        while (!(EUSCI_B0->IFG&EUSCI_A_IFG_RXIFG));
        EUSCI_B0->IFG &= ~EUSCI_A_IFG_RXIFG;

    }
    CS_DISABLE;
    if(received_command1==test_command && received_command2==test_command) return 1;
    else return 0;

}


//After all the ADCs have been enabled, the A04 device should send the MSP432 info on the ADC values.
//Parse the packets received from the A04 into actual ADC values.
void dma_1_interrupt(void)
{
    int i;
     uint16_t CRC16_Result;
    //P6->OUT|=BIT1;
    uint16_t CRC_Check;
    CS_DISABLE;

    //If a04 device 1 sent previously bad data or there were no previously bad data from both A04 devices then execute

    if((prev_bad & BIT0) || (prev_bad==0)  )
    {
    //MM Finished Transfer so now assemble all received bytes.  If error in transmission, then you resend.
        CRC_Check=(((uint16_t) recBuffer[15])<<8) | (((uint16_t) recBuffer[16]));
        CRC32->INIRES16 = 0xFFFF;                 // Init CRC16 HW module
          for(i=0;i<7;i++)
          {
            // Input random values into CRC Hardware
              CRC32->DIRB16 = (((uint16_t )recBuffer[2*i])<<8) | recBuffer[2*i+1];             // Input data in CRC
            __no_operation();
          }

          // Save results (per CRC-CCITT standard)
          CRC16_Result = (unsigned char) (CRC32->INIRES16 >> 8) | (CRC32->INIRES16 << 8);
          CRC16_Result ^= recBuffer[14];
          CRC16_Result ^= (unsigned char) (CRC16_Result & 0xFF) >> 4;
          CRC16_Result ^= (CRC16_Result << 8) << 4;
          CRC16_Result ^= ((CRC16_Result & 0xFF) << 4) << 1;
          if(CRC_Check==CRC16_Result)
          {
              //Only update value for channel_data if CRC is correct
                //response[0]= (((uint32_t) recBuffer[0])<<16) | (((uint32_t) recBuffer[1])<<8) | recBuffer[2];
                channel_data[0]= ((((int32_t) recBuffer[3])<<24)>>8) | (((uint32_t) recBuffer[4])<<8) | recBuffer[5];
                channel_data[1]= ((((int32_t) recBuffer[6])<<24)>>8) | (((uint32_t) recBuffer[7])<<8) | recBuffer[8];
                channel_data[2]= ((((int32_t) recBuffer[9])<<24)>>8) | (((uint32_t) recBuffer[10])<<8) | recBuffer[11];
                channel_data[3]= ((((int32_t) recBuffer[12])<<24)>>8) | (((uint32_t) recBuffer[13])<<8) | recBuffer[14];
                prev_bad&=~BIT0;
          }
          else
          {
              //If CRC is not correct, reenable the DMA so that could receive data again.
              prev_bad^=BIT0;
          }
    }

    //If a04 device 2 sent previously bad data or there were no previously bad data from both A04 devices then execute
    if((prev_bad & BIT1) || (prev_bad==0)  )
    {
    //MM Finished Transfer so now assemble all received bytes.  If error in transmission, then you resend.
        CRC_Check=(((uint16_t) recBuffer[33])<<8) | (((uint16_t) recBuffer[34]));
        CRC32->INIRES16 = 0xFFFF;                 // Init CRC16 HW module
          for(i=0;i<7;i++)
          {
            // Input random values into CRC Hardware
              CRC32->DIRB16 = (((uint16_t )recBuffer[2*i+18])<<8) | recBuffer[2*i+19];             // Input data in CRC
            __no_operation();
          }

          // Save results (per CRC-CCITT standard)
          CRC16_Result = (unsigned char) (CRC32->INIRES16 >> 8) | (CRC32->INIRES16 << 8);
          CRC16_Result ^= recBuffer[32];
          CRC16_Result ^= (unsigned char) (CRC16_Result & 0xFF) >> 4;
          CRC16_Result ^= (CRC16_Result << 8) << 4;
          CRC16_Result ^= ((CRC16_Result & 0xFF) << 4) << 1;
          if(CRC_Check==CRC16_Result)
          {
              //Only update value for channel_data if CRC is correct
                //response[0]= (((uint32_t) recBuffer[0])<<16) | (((uint32_t) recBuffer[1])<<8) | recBuffer[2];
              channel_data[4]= ((((int32_t) recBuffer[21])<<24)>>8) | (((uint32_t) recBuffer[22])<<8) | recBuffer[23];
              channel_data[5]= ((((int32_t) recBuffer[24])<<24)>>8) | (((uint32_t) recBuffer[25])<<8) | recBuffer[26];
              channel_data[6]= ((((int32_t) recBuffer[27])<<24)>>8) | (((uint32_t) recBuffer[28])<<8) | recBuffer[29];
              channel_data[7]= ((((int32_t) recBuffer[30])<<24)>>8) | (((uint32_t) recBuffer[31])<<8) | recBuffer[32];
                prev_bad&=~BIT1;
          }
          else
          {
              //If CRC is not correct, reenable the DMA so that could receive data again but do not do this if this is the second time asking to resend.
              prev_bad^=BIT1;


          }
    }

    if(prev_bad!=0)
    {
            CS_ENABLE;
            MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH0_EUSCIB0TX0,
                    UDMA_MODE_BASIC, dummy_send,
                    (void*) &EUSCI_B0->TXBUF, 36);
            MAP_DMA_setChannelTransfer(UDMA_PRI_SELECT | DMA_CH1_EUSCIB0RX0,
                    UDMA_MODE_BASIC,
                    (void*)&EUSCI_B0->RXBUF, recBuffer,
                    36);
            /* Now that the DMA is primed and setup, enabling the channels. The EUSCI
             * hardware should take over and transfer/receive all bytes */
            MAP_DMA_enableChannel(0);
            MAP_DMA_enableChannel(1);

    }

      //P6->OUT&=~BIT1;

}

