#include "mbed.h"
#include <math.h>
#include <stdlib.h>
#include "radio.h"
#include "sx1280-hal.h"
#include "Eeprom.h"
#include "DemoApplication.h"
#include "FreqLUT.h"
#include "stm32l4xx_hal_uart.h"

#define Payload_Length          16
#define BUFFER_SIZE             255
#define RX_TIMEOUT_TICK_SIZE    RADIO_TICK_SIZE_1000_US

/*!
 * \brief Buffer and its size
 */
uint8_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

/*!
 * \brief Define IO and callbacks for radio
 * mosi, miso, sclk, nss, busy, dio1, dio2, dio3, rst, callbacks
 */
SX1280Hal Radio( D11, D12, D13, D7, D3, D5, NC, NC, A0, NULL );

/*!
 * \brief Mask of IRQs
 */
uint16_t g_vIrqMask = 0x0000;

/*************************************************
Function: Radio_Init(void) 
Description: Radio Init 
Input: NULL     
Output: NULL 
Return: NULL 
*************************************************/
void Radio_Init( void )
{
    wait_ms( 500 ); // wait for on board DC/DC start-up time

    Radio.Init( );

	// Can also be set in LDO mode but consume more power.0:LDO,1:DC-DC
	  Radio.SetRegulatorMode( ( RadioRegulatorModes_t )0x01 );  // 0x01: DC-DC(default)
                                                              // 0x00: LDO
    Radio.SetStandby( STDBY_RC );

    memset( &Buffer, 0x00, BufferSize );
}

/*************************************************
Function: UART_Init(void) 
Description: UART Init 
Input: NULL     
Output: NULL 
Return: NULL 
*************************************************/
UART_HandleTypeDef huart2;
void UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_UART_Init(&huart2);
}

/*************************************************
Function: UART_SendString(uint8_t *data) 
Description: String Send 
Input: String     
Output: NULL 
Return: NULL 
*************************************************/
void UART_SendString(uint8_t *data)
{
	uint8_t length = strlen( (const char *)data );
	HAL_UART_Transmit(&huart2,data,length,5);
}

/*************************************************
Function: convInt32ToText(int32_t value) 
Description: Hex converter
Input: Hexadecimal     
Output: NULL 
Return: String 
*************************************************/
uint8_t* convInt32ToText(int32_t value) 
{
    static char pValueToTextBuffer[12];
    char *pLast;
    char *pFirst;
    char last;
    uint8_t negative;

    pLast = pValueToTextBuffer;

    // Record the sign of the value
    negative = (value < 0);
    value = abs(value);

    // Print the value in the reverse order
    do {
        *(pLast++) = '0' + (uint8_t)(value % 10);
        value /= 10;
    } while (value);

    // Add the '-' when the number is negative, and terminate the string
    if (negative) *(pLast++) = '-';
    *(pLast--) = 0x00;

    // Now reverse the string
    pFirst = pValueToTextBuffer;
    while (pLast > pFirst) {
        last = *pLast;
        *(pLast--) = *pFirst;
        *(pFirst++) = last;
    }

    return (uint8_t *)pValueToTextBuffer;
}
 
/*************************************************
Function: Para_Init( uint8_t modulation )
Description: Radio Modulation Init
Input: Modulation Type     
Output: NULL 
Return: NULL 
*************************************************/
uint8_t g_vaCrcSeedLocal[3] = { 0x00, 0x45, 0x67 };
void Para_Init( uint8_t modulation )
{
	uint8_t m_aBuf[7];
	Radio.SetStandby( STDBY_RC );
 
	switch(modulation)
	{
		case PACKET_TYPE_GFSK:
					Radio.SetPacketType( PACKET_TYPE_GFSK );
					break;
		case PACKET_TYPE_LORA:
					Radio.SetPacketType( PACKET_TYPE_LORA );
					break;
		case PACKET_TYPE_FLRC:
					Radio.SetPacketType( PACKET_TYPE_FLRC );
					break;
		default:
					Radio.SetPacketType( PACKET_TYPE_LORA );
					break;
	}
	
	Radio.SetRfFrequency( 2450000000 );
	Radio.SetBufferBaseAddresses( 0x00, 0x00 );
	
	switch(modulation)
	{
		case PACKET_TYPE_GFSK:
				 /*********************GFSK BEGAIN******************************/
				  //	Radio.SetModulationParams( &ModulationParams );
					m_aBuf[0] = 0xEF;   //BR:0.125Mbps, BW:0.3MHz, DEV:62.5kHz
					m_aBuf[1] = 0x04;   //mod_ind:0.5
					m_aBuf[2] = 0x20;   //0.5
					Radio.WriteCommand( RADIO_SET_MODULATIONPARAMS, m_aBuf, 3 );
					
				  //	Radio.SetPacketParams( &PacketParams );
					m_aBuf[0] = 0x30;   //preamble length:16symbols
					m_aBuf[1] = 0x08;   //sync word length: 5 bytes
					m_aBuf[2] = 0x10;   //disable sync word combination
					m_aBuf[3] = 0x00;   //fixed length
					m_aBuf[4] = Payload_Length;   //payload length
					m_aBuf[5] = 0x20;   //crc Disable
					m_aBuf[6] = 0x08;   //whitening disable
					Radio.WriteCommand( RADIO_SET_PACKETPARAMS, m_aBuf, 7 );
					// only used in GFSK, FLRC (4 bytes max) and BLE mode
					Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );
					// only used in GFSK, FLRC 
					Radio.SetCrcSeed( g_vaCrcSeedLocal );
					Radio.SetCrcPolynomial( 0x0123 );
				 /*************************GFSK END*************************/	
					break;
		case PACKET_TYPE_LORA:
         /*********************LORA BEGAIN******************************/
				  //	Radio.SetModulationParams( &ModulationParams );
					m_aBuf[0] = 0xC0;   //SF:12(0xC0),BW:203(0x34)-(0.595);
                              //SF:9(0x90),BW:406(0x26)-(7.14);    
                              //SF:5(0x50),BW:1625(0x0A)-(253.91);        
					m_aBuf[1] = 0x34;   //BW:203
					m_aBuf[2] = 0x01;   //CR:4/5
					Radio.WriteCommand( RADIO_SET_MODULATIONPARAMS, m_aBuf, 3 );
					
					if((m_aBuf[0] == 0x50) || (m_aBuf[0] == 0X60))
					{
						Radio.WriteRegister(0x925, 0x1E);
					}
					else if((m_aBuf[0] == 0x70) || (m_aBuf[0] == 0X80))
					{
						Radio.WriteRegister(0x925, 0x37);
					}
					else
					{
						Radio.WriteRegister(0x925, 0x32);
					}
					
				  //	Radio.SetPacketParams( &PacketParams );
					m_aBuf[0] = 0x32;   //16symbols
					m_aBuf[1] = 0x80;   //Implicit Header --ÒþÊ½°üÍ·
					m_aBuf[2] = Payload_Length;   //payload length
					m_aBuf[3] = 0x20;   //CRC enable
					m_aBuf[4] = 0x40;   //I/Q swapped
					Radio.WriteCommand( RADIO_SET_PACKETPARAMS, m_aBuf, 5 );
         /*************************LORA END***************************/	
					break;
		case PACKET_TYPE_FLRC:
				 /*********************FLRC BEGAIN******************************/
					//	Radio.SetModulationParams( &ModulationParams );
					m_aBuf[0] = 0x45;   //BR:0.26Mbps, BW:0.3MHz
					m_aBuf[1] = 0x00;   //CR:1/2
					m_aBuf[2] = 0x10;   //Gaussian Filtering:0.5
					Radio.WriteCommand( RADIO_SET_MODULATIONPARAMS, m_aBuf, 3 );
					
				  //	Radio.SetPacketParams( &PacketParams );
					m_aBuf[0] = 0x70;   //AGCPreambleLength:24symbols
					m_aBuf[1] = 0x04;   //sync word length: 21bits + 4bytes sync word
					m_aBuf[2] = 0x10;   //disable sync word combination
					m_aBuf[3] = 0x00;   //fixed length
					m_aBuf[4] = Payload_Length;   //payload length
					m_aBuf[5] = 0x00;   //crc disable
					m_aBuf[6] = 0x08;   //whitening disable
					Radio.WriteCommand( RADIO_SET_PACKETPARAMS, m_aBuf, 7 );
					// only used in GFSK, FLRC (4 bytes max) and BLE mode
					Radio.SetSyncWord( 1, ( uint8_t[] ){ 0xDD, 0xA0, 0x96, 0x69, 0xDD } );
					// only used in GFSK, FLRC 
					Radio.SetCrcSeed( g_vaCrcSeedLocal );
					Radio.SetCrcPolynomial( 0x0123 );
				 /*************************FLRC END*************************/
					break;
		default:
 
					break;
	} 
	Radio.SetTxParams( 0x0D, RADIO_RAMP_20_US ); //power = 0x0D + 18;-->-18-13;0x0D means the MAX power 	
	Radio.SetRx( ( TickTime_t ) { RADIO_TICK_SIZE_1000_US, 0xFFFF } ); //continues reception
 
}

/*************************************************
Function: void Transceiver(uint8_t TRX) 
Description: Transceiver Applications
Input: Transceiver Type     
Output: NULL 
Return: NULL 
*************************************************/
DigitalOut TxLed(A5);
DigitalOut RxLed(PB_2);


uint8_t tx_rx_Buffer[Payload_Length]; 
uint8_t CorrValue = 0;
uint16_t recCount = 0;
uint16_t triCount = 0;
void Transceiver(uint8_t TRX) 
{
	if(TRX == 0x00)
	{
		triCount++;
		for( uint8_t i = 0; i < Payload_Length-1; i++ )
		{
				tx_rx_Buffer[i] = i+0x0F;
		}
		tx_rx_Buffer[Payload_Length-1] = triCount%10 + 0x30;
		
		Radio.SendPayload( tx_rx_Buffer, Payload_Length, ( TickTime_t ){ RADIO_TICK_SIZE_1000_US, 1000 } );
		UART_SendString((uint8_t *)"Transmitted: "); 
		HAL_UART_Transmit(&huart2,convInt32ToText(triCount),5,5);
		UART_SendString((uint8_t *)"\r\n");
		
		TxLed=!TxLed;
		HAL_Delay(1000);
	}
	else
	{
		g_vIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT; 
    Radio.SetDioIrqParams( g_vIrqMask, g_vIrqMask, IRQ_RADIO_NONE, IRQ_RADIO_NONE ); 
		while(Radio.GetIrqStatus() == 0x02)  //RX Done
		{
			Radio.ClearIrqStatus(g_vIrqMask);  
			Radio.GetPayload( tx_rx_Buffer, &BufferSize, Payload_Length );
			Radio.SetRx( ( TickTime_t ) { RADIO_TICK_SIZE_1000_US, 0xFFFF } );
      
		  for(uint8_t i=0;i<Payload_Length-1;i++)
			{
				if(tx_rx_Buffer[i] == (i + 0x0F))
				{
					CorrValue++;
				}
				else
				{
					CorrValue = 0;
				}
			}
				
			if(CorrValue == (Payload_Length-1))
			{				
				recCount++;
				UART_SendString((uint8_t *)"The rightmost number: \"");  
				HAL_UART_Transmit(&huart2,(uint8_t *) &tx_rx_Buffer[Payload_Length-1],1,5);
				UART_SendString((uint8_t *)"\"\r\n");  
				//UART_SendString((uint8_t *)"\" - ");
				//HAL_UART_Transmit(&huart2,convInt32ToText(recCount + 1),0,5);
				//UART_SendString((uint8_t *)" success\r\n");
				RxLed=!RxLed;
			}
			
			CorrValue = 0;
			
			memset(tx_rx_Buffer,0x00,Payload_Length);
	  }  
	}
}

