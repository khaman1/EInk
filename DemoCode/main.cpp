#include "mbed.h"
#include "DemoApplication.h" 
 
/*************************************************
Function: main( void )
Description: main program 
Input: NULL     
Output: NULL 
Return: NULL 
*************************************************/
DigitalOut BTLed(A4);
//Serial pc(SERIAL_TX, SERIAL_RX);

//Serial pc(D8, D2);

int main( void )
{    	
		BTLed = 1;
	
		UART_Init( ); 
	  UART_SendString((uint8_t *)FIRMWARE_VERSION);
	
    Radio_Init( ); 
		Para_Init(0x01); // 0x00:PACKET_TYPE_GFSK
										 // 0x01:PACKET_TYPE_LORA
										 // 0x03:PACKET_TYPE_FLRC
    while( 1 )
    {
			  Transceiver(0x01); // 0x00:transmit			
													 // 0x01:receive		
				//pc.printf("Hello World !\n");
    }
}
