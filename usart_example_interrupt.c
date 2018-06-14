

#include "usart_driver.h"
#include "avr_compiler.h"

/*! Number of bytes to send in test example. */
#define NUM_BYTES  3
/*! Define that selects the Usart used in example. */
#define USART USARTC0

/*! USART data struct used in example. */
USART_data_t USART_data;
/*! Test data to send. */
uint8_t sendArray[NUM_BYTES] = {'a','b','c'};
/*! Array to put received data in. */
uint8_t receiveArray[NUM_BYTES];
/*! Success variable, used to test driver. */
bool success;


/*! \brief Example application.
 *
 *  Example application. This example configures USARTC0 for with the parameters:
 *      - 8 bit character size
 *      - No parity
 *      - 1 stop bit
 *      - 9600 Baud
 *
 *  This function then sends three bytes and tests if the received data is
 *  equal to the sent data. The code can be tested by connecting PC3 to PC2. If
 *  the variable 'success' is true at the end of the function, the three bytes
 *  have been successfully sent and received.
*/
int main(void)
{
	/* counter variable. */
	uint8_t i;

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PC3 (TXD0) as output. */
	PORTC.DIRSET   = PIN3_bm;
	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock frequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	/* Enable global interrupts. */
	sei();

	/* Send sendArray. */
	i = 0;
	while (i < NUM_BYTES) {
		bool byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data, sendArray[i]);                    //Used to send the Sendarray defined earlier
		if(byteToBuffer){
			i++;
		}
	}
	while(1){                                                                               //for interrupt
		
	}

}


/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTC0_RXC_vect)
{                                                                                                //triggered when Data is received by Atxmega.
	USART_RXComplete(&USART_data);
	int i=0;
	
		if (USART_RXBufferData_Available(&USART_data)) {
			receiveArray[i] = USART_RXBuffer_GetByte(&USART_data);
			
		}
		bool byteToBuffer;
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data,receiveArray[i] );
		
		
	}
	

	
			


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTC0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}
