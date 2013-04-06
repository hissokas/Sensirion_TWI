/*
 *  web_SPI.c
 *  WebServer
 *
 *  Created by Sysadmin on 11.Oktober.09.
 *  Copyright 2009 Ruedi Heimlicher. All rights reserved.
 *
 */

#include <avr/io.h>



// defines fuer Atmega328p

/*
#define SPI_CONTROL_DDR			DDRB
#define SPI_CONTROL_PORT		PORTB
#define SPI_CONTROL_CS_HC		PORTB0	//CS fuer HomeCentral Master

#define SPI_CONTROL_MOSI		PORTB3
#define SPI_CONTROL_MISO		PORTB4
#define SPI_CONTROL_SCK			PORTB5
*/

// neu 2.6.


#define OSZIPORT	PORTA
#define OSZIPORTDDR	DDRA
#define OSZIPORTPIN	PINA
#define PULS	3

#define OSZILO OSZIPORT &= ~(1<<PULS)
#define OSZIHI OSZIPORT |= (1<<PULS)
#define OSZITOGG OSZIPORT ^= (1<<PULS)

#define SPI_CONTROL_DDR			DDRD						// DDR fuer SPI
#define SPI_CONTROL_PORT		PORTD						// Port fuer SPI
#define SPI_CONTROL_PORTPIN	PIND						// Port-Pin fuer SPI

// ************************************************
// Modifizierte Belegung fuer Betrieb mit Webserver
// ************************************************

#define SPI_CONTROL_MOSI		PORTD0					// Eingang fuer Daten zum Slave
#define SPI_CONTROL_MISO		PORTD1					// Ausgang fuer Daten vom Slave
#define SPI_CONTROL_SCK			PORTD2					// Ausgang fuer CLK
#define SPI_CONTROL_CS_HC		PORTD3					// Ausgang CS fuer Slave

// ************************************************
// ************************************************


#define CS_HC_PASSIVE			SPI_CONTROL_PORT |= (1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist HI
#define CS_HC_ACTIVE				SPI_CONTROL_PORT &= ~(1<< SPI_CONTROL_CS_HC)	// CS fuer HC ist LO 

#define SPI_PULSE_DELAY			50								// Pause bei shift_byte

#define SPI_BUFSIZE				32								// Anzahl Bytes

// Ausgang:
volatile uint8_t					outbuffer[SPI_BUFSIZE];	// buffer fuer die Ausgangsdaten
volatile uint8_t					spi_startdaten;			// Startdaten fuer Ausgang
volatile uint8_t					spi_enddaten;				// Enddaten fuer Ausgang
volatile uint8_t					spi_hbdaten;
volatile uint8_t					spi_lbdaten;

// Eingang
volatile uint8_t					inbuffer[SPI_BUFSIZE];	// buffer fuer die Eingangsdaten
volatile uint8_t					in_startdaten;				// Startdaten fuer Eingang
volatile uint8_t					in_enddaten;				// Enddaten fuer Eingang
volatile uint8_t					in_hbdaten;
volatile uint8_t					in_lbdaten;

static volatile uint8_t			spistatus=0;				// Status der Uebertragung
static volatile uint8_t			ByteCounter=0;				// aktuelle Bytenummer


uint8_t SPI_shift_byte(uint8_t out_byte);

void Init_SPI_Master(void) 
{ 
	SPI_CONTROL_DDR |= ((1<<SPI_CONTROL_MOSI)|(1<<SPI_CONTROL_SCK)|(1<<SPI_CONTROL_CS_HC));	// Set MOSI , SCK , and SS output 
	SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);

	SPI_CONTROL_DDR &= ~(1<<SPI_CONTROL_MISO);																// MISO Eingang
	SPI_CONTROL_PORT |=(1<<SPI_CONTROL_MISO);																	// HI

	OSZIPORTDDR |=(1<<PULS);
	OSZIPORT |= (1<<PULS);

} 



uint8_t SPI_shift_byte(uint8_t out_byte)
{ 
	uint8_t in_byte=0;

	uint8_t i=0;
	for(i=0; i<8; i++)
	{
		if (out_byte & 0x80)
		{
			/* this bit is high */
			SPI_CONTROL_PORT |=_BV(SPI_CONTROL_MOSI); // MOSI HI
		}
		else
		{
			/* this bit is low */
			SPI_CONTROL_PORT &= ~_BV(SPI_CONTROL_MOSI); // MOSI LO						
		}
		_delay_us(2*SPI_PULSE_DELAY);
		
		SPI_CONTROL_PORT &=~(1<<SPI_CONTROL_SCK);				// Takt LO
		_delay_us(2*SPI_PULSE_DELAY);		
		
		// Slave lesen
		if (SPI_CONTROL_PORTPIN & (1<<SPI_CONTROL_MISO))	// Bit vom Slave ist HI
		{
			in_byte |= (1<<(7-i));
		}
		else
		{
			in_byte &= ~(1<<(7-i));
		}
		_delay_us(SPI_PULSE_DELAY);
		SPI_CONTROL_PORT |=(1<<SPI_CONTROL_SCK);				// Takt HI
		
		out_byte = out_byte << 1;									//	Byte um eine Stelle nach links schieben
		_delay_us(SPI_PULSE_DELAY);
	
	} // for i
	return in_byte;
}





