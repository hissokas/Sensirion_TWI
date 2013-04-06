/*websr
 *  websr.c
 *  TWI_Master
 *
 *  Created by Sysadmin on 10.02.08.
 *  Copyright 2008 Ruedi Heimlicher. All rights reserved.
 *
 */
 
#include <avr/io.h>
#include <util/delay.h>

#include "websr.h"

#define SR_DATA_IO_PORT			PORTD
#define SR_DATA_IO_PORTPIN		PIND
#define SR_DATA_IO_PORTDIR		DDRD

#define SR_DATA_PORT				PORTC // Clock und Data
#define SR_DATA_PORTPIN			PINC // Data lesen
#define SR_DATA_PORTDIR			DDRC// Data Richtung


#define SR_DATA_IO_PIN			1

#define SR_HANDSHAKE_PORT		PORTD // Talk, Listen
#define SR_HANDSHAKE_PORTPIN	PINC  // Listen lesen
#define SR_HANDSHAKE_PORTDIR	DDRC  // Listen lesen

#define SR_TALK_PIN				2
#define SR_LISTEN_PIN			3


// nur Parallel
/*
// Atmega8
#define SR_S0_PIN				4
#define SR_S1_PIN				5
#define SR_OE_PIN				6 // auf PORTC
#define SR_CLK_PIN			7
*/

// Atmega 32
#define SR_CLK_PIN			4	// auf PORTC
#define SR_S0_PIN				5
#define SR_S1_PIN				6
#define SR_OE_PIN				7 


#define SR_PULSE_DELAY			250

#define SR_LISTEN_BIT			7
#define SR_TALK_BIT				6
#define SR_ERR_BIT				5 // 
#define DATENBREITE				8

#define DATA_RECEIVE_BIT		3
#define SEND_REQUEST_BIT		2

#define SEND_ERR_BIT				4
#define SEND_WAIT_BIT			1
#define SEND_SERIE_BIT			0

#define TIMER2_WERT				0xDF
#define MAX_TRIES					10
static volatile uint8_t websrstatus=0;
static volatile uint8_t ByteCounter=0;

void websrdelay_ms(unsigned int ms)
/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}


void websr_timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt
	//TCCR2 |= (1<<CS21);	//Takt /8	Intervall 8 us max= 2 ms

	TCCR2A |= (1<<CS20)|(1<<CS21)|(1<<CS22);	//Takt /1024	Intervall 32 us

	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR2 |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2A = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 


/*
 * Sets Pins
 *
*/
void websr_init(void)
{
	// Talk als Ausgang
	SR_HANDSHAKE_PORTDIR |=(1<<SR_TALK_PIN);
	SR_HANDSHAKE_PORT |=(1<<SR_TALK_PIN); // Hi
	
	// Listen als Eingang
	SR_HANDSHAKE_PORTDIR &= ~(1<<SR_LISTEN_PIN);
	SR_HANDSHAKE_PORT |=(1<<SR_LISTEN_PIN); // Hi
	
	SR_HANDSHAKE_PORTDIR |=(1<<SR_S0_PIN);// SR_S0_PIN als Ausgang
	SR_HANDSHAKE_PORT &= ~(1<<SR_S0_PIN); // LO
	
	SR_HANDSHAKE_PORTDIR |=(1<<SR_S1_PIN);// SR_S1_PIN als Ausgang
	SR_HANDSHAKE_PORT &= ~(1<<SR_S1_PIN); // LO
	
	SR_HANDSHAKE_PORTDIR |=(1<<SR_OE_PIN);// SR_OE_PIN als Ausgang
	SR_HANDSHAKE_PORT |= (1<<SR_OE_PIN); // HI
	
	// HANDSHAKE_PORT
	
	// PIN 3 von PORTC als Ausgang: Ausgabe eines Request oder confirm
	SR_HANDSHAKE_PORTDIR |= (1<<SR_TALK_PIN); // Ausgang
	SR_HANDSHAKE_PORT |= (1<<SR_TALK_PIN); // HI
	
	// PIN 4 von PORTC als Eingang: Hoeren auf Partner
	SR_HANDSHAKE_PORTDIR &= ~(1<<SR_LISTEN_PIN);
	SR_HANDSHAKE_PORT |= (1<<SR_LISTEN_PIN); // HI
	
	// DATA_PORT	
	// PIN 2 von PORTD als Ausgang: Ausgabe auf Clock
	SR_DATA_PORTDIR |=(1<<SR_CLK_PIN);
	SR_DATA_PORT |=(1<<SR_CLK_PIN);// HI
	
	//PORTD als Ausgang
	SR_DATA_IO_PORTDIR = 0xFF;
	SR_DATA_IO_PORT = 0xFF;

	
	
	ByteCounter=0;
	
}


/*
 * Sets the mode 
 *
*/
void websr_set_mode(uint8_t derMode)
{
switch (derMode)
	{
		case 0: //Out
		{
			//PORTD als Ausgang
			SR_DATA_IO_PORTDIR = 0xFF;
			SR_DATA_IO_PORT = 0xFF;
		}break;
			
		case 1: // In
		{
			//PORTD als Eingang
			SR_DATA_IO_PORTDIR = 0x00;
			SR_DATA_IO_PORT = 0xFF;
			
		}
			
	}
	
	
}

/*
 * Pulse the shift clock
 *
*/
void websr_pulse(uint8_t delay)
{
		_delay_us(delay);
		SR_DATA_PORT &= ~_BV(SR_CLK_PIN);
		_delay_us(delay);
		/* pulse the shift clock up */
		SR_DATA_PORT |= _BV(SR_CLK_PIN);
		
		
		
}

void websr_pulse_ms(uint8_t delay)
{
		_delay_ms(delay);
		SR_DATA_PORT &= ~_BV(SR_CLK_PIN);
		_delay_ms(delay);
		/* pulse the shift clock up */
		SR_DATA_PORT |= _BV(SR_CLK_PIN);
		
		
		
}

void websr_OE_pulse(uint8_t delay)
{
	// active LO
		//_delay_us(delay);
		SR_HANDSHAKE_PORT &= ~_BV(SR_OE_PIN);
		//_delay_us(delay);
		/* pulse the OE clock up */
		SR_HANDSHAKE_PORT |= _BV(SR_OE_PIN);
		
		
		
}



/*
 * Request fuer Hoeren


*/

uint8_t ready_for_send()
{
	if (websrstatus & (1<<DATA_RECEIVE_BIT)) // hoeren ist im Gang
	{
		return 0;
	}
	
	/*
	else	if ((websrstatus & (1<< SEND_REQUEST_BIT)))// kein Listen,  sendrequest da
	{
		//websrstatus &= ~(1<< SEND_REQUEST_BIT); // Bit zuruecksetzen
		
		
		
		
		return 1;
	}
	*/
	return 0;
}


/*
 * Sets the mode of the shift register (74'299)
 *
*/
void set_shift_mode(uint8_t derMode)
{

	switch (derMode)	
	{
		case 0:	//hold
			SR_HANDSHAKE_PORT &=~(1<<SR_S0_PIN);
			SR_HANDSHAKE_PORT &=~(1<<SR_S1_PIN);
			break;
		case 1: //shift right
			SR_HANDSHAKE_PORT &=~(1<<SR_S1_PIN);
			SR_HANDSHAKE_PORT |=(1<<SR_S0_PIN);
			break;
		case 2: //shift left
			SR_HANDSHAKE_PORT &=~(1<<SR_S0_PIN);
			SR_HANDSHAKE_PORT |=(1<<SR_S1_PIN);
			break;
		case 3: //load
			SR_HANDSHAKE_PORT |=(1<<SR_S0_PIN);
			SR_HANDSHAKE_PORT |=(1<<SR_S1_PIN);
			break;
	
	}
	
}


/*
 * shifts a byte into the SR*
 * Parameters:
 *      out_byte        The byte to load .
*/
uint8_t  websr_shift_byte_out(uint8_t out_byte)
{ 
	// Listen-Bit zuruecksetzen
	//	websrstatus &= ~(1<< SR_LISTEN_BIT); // Bit 7
	
	
	// Data auf Port laden
	SR_DATA_IO_PORT = out_byte;
	
	// Data in SR laden
	set_shift_mode(3);
	
	//websr_pulse(SR_PULSE_DELAY);
	websr_pulse(2);
	//websr_pulse_ms(10);

	
	// SR auf shift-left einstellen
	set_shift_mode(2);
	
	
	// Talk-Request an Partner senden, warten auf Antwort
	SR_HANDSHAKE_PORT &= ~(1<<SR_TALK_PIN);	// Bit 3
	
	// Talk-Bit setzen
	websrstatus |= (1<< SR_TALK_BIT); // Bit 6
	
	//_delay_ms(1);
	
	//lcd_gotoxy(2,1);
	//lcd_puts("A\0");
	
	uint16_t zz=0;
	uint16_t z=1;
	z=1;
	/*
	 if ((SR_HANDSHAKE_PORTPIN & (1<< SR_LISTEN_PIN)))
	 {
	 lcd_gotoxy(0,1);
	 lcd_putc('H');
	 }
	 else
	 {
	 lcd_gotoxy(0,1);
	 lcd_putc('L');
	 }
	 */
	
	//websrstatus |=(1<< SEND_ERR_BIT);
	while ((SR_HANDSHAKE_PORTPIN & (1<< SR_LISTEN_PIN)) )//&& (!(websrstatus &(1<< SEND_ERR_BIT)))) // noch keine Antwort vom Partner > warten
	{
		// Timer2 zuruecksetzen
		TCNT2=0; 
		_delay_us(100); // warten
		z++;
		if (z == 0)
		{
			zz++;
			if (zz >= 0)
			{
			break;
			}
		}
		
	}
	//if (ByteCounter == 0)
	
//	z &= 0xFF00;
//	z >>= 8;
	//zz &= 0xFF00;
	//zz >>= 8;
	//lcd_gotoxy(13,1);
	//lcd_puthex(ByteCounter);
	if (ByteCounter == 0xFF)
	{
		//lcd_gotoxy(16,1);
		//lcd_puthex(z);
		//lcd_puthex(zz);
		//lcd_gotoxy(17,0);
		if (zz >= 0x000F)
		{
			lcd_gotoxy(17,0);
			lcd_putc('E');
			websrstatus |= (1<< SR_ERR_BIT); 
			websr_reset();
			return 2;
		}
		else
		{
		//	lcd_putc('G');
		}
		
	}
	
	//_delay_ms(4);
	
	websrstatus &= ~(1<< SEND_ERR_BIT);
	websr_timer2(250);
	
	TCNT2 = 0;
	if (websrstatus &(1<< SEND_ERR_BIT))
	{
		lcd_gotoxy(4,1);
		lcd_puts("N\0");
		//_delay_ms(100);
		websrstatus &= ~(1<< SEND_ERR_BIT);
		return 1;
	}
	
	//lcd_gotoxy(3,1);
	//lcd_putc('+');
	
	//	if (websrstatus & (1<< SEND_REQUEST_BIT))// Send-Request ist noch gesetzt, Timer nicht abgelaufen
	{
		//	lcd_gotoxy(0,0);
		
		//lcd_puthex(z & (0xF0 >> 4));
		//z=0xA2C4;
		//lcd_puthex(z);
		//	z &= 0xFF00;
		//	z >>=8;
		//	lcd_puthex(z);
		
		// Antwort da
//		lcd_gotoxy(0,1);
//		lcd_puts("Y\0");
		//_delay_ms(200); // Eventuell mehr
		//websrstatus |= (1<< SR_LISTEN_BIT); // Bit 7
		//	lcd_gotoxy(10,0);
		//	lcd_putc('*');
		
	} // if websrstatus & (1<< SEND_REQUEST_BIT)	//
	
	z=0;
	// Auf ruecksetzen von Listen durchg Partner warten
	
	while ((!(SR_HANDSHAKE_PORTPIN & (1<< SR_LISTEN_PIN)))) // Partner liest noch, Listen-Pin ist noch LO > warten
	{	
		//lcd_gotoxy(16,1);
		//lcd_putint1(z);
		_delay_us(20); // warten
		z++;
	}
	//lcd_gotoxy(18,0);
	//lcd_putc('H');
	TCNT2=0;
	_delay_ms(1);
	// Talk-Request zuruecknehmen 
	SR_HANDSHAKE_PORT |= (1<<SR_TALK_PIN);
	
	// Dem Partner Zeit lassen
	_delay_ms(2);
	return 0;
}



// Listen-PIN  pollen, warten auf Request: PIN wird LO
uint8_t  ListenForRequest()
{
	if (websrstatus & (1<< SR_TALK_BIT))
	{
	 // ich sende selber
	 return 0;
	 
	}
	return 0;
	// Signal fuer Oszi
	//		PORTC |= (1<<PC3);
	//		_delay_ms(2);
	//		PORTC &= ~(1<<PC3);

	
	if ((SR_HANDSHAKE_PORTPIN & (1<<SR_LISTEN_PIN))) // PIN ist (wieder) HI
	{
		if (websrstatus &(1<<SR_LISTEN_BIT)) // Listen-Bit ist noch gesetzt, Partner hat gesendet
		{
			// SR in register laden
			//set_shift_mode(3);
			//websr_pulse_ms(4);
			// Listen-Bereitschaft an Partner zuruecknehmen
			SR_HANDSHAKE_PORT |= (1<<SR_TALK_PIN);
			
			// Listen-Bit zuruecksetzen
			websrstatus &= ~(1<<SR_LISTEN_BIT);
			
			websr_set_mode(0);
			
		}
		else
		{
		// Talk-Bit zuruecksetzen
		
//		 websrstatus &= ~(1<< SR_TALK_BIT); // Bit 6

//		 websrstatus &= ~(1<<SR_LISTEN_BIT);
		}
		
	}
	else // SR_LISTEN_PIN ist LO, Request vom Partner
	{
		// *******************
		// Talk-Bit zuruecksetzen
		 websrstatus &= ~(1<< SR_TALK_BIT); // Bit 6
		// *******************
		
		
		// Ist LISTEN-BIT im websrstatus schon gesetzt?
		if (websrstatus & (1<<SR_LISTEN_BIT)) // Bit 7
		{
			// Senden ist eingeleitet, nichts tun
			
		}
		else // neuer Request, Listen-Bit setzen
		{
			websr_set_mode(1); // PORTD als Eingang
			set_shift_mode(1); // shift right
			if (ByteCounter == 0xFF)
			{
				uint8_t i=0;
				for (i=0;i< 0 ;i++)
				{
				_delay_ms(20);
				_delay_ms(20);
				_delay_ms(20);
				_delay_ms(20);
				_delay_ms(20);
				}

			}

			// Listen-Bereitschaft an Partner senden
			SR_HANDSHAKE_PORT &= ~(1<<SR_TALK_PIN);
			// Listen-Status-Bit setzen
			websrstatus |= (1<<SR_LISTEN_BIT);
			//_delay_ms(4);
			return 1; // Bereit zum Lesen
		}
		
		
		
	}
	return 0;
	
}

/*
 * shift a byte from the SR into the processor.  
 *
 * Parameters:
 *      in_byte        The byte to shift in from the '299.
*/ 
uint8_t websr_shift_byte_in()
{
	// Request vom Partner testen, warten auf Antwort
	// Listen-Bit zuruecksetzen
	// SR in register laden
	
	//websr_pulse_ms(4);
	uint8_t inbyte=0;
	//Warten, bis senden fertig
	while (!(SR_HANDSHAKE_PORTPIN & (1<<SR_LISTEN_PIN)))
	{
		;
	}
	//_delay_ms(1);
	
	//websr_set_mode(1); // in ListenForRequest
	// SR hold
	set_shift_mode(0);
	// Clk an SR
	//websr_pulse(1);
	// Register ausgeben
	//_delay_ms(1);
	SR_HANDSHAKE_PORT &= ~_BV(SR_OE_PIN);
	_delay_us(50);

	inbyte = SR_DATA_IO_PORTPIN;
	//_delay_ms(1);
	SR_HANDSHAKE_PORT |= _BV(SR_OE_PIN);
	/*
	 set_shift_mode(0);
	 // PORTD wieder als Ausgang
	 //websr_set_mode(0);
	 
	 //	lcd_clr_line(1);
	 
	 
	 // Listen-Bit zuruecksetzen
	 //	websrstatus &= ~(1<<SR_LISTEN_BIT);						// nur noch in ListenForRequest
	 //_delay_ms(10);
	 
	 // Listen-Bereitschaft an Partner zuruecknehmen
	 //	SR_HANDSHAKE_PORT |= (1<<SR_TALK_PIN);					// nur noch in ListenForRequest
	 */
	 	 //lcd_gotoxy(16,0);
	 //lcd_puthex(inbyte);

	return inbyte ;
}

void websr_reset()
{
	// Aufraeumen, zu lange warten auf Partner
	websrstatus &= ~(1<< SEND_SERIE_BIT); // Bit 0
	websrstatus &= ~(1<< SR_ERR_BIT); // Bit 0
	websrstatus &= ~(1<< DATA_RECEIVE_BIT);
	websrstatus &= ~(1<< SEND_WAIT_BIT);
	websrstatus &= ~(1<< SR_LISTEN_BIT);
	websrstatus &= ~(1<< SEND_REQUEST_BIT); // Send-Request zuruecksetzen
	websrstatus &= ~(1<< SEND_ERR_BIT); 
	// TALK zuruecknehmen
	SR_HANDSHAKE_PORT |= (1<<SR_TALK_PIN);
	set_shift_mode(0);
	
	// Timer2 stop
	TCCR2B=0;

}
