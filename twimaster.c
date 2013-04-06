/*************************************************************************
* Title:    I2C master library using hardware TWI interface
* Author:   Peter Fleury <pfleury@gmx.ch>  http://jump.to/fleury
* File:     $Id: twimaster.c,v 1.3 2005/07/02 11:14:21 Peter Exp $
* Software: AVR-GCC 3.4.3 / avr-libc 1.2.3
* Target:   any AVR device with hardware TWI 
* Usage:    API compatible with I2C Software Library i2cmaster.h
**************************************************************************/
#include <inttypes.h>
#include <compat/twi.h>
#include <avr/delay.h>
#include "i2cmaster.h"



/* define CPU frequency in Mhz here if not defined in Makefile */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

/* I2C clock in Hz */
#define SCL_CLOCK  10000L

uint16_t MAX_TRIES = 400;

#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms

#define STARTERR			0xA1
#define ADDRESSERR		0xA2
#define MAX_TRIESERR		0xA3
#define STOPERR			0xA4
#define WRITEERR			0xA5
#define READ_ACKERR		0xA6
#define READ_NACKERR		0xA6


#define OSZIPORT				PORTA
#define OSZIPORTDDR			DDRA
#define OSZIPORTPIN			PINA

#define PULSA					0

#ifndef OSZIALO
#define OSZIALO OSZIPORT &= ~(1<<PULSA)
#endif
#define OSZIAHI OSZIPORT |= (1<<PULSA)
#define OSZIATOG OSZIPORT ^= (1<<PULSA)

#define PULSB					1


#define OSZIBLO OSZIPORT &= ~(1<<PULSB)

#define OSZIBHI OSZIPORT |= (1<<PULSB)
#define OSZIBTOG OSZIPORT ^= (1<<PULSB)

#define TWI_PORT			PORTC
#define TWI_DDR			DDRC
#define TWI_PIN			PINC
#define SCL_PIN			PORTC0
#define SDA_PIN			PORTC1

extern void lcd_puts(const char *s);
extern void lcd_gotoxy(uint8_t x, uint8_t y);
extern void err_clr_line(uint8_t Linie);
extern void err_gotoxy(uint8_t x, uint8_t y);
extern void err_putc(const char c);
extern void err_puts(const char *s);
extern void delay_ms(uint16_t);
extern void err_puthex(uint8_t zahl);
extern void lcd_puthex(uint8_t zahl);
void err_clr_part(uint8_t Linie, uint8_t von, uint8_t bis);
uint8_t i2c_debloc(void);

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
	TWSR |= (1<<0);                         /* prescaler 4*/
	TWSR |= (1<<1);                         /* prescaler 16*/
	//	beide Bits: prescaler 64
	//  TWSR = 0;								/* no prescaler */
	TWBR = (uint8_t)((F_CPU/SCL_CLOCK)-16)/2;		/* must be > 10 for stable operation */
	//lcd_gotoxy(10,1);
   //lcd_puthex(TWBR);
   TWBR=96;
}/* i2c_init */


/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address)
{
//OSZIALO;
    uint8_t   twst;
	uint8_t counter=0;
	uint16_t whilecounter = WHILEMAX; // 5 ms
//	lcd_cls();
//	lcd_puts("i2c_start\0");
//	delay_ms(400);

	restart:
	if (counter++ >= MAX_TRIES)
	{	
		//delay_ms(10);
		err_clr_part(0,9,19);
		err_puts(" m tr\0");
		err_puthex(address);
		//delay_ms(200);
		TWI_Flag=MAX_TRIESERR;
		//OSZIAHI;
		return MAX_TRIESERR;
	}

	begin:

//  TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE)

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
//	lcd_cls();
//	lcd_puts("i2c_start1\0");
	
	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
	whilecounter--;
	}
	
	if (whilecounter==0) // kein Erfolg
	{
		TWI_Flag=STARTERR;
		//OSZIAHI;
		return STARTERR;
	}

	// check value of TWI Status Register. Mask prescaler bits.
	twst = TW_STATUS & 0xF8;
	if (twst == TW_MT_ARB_LOST)
	{
		goto begin;
	 }
	if ( (twst != TW_REP_START) && (twst != TW_START)) 
	{
		return 2;
	}

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);
	// wail until transmission completed and ACK/NACK has been received
	whilecounter=WHILEMAX; 
	while(!(TWCR & (1<<TWINT)) && whilecounter) 
	{
	whilecounter--;
	}
	
	if (whilecounter==0)	// Address-Fehler
	{
		TWI_Flag=ADDRESSERR;
		//OSZIAHI;
		return ADDRESSERR;
	}
	
//	lcd_puts("wh \0");

	twst = TW_STATUS & 0xF8;
	
	if (twst == TW_MT_SLA_NACK)
	{
		delay_ms(1);
		goto restart;
	 }
	if (twst == TW_MT_ARB_LOST) goto begin;
	if ( (twst != TW_MT_SLA_ACK ) && (twst != TW_MR_SLA_ACK)) goto error;

	

	return 0;

quit:
	// send stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	//OSZIAHI;
	return 0;

error:
	// send stop condition
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
	//OSZIAHI;
	return 3;
}/* i2c_start */


/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;


    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask prescaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	}
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     }

}/* i2c_start_wait */


/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address)
{
    return i2c_start( address );

}/* i2c_rep_start */


/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
	uint16_t whilecounter=WHILEMAX; // 5 ms
    /* send stop condition */
//	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
//	29.10.08
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO)|(1<<TWEA)|(1<<TWIE);
	// wait until stop condition is executed and bus released
	/*
	while((TWCR & (1<<TWSTO))&& whilecounter)
	{
	whilecounter--;
	}

	if (whilecounter==0)	// Address-Fehler
	{
	TWI_Flag=STOPERR;
	//return STOPERR;
	}
*/
}/* i2c_stop */


/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write( unsigned char data )
{	
    uint8_t   twst;
    uint16_t whilecounter=WHILEMAX; // 5 ms
	
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
	whilecounter--;
	}
	
	if (whilecounter==0)	// Address-Fehler
	{
		TWI_Flag=WRITEERR;
		return WRITEERR;
	}

	// check value of TWI Status Register. Mask prescaler bits
	twst = TW_STATUS & 0xF8;

	if( twst != TW_MT_DATA_ACK) return twst;
	return 0;

}/* i2c_write */


/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	uint16_t whilecounter=WHILEMAX; // 5 ms
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
		whilecounter--;
	 } 
	if (whilecounter==0)	// Read-Ack-Fehler
	{
		TWI_Flag=READ_ACKERR;
		
	}

    return TWDR;

}/* i2c_readAck */


/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	uint16_t whilecounter=WHILEMAX; // 5 ms
		//lcd_gotoxy(16,0);
		//lcd_puts("NACK\0");
		//delay_ms(100);

	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)) && whilecounter)
	{
		whilecounter--;
	}

	if (whilecounter==0)	// Read-Nack-Fehler
	{
		TWI_Flag=READ_ACKERR;
		
	}
	
    return TWDR;

}/* i2c_readNak */


uint8_t i2c_debloc(void)
{	
	TWCR &= ~(1<<TWEN); //disable TWI
	TWI_DDR &= ~(1<<SDA_PIN);	// SDA-Pin als EINgang
	TWI_PORT |= (1<<SDA_PIN);  // HI
	TWI_DDR &= ~(1<<SCL_PIN);	// SCL-Pin als EINgang
	TWI_PORT |= (1<<SCL_PIN);  // HI
	delay_ms(2);
	TWI_DDR |= (1<<SCL_PIN);	// SCL-Pin als Ausgang
	TWI_PORT |= (1<<SCL_PIN);  // HI
	delay_ms(2);
	uint8_t i=0x0F;
	while (i ) 
	{
		// SCL-Pulse schicken, um Slave zu deblockieren
		TWI_PORT &= ~(1<<SCL_PIN); // LO
		delay_ms(1);
		TWI_PORT |= (1<<SCL_PIN);	// HI
		delay_ms(10);
		i--;
		if (TWI_PIN & (1<<SDA_PIN)) // SDA ist wieder HI
		{

			return i;
		}
		
	}
	return i;
}
