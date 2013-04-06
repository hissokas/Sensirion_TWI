/*
 *  slaves.c
 *  TWI_Master
 *
 *  Created by Sysadmin on 16.03.08.
 *  Copyright 2008 Ruedi Heimlicher. All rights reserved.
 *
 */

#include "slaves.h"
#include <avr/io.h>

#define OSZIAPORT				PORTA
#define OSZIAPORTDDR			DDRA
#define OSZIAPORTPIN			PINA
#define PULSA					0

#ifndef OSZIALO
#define OSZIALO OSZIAPORT &= ~(1<<PULSA)
#endif
#ifndef OSZIAHI
#define OSZIAHI OSZIAPORT |= (1<<PULSA)
#endif
#ifndef OSZIATOG
#define OSZIATOG OSZIAPORT ^= (1<<PULSA)
#endif

#define buffer_size 8 
#define tag_data_size 32
#define tag_start_adresse 0

#define TAGPLANBREITE		0x40	// 64 Bytes, 2 page im EEPROM
#define RAUMPLANBREITE		0x200	// 512 Bytes

extern volatile uint8_t rxbuffer[buffer_size];
extern volatile uint8_t txbuffer[buffer_size];
//extern char* wochentagstring[];

extern unsigned char WOCHENPLANBREITE;

uint8_t SlaveSchreiben(unsigned char ADRESSE)
{
	/* Slave schreiben */
	uint8_t writeerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	
	if(writeerfolg==0) //Slave bereit zum schreiben?
	{
	//	lcd_cls();
	//	lcd_puts("Write GO\0 ");
	//	delay_ms(2000);
		
		writeerfolg=i2c_write(0x00); //Buffer Startadresse zum Schreiben
		if (writeerfolg==0)
		{
			/*
			err_cls();
			err_puts("write Startadresse OK\0 ");
			delay_ms(800);
			*/
			writeerfolg=i2c_rep_start(ADRESSE+I2C_WRITE); //Schreiben beginnen
			if (writeerfolg==0)
			{
				/*
				err_cls();
				err_puts("rep write starten \0 ");
				delay_ms(800);
				*/
				// lcd_cls();
				// lcd_puts("Byte 0: \0 ");
				// lcd_putint(Kesseldata);
				// delay_ms(1800);
				uint8_t i=0;
				while ((writeerfolg==0)&&(i<8))
				{
					/*
					err_gotoxy(0,1);
					err_puts("Byte \0 ");
					err_putint1(i);
					*/
					//writeerfolg=i2c_write(Daten[i]);
					writeerfolg=i2c_write(txbuffer[i]);
					/*
					err_gotoxy(0,1);
					err_puts("E:\0 ");
					err_putint1(writeerfolg);
					delay_ms(800);
					*/
					
					if (i==7)
					{
						/*
						err_gotoxy(10,1);
						err_puts("Write 8B OK\0");
						*/
					}
					
					i++;
				}//while 
				if (i<7)//Fehler
				{
					//err_clr_line(1);
					//err_gotoxy(0,1);
					//err_puts("W SL \0 ");
					//err_puthex(ADRESSE);					
					//err_puts(" Err Bt \0 ");
					//err_putint1(i);
				}
				i2c_stop();
			}	//	RepStart
			else
			{
				
				//err_clr_line(1);
				//err_gotoxy(0,1);
				//err_puts("Err RpStrt \0 ");
				//err_putint1(writeerfolg);
				i2c_stop();
			}
			
		}	//	Buffer Startadresse zum Schreiben
		else
		{
			//err_clr_line(1);
			//err_gotoxy(0,1);
			//err_puts("Err bf adr \0 ");
			//err_putint1(writeerfolg);

			i2c_stop();
		}
		
	}//if writeerfolg
	else
	{
		//err_clr_line(1);
		//err_gotoxy(0,1);
		//err_puts("Slave \0 ");
		//err_puthex(ADRESSE);
		//err_puts(" nicht OK\0 ");
		//delay_ms(80);
//		lcd_clr_line(3);
	}
	
	return writeerfolg;	
	
}// Ende SlaveSchreiben

uint8_t SlavedatenSchreiben(const unsigned char ADRESSE, const uint8_t *Daten)
{
	/* Slave schreiben */
	{

		//err_clr_line(1);
		//err_puthex(ADRESSE);
		}
	//	err_putc(' ');
	uint8_t writeerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	//err_putc('1');
	//err_puthex(writeerfolg);
	if(writeerfolg==0) //Slave bereit zum schreiben?
	{
		//lcd_cls();
		//lcd_puts("Write GO\0 ");
		//delay_ms(2000);
		
		writeerfolg=i2c_write(0x00); //webflag ist 0, nur 8 Bytes
		//err_putc('2');
		//err_puthex(writeerfolg);

		if (writeerfolg==0)
		{
			/*
			err_cls();
			err_puts("write Startadresse OK\0 ");
			delay_ms(800);
			*/
			writeerfolg=i2c_rep_start(ADRESSE+I2C_WRITE); //Schreiben beginnen
			//err_putc('3');
			//err_puthex(writeerfolg);

			if (writeerfolg==0)
			{
				/*
				err_cls();
				err_puts("rep write starten \0 ");
				delay_ms(800);
				*/
				// lcd_cls();
				// lcd_puts("Byte 0: \0 ");
				// lcd_putint(Kesseldata);
				// delay_ms(1800);
				uint8_t i=0;
				//err_putc('4');

				while ((writeerfolg==0)&&(i<8))
				{
					/*
					err_gotoxy(0,1);
					err_puts("Byte \0 ");
					err_putint1(i);
					*/
					//writeerfolg=i2c_write(Daten[i]);

					writeerfolg=i2c_write(Daten[i]);

					//err_putint1(i);
					//err_putc(' ');
					//err_puthex(writeerfolg);
					//delay_ms(400);
					//err_dec_cursor(4);

					/*
					err_gotoxy(0,1);
					err_puts("E:\0 ");
					err_putint1(writeerfolg);
					delay_ms(800);
					*/
					
					if (i==7)
					{
						
						//err_gotoxy(16,0);
						//err_puts("W OK\0");
						//delay_ms(1000);
					}
					
					i++;
				}//while 
				if (i<7)//Fehler
				{
					//err_clr_part(1,9,19);
					//err_puthex(ADRESSE);
					//err_puts(" w bt er\0");
					//err_putint1(i);
					//delay_ms(10);
				}
				i2c_stop();
			}	//	RepStart
			else
			{
				
				//err_clr_part(1,9,19);
				//err_puthex(ADRESSE);
				//err_puts(" w rp er\0");
				i2c_stop();
			}
			
		}	//	Buffer Startadresse zum Schreiben
		else
		{
			//err_clr_part(1,9,19);
			//err_puthex(ADRESSE);
			//err_puts(" w bf er\0");
			i2c_stop();
		}
		
	}//if writeerfolg
	else
	{
			//err_clr_part(1,9,19);
			//err_puthex(ADRESSE);
			//err_puts(" r str er\0");
			//delay_ms(10);
//		lcd_clr_line(3);
	}
				
//	err_puthex(writeerfolg);
//	delay_ms(100);
//	err_clr_line(1);
	return writeerfolg;	
	
}

uint8_t SlaveLesen(unsigned char ADRESSE)
{
	
	uint8_t readerfolg=0;
	readerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (readerfolg==0)
	{
		
		/*
		 err_cls();
		 err_puts("Start Read OK\0 ");
		 delay_ms(800);
		 */
		readerfolg=i2c_write(0x00); //Buffer Startadresse zum Auslesen
		if (readerfolg==0)
		{
			//lcd_clr_line(1);
			/*
			 lcd_cls();
			 lcd_gotoxy(0,3);
			 lcd_puts("read Startadresse OK\0 ");
			 delay_ms(800);
			 */
		}
		else
		{
			i2c_stop();
			//err_clr_line(0);
			//err_gotoxy(0,0);
			//err_puts("SlaveLesen\0 ");
			//err_clr_line(1);
			//err_gotoxy(0,1);
			//err_puts("i2c_write: Kein Erfolg\0");
		}
		
		readerfolg=i2c_rep_start(ADRESSE+I2C_READ); //Lesen beginnen
		if (readerfolg==0)
		{
			/*
			 lcd_cls();
			 lcd_gotoxy(0,2);
			 lcd_puts("rep_start OK\0 ");
			 delay_ms(800);
			 */
		}
		else
		{
			i2c_stop();
			//err_clr_line(0);
			//err_gotoxy(0,0);
			//err_puts("SlaveLesen\0 ");
			
			
			
			//err_puts("rep_start: Kein Erfolg\0");
		}
		
		
		uint8_t i=0;
		while (i<7)
		{
			/*
			 lcd_gotoxy(0,3);
			 lcd_puts("while i: \0");
			 lcd_putint1(i);
			 */
			//Daten[i]=i2c_readAck();
			rxbuffer[i]=i2c_readAck();
			//lcd_puts(" read OK\0");
			i++;
		}
		//lcd_gotoxy(0,3);
		//lcd_puts("Byt 7: \0");
		
		//Daten[7]=i2c_readNak();//letztes Byte lesen, darum kein ACK
		rxbuffer[7]=i2c_readNak();//letztes Byte lesen, darum kein ACK
		
		i2c_stop();
		/*
		 lcd_clr_line(3);
		 lcd_gotoxy(0,3);
		 lcd_puts("R: 8 Bytes lesen OK\0");
		 */
	} 
	else
	{
		//i2c_stop();
		//err_clr_line(0);
		//err_gotoxy(0,0);
		//err_puts("SlaveLesen\0 ");
		
		//err_clr_line(1);
		//err_gotoxy(10,0);
		//err_puts("R Err \0");
		//err_puthex(ADRESSE);
		//err_puthex(readerfolg);
		//delay_ms(800);
	}
	
	
	return readerfolg;
}

uint8_t SlavedatenLesen(const unsigned char ADRESSE, uint8_t *Daten)
{
	
	uint8_t readerfolg=0;
	readerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (readerfolg==0)
	{
		/*
		 err_cls();
		 err_puts("Start Read OK\0 ");
		 delay_ms(800);
		 */
		delay_ms(1);
		readerfolg=i2c_write(0x00); //Buffer Startadresse zum Auslesen
		if (readerfolg==0)
		{
			//lcd_clr_line(1);
			/*
			 lcd_cls();
			 lcd_gotoxy(0,3);
			 lcd_puts("read Startadresse OK\0 ");
			 delay_ms(800);
			 */
		}
		else
		{
			i2c_stop();
			//err_clr_part(1,9,19);
			//err_puthex(ADRESSE);
			//err_puts(" r ad er\0");
			//delay_ms(10);
			return readerfolg;
			
		}
		
		delay_ms(1);
		readerfolg=i2c_rep_start(ADRESSE+I2C_READ); //Lesen beginnen
		if (readerfolg==0)
		{
			/*
			 lcd_cls();
			 lcd_gotoxy(0,2);
			 lcd_puts("rep_start OK\0 ");
			 delay_ms(800);
			 */
		}
		else
		{
			i2c_stop();
			//err_clr_part(1,9,19);
			//err_puthex(ADRESSE);
			//err_puts(" r rp er\0");
			//delay_ms(10);
			return readerfolg;
		}
		
		delay_ms(1);
		uint8_t i=0;
		while (i<7)
		{
			/*
			 lcd_gotoxy(0,3);
			 lcd_puts("while i: \0");
			 lcd_putint1(i);
			 */
			//Daten[i]=i2c_readAck();
			Daten[i]=i2c_readAck();
			_delay_us(10);
			//lcd_puts(" read OK\0");
			i++;
		}
		//lcd_gotoxy(0,3);
		//lcd_puts("Byt 7: \0");
		
		//Daten[7]=i2c_readNak();//letztes Byte lesen, darum kein ACK
		Daten[7]=i2c_readNak();//letztes Byte lesen, darum kein ACK
		delay_ms(1);
		i2c_stop();
		
		//err_clr_line(0);
		//err_puthex(ADRESSE);
		//err_puts("R OK\0");
		//delay_ms(200);
		//err_clr_part(0,0,10);
		
	} 
	else
	{
		//i2c_stop();
		
			//err_clr_part(1,9,19);
			//err_puthex(ADRESSE);
			//err_puts(" r strt er\0");
			//delay_ms(10);
	}
	
	
	return readerfolg;
}


void WochentagZeigen(const uint8_t Daten[], uint8_t Tag, uint8_t Stunde)
{
	char* wochentag[] = {"MO","DI","MI","DO","FR","SA","SO"};
	uint8_t line=2;
	lcd_clr_line(2);
	lcd_gotoxy(0,2);
	lcd_puts(wochentag[Tag]);
/*	
Daten: 8 Bytes, 4 Stunden pro Byte
Bit 7, 6: Stunde 0
Bit 5, 4: Stunde 1
...
Zwei Stunden werden mit einem Zeichen dargestellt.

*/	
	uint8_t StundenCode=0;
	uint8_t i=0;
	for (i=0;i<12;i++) // 12 Anzeigefelder
	{
		StundenCode=0x00;
		/*
		lcd_clr_line(0);
		delay_ms(100);
				
		lcd_gotoxy(0,0);
		lcd_putint(i);
		lcd_gotoxy(4,0);
		lcd_puthex(Daten[2*i]);
		*/
		uint8_t StundenCodeA=0x00;
		uint8_t StundenCodeB=0x00;
		if (i%2) //	ungerades Anzeigefeld, Bits 3 - 0
		{
		StundenCodeA=(Daten[i/2]>>2);	//	erster Balken im char, bit 3, 2
		StundenCodeB=(Daten[i/2]>>0);	//	zweiter Balken im char, bits 1, 0
		
		}
		else	//	gerades Anzeigefeld, Bits 7 - 4
		{
		StundenCodeA=(Daten[i/2]>>6);	//	erster Balken im char, bit 7, 6
		StundenCodeB=(Daten[i/2]>>4);	//	zweiter Balken im char, bits 5, 4
		}
		// Bits 0, 1:
		StundenCodeA &= 0x03;
		StundenCodeB &= 0x03;

		
		
		/*
		lcd_gotoxy(7,0);
		lcd_puthex(StundenCodeA);
		
		//delay_ms(2000);
		*/
		switch (StundenCodeA)
		{
			case 0: //	ganze Stunde OFF
				break;
			case 2:	//	erste halbe Stunde ON
				StundenCode |=(1<<3);// viertes Bit	0000 1000	08
				break;
			case 1:	//	zweite halbe Stunde ON	
				StundenCode |=(1<<2);// drittes Bit	0000 0100	04
				break;		
			case 3:	//	ganze Stunde ON
				StundenCode |=(1<<3);// viertes Bit
				StundenCode |=(1<<2);// drittes Bit	0000 1100	0C
				break;		
				
		}//switch i	//	erster Balken im char
		/*
		lcd_gotoxy(11,0);
		lcd_puthex(Daten[2*i+1]);
		*/
		
				/*
		lcd_gotoxy(14,0);
		lcd_puthex(StundenCodeB);
		//delay_ms(2000);
		*/
		switch (StundenCodeB)
		{
			case 0:
				StundenCode |=(0<<1);// erstes Bit
				StundenCode |=(0<<0);// zweites Bit	0000 0000	00
				break;		
			
			case 2:	//erste halbe Stunde ON
				StundenCode |=(1<<1);// erstes Bit	0000 0010	02
				break;
			case 1:	//zweite halbe Stunde ON
				StundenCode |=(1<<0);// zweites Bit	0000 0001	01
				break;		
			case 3:	//	ganze Stunde ON
				StundenCode |=(1<<1);// erstes Bit
				StundenCode |=(1<<0);// zweites Bit	0000 0011	03
				break;		
		}//switch i	//	zweiter Balken im char
		
		
		
		//err_gotoxy(2*i,1);
		//err_puts("cd:\0");
		//err_puthex(StundenCode);
		//delay_ms(1);
		
		
		//CGRAM laden
		
		
		
		lcd_gotoxy(i+7,line);
		switch (StundenCode)
		{
			case 0://off											__ __
				lcd_putc(8);
				break;
			case 1://	B	zweite halbe Stunde						__ _|		1
				lcd_putc(7);
				break;
			case 3://	B	ganze Stunde							__ ||		3
				lcd_putc(6);
				break;
			case 7://	A erste halbe Stunde und B ganz				_| ||		7
				lcd_putc(5);
				break;
			case 15://	A und B ganz								|| ||		F
				lcd_putc(4);
				break;
			case 14://	A ganze Stunde, B erste halbe Stunde		|| |_		E
				lcd_putc(3);
				break;
			case 12://	A ganze Stunde								|| __		C
				lcd_putc(2);
				break;
			case 8://	A erste halbe Stunde						|_ __		8
				lcd_putc(1);
				break;
			
			// CGRAM neu laden
			
	/*		
			case 2: //B erste halbe Stunde							__ |_		2
				lcd_putc(3);
				break;			
			
			case 4://	A zweite halbe Stunde						_| __		4
				lcd_putc(1);
				break;
				
			case 5: //A, B zweite halbe Stunde						_| _|		5
				lcd_putc(7);
				break;

			case 6: //A zweite halbe Stunde, B erste halbe Stunde	_| |_		6
				lcd_putc(2);
				break;
			
			case 9: //A erste halbe Stunde B erste halbe Stunde		|_ _|		9
				lcd_putc(6);
				break;

			case 10: //B erste halbe Stunde							|_ |_		2
				lcd_putc(8);
				break;
			
			case 11: //	A erste halbe Stunde, B ganz				|_ ||		11
				lcd_putc(4);	
				break;
			
			case 13: //	A ganze Stunde, B zweite halbe Stunde		|| _|		13
				lcd_putc(5);	
				break;
									
	*/		
				
					
			default:
				lcd_putc(8);
				break;
				
		}//switch
		//delay_ms(1000);
		//delay_ms(1000);
	}
	
	lcd_gotoxy(7+ (Stunde/2),line+1);
	//lcd_putc(94);
	lcd_putc(34);
	/*
	if (Stunde%2)
	{
	lcd_putc(6);
	}
	else
	{
	lcd_putc(2);
	}
	*/
	
	
}//WochentagZeigen

uint8_t Tagplanwert(const uint8_t Daten[], const uint8_t Stunde)
{
/*	
Daten: 8 Bytes, 4 Stunden pro Byte
Bit 7, 6: Stunde 0
Bit 5, 4: Stunde 1
...
*/
uint8_t wert=Daten[Stunde/4]; // Byte in Daten
//err_puts(" w0");
//err_puthex(wert);
uint8_t i=Stunde%4;		// Position im Byte
//err_putint2(i);
wert=wert>>(2*(3-i));	// Stundenpaket um (3-i) Stellen nach links schieben
wert &= 0x03;
//err_puthex(wert);
return wert;
}

void TagZeigen(uint8_t Daten[], uint8_t Tag)
{
	char* wochentag[] = {"MO","DI","MI","DO","FR","SA","SO"};
	uint8_t line=2;
	lcd_cls();
	lcd_gotoxy(0,line);
	lcd_puts(wochentag[Tag]);
//	
	
	uint8_t StundenCode=0;
	uint8_t i=0;
	for (i=0;i<12;i++)
	{
		StundenCode=0x00;
		/*
		lcd_clr_line(0);
		delay_ms(100);
				
		lcd_gotoxy(0,0);
		lcd_putint(i);
		lcd_gotoxy(4,0);
		lcd_puthex(Daten[2*i]);
		*/
		
		uint8_t StundenCodeA=(Daten[2*i]>>6);	//	erster Balken im char, bit 7, 6
		/*
		lcd_gotoxy(7,0);
		lcd_puthex(StundenCodeA);
		
		//delay_ms(2000);
		*/
		switch (StundenCodeA)
		{
			case 0:
				StundenCode |=(0<<3);// viertes Bit
				StundenCode |=(0<<2);// drittes Bit	0000 0000	00
				break;
			case 2:	//	erste halbe Stunde ON
				StundenCode |=(1<<3);// viertes Bit	0000 1000	08
				break;
			case 1:	//	zweite halbe Stunde ON	
				StundenCode |=(1<<2);// drittes Bit	0000 0100	04
				break;		
			case 3:	//	ganze Stunde ON
				StundenCode |=(1<<3);// viertes Bit
				StundenCode |=(1<<2);// drittes Bit	0000 1100	0C
				break;		
				
		}//switch i	//	erster Balken im char
		/*
		lcd_gotoxy(11,0);
		lcd_puthex(Daten[2*i+1]);
		*/
		
		uint8_t StundenCodeB=(Daten[2*i+1]>>6);	//	zweiter Balken im char
		/*
		lcd_gotoxy(14,0);
		lcd_puthex(StundenCodeB);
		//delay_ms(2000);
		*/
		switch (StundenCodeB)
		{
			case 0:
				StundenCode |=(0<<1);// erstes Bit
				StundenCode |=(0<<0);// zweites Bit	0000 0000	00
				break;		
			
			case 2:	//erste halbe Stunde ON
				StundenCode |=(1<<1);// erstes Bit	0000 0010	02
				break;
			case 1:	//zweite halbe Stunde ON
				StundenCode |=(1<<0);// zweites Bit	0000 0001	01
				break;		
			case 3:	//	ganze Stunde ON
				StundenCode |=(1<<1);// erstes Bit
				StundenCode |=(1<<0);// zweites Bit	0000 0011	03
				break;		
		}//switch i	//	zweiter Balken im char
		
		
		//uint8_t	charBalkenCode= 0;
		/*
		lcd_gotoxy(18,0);
		lcd_puthex(StundenCode);
		//delay_ms(2000);
		*/
		
		//CGRAM laden
		
		
		
		lcd_gotoxy(i+7,line);
		switch (StundenCode)
		{
			case 0://off											__ __
				lcd_putc(8);
				break;
			case 1://	B	zweite halbe Stunde						__ _|		1
				lcd_putc(7);
				break;
			case 3://	B	ganze Stunde							__ ||		3
				lcd_putc(6);
				break;
			case 7://	A erste halbe Stunde und B ganz				_| ||		7
				lcd_putc(5);
				break;
			case 15://	A und B ganz								|| ||		F
				lcd_putc(4);
				break;
			case 14://	A ganze Stunde, B erste halbe Stunde		|| |_		E
				lcd_putc(3);
				break;
			case 12://	A ganze Stunde								|| __		C
				lcd_putc(2);
				break;
			case 8://	A erste halbe Stunde						|_ __		8
				lcd_putc(1);
				break;
			
			// CGRAM neu laden
			
	/*		
			case 2: //B erste halbe Stunde							__ |_		2
				lcd_putc(3);
				break;			
			
			case 4://	A zweite halbe Stunde						_| __		4
				lcd_putc(1);
				break;
				
			case 5: //A, B zweite halbe Stunde						_| _|		5
				lcd_putc(7);
				break;

			case 6: //A zweite halbe Stunde, B erste halbe Stunde	_| |_		6
				lcd_putc(2);
				break;
			
			case 9: //A erste halbe Stunde B erste halbe Stunde		|_ _|		9
				lcd_putc(6);
				break;

			case 10: //B erste halbe Stunde							|_ |_		2
				lcd_putc(8);
				break;
			
			case 11: //	A erste halbe Stunde, B ganz				|_ ||		11
				lcd_putc(4);	
				break;
			
			case 13: //	A ganze Stunde, B zweite halbe Stunde		|| _|		13
				lcd_putc(5);	
				break;
									
	*/		
				
					
			default:
				lcd_putc(8);
				break;
				
		}//switch
		lcd_gotoxy(i+7,line+1);
		lcd_putc(94);

		//delay_ms(1000);
	}
}




uint8_t TagSchreiben(unsigned char ADRESSE, uint8_t Daten[], uint8_t Tag)
{
	uint8_t eepromerfolg=0;
	//lcd_gotoxy(0,0);
	//	lcd_cls();
	//	lcd_puts("TagSchreiben GO\0 ");
	//delay_ms(3000);
	
	/*
	uint8_t i;
	for (i=0;i<24;i++)
	{
	lcd_gotoxy(0,1);
	delay_ms(10);
	lcd_putint(i);
	lcd_gotoxy(5,1);
	delay_ms(10);
	lcd_putint(Daten[i]);
	delay_ms(800);
	}
	*/
	
	eepromerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (eepromerfolg==0)
	{
		//lcd_gotoxy(0,1);
		//lcd_puts("TS Start OK\0 ");
		//delay_ms(3000);
		
		uint16_t ee_adresse=tag_start_adresse+Tag*tag_data_size;
		
		eepromerfolg= i2c_write(ee_adresse>>8);	// high bits
		eepromerfolg += i2c_write(ee_adresse);	//low bits
		if (eepromerfolg==0)
		{
			//lcd_gotoxy(0,1);
			//lcd_puts("TS Startadr OK\0 ");
			//delay_ms(1000);
			
			uint8_t i=0;
			
			while ((eepromerfolg==0)&&(i<buffer_size))
			{
				/*
				 lcd_gotoxy(0,1);
				 lcd_puts("Byte \0 ");
				 lcd_putint(i);
				 */
				eepromerfolg=i2c_write(Daten[i]);
				/*
				 lcd_puts(" E: \0 ");
				 lcd_putint(writeerfolg);
				 delay_ms(1800);
				 */
				 /*
				if (i==buffer_size-1)
				{
					lcd_gotoxy(0,1);
					lcd_puts("TS: \0");
					lcd_putint(i);
					lcd_puts(" Bytes OK    \0 ");
					delay_ms(2000);
				}
				*/
				i++;
			}//while 
			if (i<buffer_size-1)//Fehler
			{
				//err_gotoxy(0,1);
				//err_puts("Err Byte: \0 ");
				//err_putint(i);
			}
			i2c_stop();
			
			
		}// if startadresse
		else
		{
			// Startadresse failed
			//err_gotoxy(0,1);
			//err_puts("TS Startadresse failed \0 ");
			//delay_ms(10);
			i2c_stop();
			return eepromerfolg;
		}
		
	}// if start
	else
	{
		//Start failed
		//err_gotoxy(0,0);
		//err_puts("TagSchreiben\0");
		//err_gotoxy(0,1);
		//err_puts("Start failed \0 ");
		//delay_ms(10);

		return eepromerfolg;
	}
	/*
		lcd_cls();
		lcd_gotoxy(0,0);
		lcd_puts("TS  END Tag:  \0");
		lcd_putint(Tag);
		delay_ms(1000);
	*/
	
	return eepromerfolg;
}

uint8_t EEPROMTagLesen(unsigned char ADRESSE, uint8_t *Daten, uint8_t hb,uint8_t lb)
{
	
	uint8_t eepromerfolg=0;
	/*
	lcd_cls();
	lcd_puts("W.TagLesen GO \0");
	*/
	eepromerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (eepromerfolg==0)
	{
		/*
		lcd_gotoxy(0,1);
		lcd_puts("EE Start OK     \0");
		delay_ms(2000);
		*/
		
		/*
		lcd_cls();
		lcd_puts("TagLesen\0");
		lcd_gotoxy(0,1);
		lcd_puts("A0: \0");
		lcd_putint(ee_adresse);
		*/
		eepromerfolg= i2c_write(hb);	// high bits
		eepromerfolg += i2c_write(lb);	//low bits
		
		/*
		lcd_puts(" A1: \0");
		lcd_putint(ee_adresse<<8);
		delay_ms(1000);
		*/
//		eepromerfolg= i2c_write(tag_start_adresse+Tag*tag_data_size);//low bits
//		eepromerfolg= i2c_write(0x00); //high bits
		if (eepromerfolg==0)
		{
			/*e
			lcd_gotoxy(0,1);
			lcd_puts("Startadr OK \0");
			delay_ms(2000);
			*/
			
		eepromerfolg=(i2c_start(ADRESSE+I2C_READ));		//rep start
		if (eepromerfolg==0)
		{
			/*
			lcd_gotoxy(0,1);
			lcd_puts("EE rep Start OK\0");
			delay_ms(2000);
			*/

			uint8_t i=0;
			for (i=0;i<buffer_size;i++)
			{
				if (i<buffer_size-1)
				{
					Daten[i]=i2c_readAck();
					
					/*
					lcd_gotoxy(0,1);
					lcd_puts("i: \0");
					lcd_putint(i);
					lcd_puts(" Data \0");
					lcd_putint(Daten[i]);
					delay_ms(400);
					*/
				}
				else
				{
					Daten[i]=i2c_readNak();//letztes Byte lesen, darum kein ACK
					/*
					lcd_clr_line(1);
					lcd_gotoxy(0,1);
					lcd_puts("i: \0");
					lcd_putint(i);
					lcd_puts(" NAK\0");
					delay_ms(400);
					*/
				
				}
			}
			
			}//rep start
			
			i2c_stop();
			
		}
		else
		{
			//Startadresse failed
			i2c_stop();
			//err_clr_line(0);
			//err_gotoxy(0,0);
			//err_puts("TagLesen\0");
			//err_clr_line(1);
			//err_gotoxy(0,1);
			//err_puts("Startadr failed\0");
			//delay_ms(10);
			return eepromerfolg;
		}
		
	}
	else
	{
		//Start failed
		//err_clr_line(0);
		//err_gotoxy(0,0);
		//err_puts("TagLesen\0");
		//err_clr_line(1);
		//err_gotoxy(0,1);
		//err_puts("Start failed\0");
		//delay_ms(10);
		return eepromerfolg;
	}
	//	err_clr_line(0);
	//	err_gotoxy(0,0);
	//	err_puts("TagLesen OK\0");
	return eepromerfolg;
}

uint8_t EEPROMTagSchreiben(unsigned char ADRESSE, uint8_t * Daten, uint8_t hb, uint8_t lb )
{
	uint8_t eepromerfolg=0;
	//lcd_gotoxy(0,0);
//	lcd_cls();
//	lcd_puts("TagSchreiben GO\0 ");
	//delay_ms(3000);
	
	eepromerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (eepromerfolg==0)
	{
		//lcd_gotoxy(0,1);
		//lcd_puts("TS Start OK\0 ");
		//delay_ms(3000);
		
		
		eepromerfolg= i2c_write(hb);	// twiflag
		if (eepromerfolg==0)
		{
			//lcd_gotoxy(0,1);
			//lcd_puts("TS Startadr OK\0 ");
			//delay_ms(1000);
			
			eepromerfolg= i2c_write(lb);
			uint8_t i=0;
			if (eepromerfolg==0)
			{
			
			
			while ((eepromerfolg==0)&&(i<buffer_size))
			{
				/*
				 lcd_gotoxy(0,1);
				 lcd_puts("Byte \0 ");
				 lcd_putint(i);
				 */
				eepromerfolg=i2c_write(Daten[i]);
				/*
				 lcd_puts(" E: \0 ");
				 lcd_putint(writeerfolg);
				 delay_ms(1800);
				 */
				 /*
				if (i==buffer_size-1)
				{
					lcd_gotoxy(0,1);
					lcd_puts("TS: \0");
					lcd_putint(i);
					lcd_puts(" Bytes OK    \0 ");
					delay_ms(2000);
				}
				*/
				i++;
			}//while 
			
			}// twicode
			
			if (i<buffer_size-1)//Fehler
			{
				err_gotoxy(0,1);
				err_puts("Err Byte: \0 ");
				err_putint(i);
			}
			i2c_stop();
			
			
		}// if startadresse
		else
		{
			// Startadresse failed
			//err_gotoxy(0,1);
			//err_puts("TS Startadresse failed \0 ");
			//delay_ms(10);
			i2c_stop();
			return eepromerfolg;
		}
		
	}// if start
	else
	{
		//Start failed
		//err_gotoxy(0,0);
		//err_puts("TagSchreiben\0");
		//err_gotoxy(0,1);
		//err_puts("Start failed \0 ");
		//delay_ms(10);

		return eepromerfolg;
	}
	/*
		lcd_cls();
		lcd_gotoxy(0,0);
		lcd_puts("TS  END Tag:  \0");
		lcd_putint(Tag);
		delay_ms(1000);
	*/
	
	return eepromerfolg;
}// WebTagSchreiben

uint8_t WebTagSchreiben(unsigned char ADRESSE, uint8_t Daten[], uint8_t flags)
{
	uint8_t eepromerfolg=0;
	//lcd_gotoxy(0,0);
//	lcd_cls();
//	lcd_puts("TagSchreiben GO\0 ");
	//delay_ms(3000);
	
	/*
	uint8_t i;
	for (i=0;i<24;i++)
	{
	lcd_gotoxy(0,1);
	delay_ms(10);
	lcd_putint(i);
	lcd_gotoxy(5,1);
	delay_ms(10);
	lcd_putint(Daten[i]);
	delay_ms(800);
	}
	*/
	eepromerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (eepromerfolg==0)
	{
		//lcd_gotoxy(0,1);
		//lcd_puts("TS Start OK\0 ");
		//delay_ms(3000);
		
		
		eepromerfolg= i2c_write(flags);	// twiflag
		if (eepromerfolg==0)
		{
			//lcd_gotoxy(0,1);
			//lcd_puts("TS Startadr OK\0 ");
			//delay_ms(1000);
			
			eepromerfolg= i2c_write(0xC1);
			uint8_t i=0;
			if (eepromerfolg==0)
			{
			
			
			while ((eepromerfolg==0)&&(i<buffer_size))
			{
				/*
				 lcd_gotoxy(0,1);
				 lcd_puts("Byte \0 ");
				 lcd_putint(i);
				 */
				eepromerfolg=i2c_write(Daten[i]);
				/*
				 lcd_puts(" E: \0 ");
				 lcd_putint(writeerfolg);
				 delay_ms(1800);
				 */
				 /*
				if (i==buffer_size-1)
				{
					lcd_gotoxy(0,1);
					lcd_puts("TS: \0");
					lcd_putint(i);
					lcd_puts(" Bytes OK    \0 ");
					delay_ms(2000);
				}
				*/
				i++;
			}//while 
			
			}// twicode
			
			if (i<buffer_size-1)//Fehler
			{
				//err_gotoxy(0,1);
				//err_puts("Err Byte: \0 ");
				//err_putint(i);
			}
			i2c_stop();
			
			
		}// if startadresse
		else
		{
			// Startadresse failed
			//err_gotoxy(0,1);
			//err_puts("TS Startadresse failed \0 ");
			//delay_ms(10);
			i2c_stop();
			return eepromerfolg;
		}
		
	}// if start
	else
	{
		//Start failed
		//err_gotoxy(0,0);
		//err_puts("TagSchreiben\0");
		//err_gotoxy(0,1);
		//err_puts("Start failed \0 ");
		//delay_ms(10);

		return eepromerfolg;
	}
	/*
		lcd_cls();
		lcd_gotoxy(0,0);
		lcd_puts("TS  END Tag:  \0");
		lcd_putint(Tag);
		delay_ms(1000);
	*/
	
	return eepromerfolg;
}// WebTagSchreiben


uint8_t WochentagLesen(unsigned char ADRESSE, uint8_t *Daten, uint8_t Raum, uint8_t Objekt, uint8_t Wochentag)
{
	
	uint8_t eepromerfolg=0;
	/*
	lcd_cls();
	lcd_puts("W.TagLesen GO \0");
	*/
	eepromerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (eepromerfolg==0)
	{
		/*
		lcd_gotoxy(0,1);
		lcd_puts("EE Start OK     \0");
		delay_ms(2000);
		*/
		
		uint16_t ee_adresse=Raum*RAUMPLANBREITE + Objekt*TAGPLANBREITE+ Wochentag*0x08;
		/*
		lcd_cls();
		lcd_puts("TagLesen\0");
		lcd_gotoxy(0,1);
		lcd_puts("A0: \0");
		lcd_putint(ee_adresse);
		*/
		eepromerfolg= i2c_write(ee_adresse>>8);	// high bits
		eepromerfolg += i2c_write(ee_adresse);	//low bits
		
		/*
		lcd_puts(" A1: \0");
		lcd_putint(ee_adresse<<8);
		delay_ms(1000);
		*/
//		eepromerfolg= i2c_write(tag_start_adresse+Tag*tag_data_size);//low bits
//		eepromerfolg= i2c_write(0x00); //high bits
		if (eepromerfolg==0)
		{
			/*e
			lcd_gotoxy(0,1);
			lcd_puts("Startadr OK \0");
			delay_ms(2000);
			*/
			
		eepromerfolg=(i2c_start(ADRESSE+I2C_READ));		//rep start
		if (eepromerfolg==0)
		{
			/*
			lcd_gotoxy(0,1);
			lcd_puts("EE rep Start OK\0");
			delay_ms(2000);
			*/
			
			
				
			
			uint8_t i=0;
			for (i=0;i<buffer_size;i++)
			{
				if (i<buffer_size-1)
				{
					Daten[i]=i2c_readAck();
					
					/*
					lcd_gotoxy(0,1);
					lcd_puts("i: \0");
					lcd_putint(i);
					lcd_puts(" Data \0");
					lcd_putint(Daten[i]);
					delay_ms(400);
					*/
				}
				else
				{
					Daten[i]=i2c_readNak();//letztes Byte lesen, darum kein ACK
					/*
					lcd_clr_line(1);
					lcd_gotoxy(0,1);
					lcd_puts("i: \0");
					lcd_putint(i);
					lcd_puts(" NAK\0");
					delay_ms(400);
					*/
				
				}
			}
			
			}//rep start
			
			i2c_stop();
			
		}
		else
		{
			//Startadresse failed
			i2c_stop();
			//err_clr_line(0);
			//err_gotoxy(0,0);
			//err_puts("TagLesen\0");
			//err_clr_line(1);
			//err_gotoxy(0,1);
			//err_puts("Startadr failed\0");
			//delay_ms(10);
			return eepromerfolg;
		}
		
	}
	else
	{
		//Start failed
		//err_clr_line(0);
		//err_gotoxy(0,0);
		//err_puts("TagLesen\0");
		//err_clr_line(1);
		//err_gotoxy(0,1);
		//err_puts("Start failed\0");
		//delay_ms(10);
		return eepromerfolg;
	}
	//	err_clr_line(0);
	//	err_gotoxy(0,0);
	//	err_puts("TagLesen OK\0");
	return eepromerfolg;
}

uint8_t TagLesen(unsigned char ADRESSE, uint8_t *Daten, uint8_t Raum, uint8_t Tag)
{
	
	uint8_t eepromerfolg=0;
	/*
	lcd_cls();
	lcd_puts("TagLesen GO \0");
	*/
	eepromerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (eepromerfolg==0)
	{
		/*
		lcd_gotoxy(0,1);
		lcd_puts("EE Start OK     \0");
		delay_ms(2000);
		*/
		
		uint16_t ee_adresse=tag_start_adresse+Tag*tag_data_size;
		/*
		lcd_cls();
		lcd_puts("TagLesen\0");
		lcd_gotoxy(0,1);
		lcd_puts("A0: \0");
		lcd_putint(ee_adresse);
		*/
		eepromerfolg= i2c_write(ee_adresse>>8);	// high bits
		eepromerfolg += i2c_write(ee_adresse);	//low bits
		
		/*
		lcd_puts(" A1: \0");
		lcd_putint(ee_adresse<<8);
		delay_ms(1000);
		*/
//		eepromerfolg= i2c_write(tag_start_adresse+Tag*tag_data_size);//low bits
//		eepromerfolg= i2c_write(0x00); //high bits
		if (eepromerfolg==0)
		{
			/*e
			lcd_gotoxy(0,1);
			lcd_puts("Startadr OK \0");
			delay_ms(2000);
			*/
			
		eepromerfolg=(i2c_start(ADRESSE+I2C_READ));		//rep start
		if (eepromerfolg==0)
		{
			/*
			lcd_gotoxy(0,1);
			lcd_puts("EE rep Start OK\0");
			delay_ms(2000);
			*/
			
			
				
			
			uint8_t i=0;
			for (i=0;i<buffer_size;i++)
			{
				if (i<buffer_size-1)
				{
					Daten[i]=i2c_readAck();
					/*
					lcd_gotoxy(0,1);
					lcd_puts("i: \0");
					lcd_putint(i);
					lcd_puts(" Data \0");
					lcd_putint(Daten[i]);
					delay_ms(400);
					*/
				}
				else
				{
					Daten[i]=i2c_readNak();//letztes Byte lesen, darum kein ACK
					/*
					lcd_clr_line(1);
					lcd_gotoxy(0,1);
					lcd_puts("i: \0");
					lcd_putint(i);
					lcd_puts(" NAK\0");
					delay_ms(400);
					*/
				
				}
			}
			
			}//rep start
			
			i2c_stop();
			
		}
		else
		{
			//Startadresse failed
			i2c_stop();
			//err_clr_line(0);
			//err_gotoxy(0,0);
			//err_puts("TagLesen\0");
			//err_clr_line(1);
			
			//err_puts("Startadr failed\0");
			//delay_ms(10);
			return eepromerfolg;
		}
		
	}
	else
	{
		//Start failed
		//err_clr_line(0);
		//err_gotoxy(0,0);
		//err_puts("TagLesen\0");
		//err_clr_line(1);
		//err_gotoxy(0,1);
		//err_puts("Start failed\0");
		//delay_ms(10);
		return eepromerfolg;
	}
	//	err_clr_line(0);
	//	err_gotoxy(0,0);
	//	err_puts("TagLesen OK\0");
	return eepromerfolg;
}







