/*
 *  sensor.c
 *  
 *
 *  Created by Sysadmin on 27.03.13.
 *  Copyright 2013 Ruedi Heimlicher. All rights reserved.
 *
 */

#include "sensor.h"
#include <avr/io.h>

#define WHILEMAX 0xFFFF // Wartezeit in while-Schleife : 5 ms


uint8_t SensorTemperaturLesen(const unsigned char ADRESSE, uint8_t *Daten)
{
	uint8_t readerfolg=0;
   uint16_t whilecounter = WHILEMAX; // 5 ms
	readerfolg=(i2c_start(ADRESSE+I2C_WRITE));
	if (readerfolg==0)
	{
		/*
		 err_cls();
		 err_puts("Start Read OK\0 ");
		 delay_ms(800);
		 */
		delay_ms(100);
		readerfolg=i2c_write(TRIG_T_MEASUREMENT_HM); //Buffer Startadresse zum Auslesen
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
		
		delay_ms(100);
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
		
		delay_ms(100);
      
      Daten[0]=i2c_readAck();
      delay_ms(100);
      Daten[1]=i2c_readAck();
      delay_ms(100);
		Daten[2] =i2c_readNak();
		delay_ms(100);
		i2c_stop();
		
		//err_clr_line(0);
		//err_puthex(ADRESSE);
		//err_puts("R OK\0");
		//delay_ms(200);
		//err_clr_part(0,0,10);
		
	}
	else
	{
		i2c_stop();
		
      //err_clr_part(1,9,19);
      //err_puthex(ADRESSE);
      //err_puts(" r strt er\0");
      //delay_ms(10);
	}
	
	
	return readerfolg;
}

uint8_t SensorHumidityLesen(const unsigned char ADRESSE, uint8_t *Daten)
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
		delay_ms(100);
		readerfolg=i2c_write(TRIG_RH_MEASUREMENT_HM); //Buffer Startadresse zum Auslesen
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
		
		delay_ms(100);
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
		
		delay_ms(100);
      
      Daten[0]=i2c_readAck();
      delay_ms(100);
      Daten[1]=i2c_readAck();
      delay_ms(100);
		Daten[2] =i2c_readNak();
		delay_ms(100);
		i2c_stop();
		
		//err_clr_line(0);
		//err_puthex(ADRESSE);
		//err_puts("R OK\0");
		//delay_ms(200);
		//err_clr_part(0,0,10);
		
	}
	else
	{
		i2c_stop();
		
      //err_clr_part(1,9,19);
      //err_puthex(ADRESSE);
      //err_puts(" r strt er\0");
      //delay_ms(10);
	}
	
	
	return readerfolg;
}