/*
    File:       err.c
    Version:    0.1.0
    Date:       Feb. 25, 2006
    
    C header file for HD44780 LCD module using a 74HCT164 serial in, parallel 
    out out shift register to operate the LCD in 8-bit mode.  Example schematic
    usuing a Powertip PC-1202A LCD available at http://www.micahcarrick.com
    
    Uses the avr-libc library for gcc (avr-gcc).
    
    ****************************************************************************
    Copyright (C) 2006 Micah Carrick   <email@micahcarrick.com>
	

    ****************************************************************************
*/

#include "err.h"
#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>

/*
void lcddelay_ms(unsigned int ms)// delay for a minimum of <ms> 
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

*/

void 
err_backlight(int backlight_on)
{
        if (backlight_on)
        {
                /* LCD Backlight on */
                ERR_PORT |= _BV(ERR_BACKLIGHT_PIN);                
        } else {
                /*  LCD Backlight off */
                ERR_PORT &= ~_BV(ERR_BACKLIGHT_PIN);
        }
}


void
err_initialize(uint8_t set_function, uint8_t set_entry_mode, uint8_t on)
{
        /* 20ms delay while LCD powers on */
        _delay_ms(30);	   
        
        /* Write 0x30 to LCD and wait 5 mS for the instruction to complete */
        err_load_byte(0x30);
        err_send_cmd();
        _delay_ms(10);
        
        /* Write 0x30 to LCD and wait 160 uS for instruction to complete */
        err_load_byte(0x30);
        err_send_cmd();
        _delay_us(160);
        
        /* Write 0x30 AGAIN to LCD and wait 160 uS */
        err_load_byte(0x30);
        err_send_cmd();
        _delay_us(160);
        
        /* Set function and wait 40uS */
        err_load_byte(set_function);
        err_send_cmd();
        
        /* Turn off the display and wait 40uS */
        err_load_byte(LCD_CMD_OFF);    
        err_send_cmd();
        
        /* Clear display and wait 1.64mS */
        err_load_byte(LCD_CMD_CLEAR);    
        err_send_cmd();
        _delay_ms(4);
        
        /* Set entry mode and wait 40uS */
        err_load_byte(set_entry_mode);    
        err_send_cmd();
        _delay_ms(4);
        /* Turn display back on and wait 40uS */
        err_load_byte(on);    
        err_send_cmd();
		_delay_ms(40);
};

/*
 * Loads a byte into the shift register (74'164).  Does NOT load into the LCD.
 *
 * Parameters:
 *      out_byte        The byte to load into the '164.
*/
void 
err_load_byte(uint8_t out_byte)
{
        /* make sure clock is low */
        ERR_PORT &= ~_BV(ERR_CLOCK_PIN);
        
        int i;
        for(i=0; i<8; i++)
        {
                /* loop through bits */
                
                if (out_byte & 0x80)
                {
                        /* this bit is high */
                        ERR_PORT |=_BV(ERR_RSDS_PIN); 
                }
                else
                {
                        /* this bit is low */
                        ERR_PORT &= ~_BV(ERR_RSDS_PIN);
                }
                out_byte = out_byte << 1;
                
                /* pulse the the shift register clock */
				
                ERR_PORT |= _BV(ERR_CLOCK_PIN);
                ERR_PORT &= ~_BV(ERR_CLOCK_PIN);
        }
}

/*
 * Loads the byte in the '164 shift register into the LCD as a command. The
 * '164 should already be loaded with the data using err_load_byte().
*/
void
err_send_cmd(void)
{

        /* Data in '164 is a command, so RS must be low (0) */
        ERR_PORT &= ~_BV(ERR_RSDS_PIN); 
        err_strobe_E();
        _delay_us(10);
}

/*
 * Loads the byte in the '164 shift register into the ERR_ as a character. The
 * '164 should already be loaded with the data using err_load_byte().
*/
void
err_send_char(void)
{

        /* Data in '164 is a character, so RS must be high (1) */
        ERR_PORT |= _BV(ERR_RSDS_PIN); 
        err_strobe_E();
        _delay_us(10);
}

/*
 * Loads the byte into the shift register and then sends it to the ERR_ as a char
 * Parameters:
 *      c               The byte (character) to display
*/
void 
err_putc(const char c)
{
        err_load_byte(c);
        err_send_char();
}

/*************************************************************************
Display string without auto linefeed 
Input:    string to be displayed
Returns:  none
*************************************************************************/
void err_puts(const char *s)
/* print string on lcd (no auto linefeed) */
{

    register char c;

    while ( (c = *s++) ) {
        err_putc(c);
    }

}/* err_puts */



void err_putint(uint8_t zahl)
{
	//char string[5];
	char string[4];
	int8_t i;                             // schleifenzähler
	
	//  string[4]='\0';                       // String Terminator
	string[3]='\0';                       // String Terminator
	for(i=2; i>=0; i--) {
		string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
		zahl /= 10;
	}
	err_puts(string);
}

void err_putint2(uint8_t zahl)	//zweistellige Zahl
{

	char string[3];
	int8_t i;								// Schleifenzähler
	zahl%=100;								// 2 hintere Stelle
	//  string[4]='\0';                     // String Terminator
	string[2]='\0';							// String Terminator
	for(i=1; i>=0; i--) {
		string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
		zahl /= 10;
	}
	err_puts(string);
}

void err_putint1(uint8_t zahl)	//einstellige Zahl
{
	//char string[5];
	char string[2];
	zahl%=10;								//  hinterste Stelle
	string[1]='\0';							// String Terminator
	string[0]=zahl +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
	err_puts(string);
}


void err_puthex(uint8_t zahl)
{
	//char string[5];
	char string[3];
	uint8_t l,h;                             // schleifenzähler
	
	string[2]='\0';                       // String Terminator
	l=(zahl % 16);
	if (l<10)
	string[1]=l +'0';  
	else
	{
	l%=10;
	string[1]=l + 'A'; 
	
	}
	zahl /=16;
	h= zahl % 16;
	if (h<10)
	string[0]=h +'0';  
	else
	{
	h%=10;
	string[0]=h + 'A'; 
	}
	
	
	err_puts(string);
}




/*
 * Strobes the E signal on the LCD to "accept" the byte in the '164.  The RS
 * line determines wheter the byte is a character or a command.
*/
void
err_strobe_E(void)
{
        /* strobe E signal */
        ERR_PORT |= _BV(ERR_ENABLE_PIN);
		_delay_us(450); 
	//	_delay_us(450); 
	//	lcddelay_ms(100);
        ERR_PORT &= ~_BV(ERR_ENABLE_PIN);
}

/*
 * Moves the cursor to the home position.
*/
void
err_cursor_home(void)
{
        err_load_byte(ERR_CMD_HOME);
        err_send_cmd();
}

/*
 * Moves the cursor to the specified position.
 * Parameters:
 *      line            Line (row)
 *      pos             Position on that line (column)
*/
void
err_move_cursor(uint8_t line, uint8_t pos)
{
//fleury: err_command((1<<ERR_DDRAM)+addressCounter)
        err_load_byte(line+pos);
        err_send_cmd();
}

/*
 * Moves the cursor a number of spaces to the right
 * Parameters:
 *      spaces          Number of spaces to move
*/
void
err_inc_cursor(uint8_t spaces)
{
        while (spaces--)
        {
                err_load_byte(ERR_CMD_CURSOR_RIGHT);
                err_send_cmd();    
        }
}

/*
 * Moves the cursor a number of spaces to the left
 * Parameters:
 *      spaces          Number of spaces to move
*/
void
err_dec_cursor(uint8_t spaces)
{
        while (spaces--)
        {
                err_load_byte(ERR_CMD_CURSOR_LEFT);
                err_send_cmd();    
        }
}

/*************************************************************************
Set cursor to specified position
Input:    x  horizontal position  (0: left most position)
          y  vertical position    (0: first line)
Returns:  none
*************************************************************************/
void err_gotoxy(uint8_t x, uint8_t y)
{
switch (y)
{
    case 0:
        err_load_byte((1<<ERR_DDRAM)+ERR_START_LINE1+x);
		err_send_cmd();
		break;
   case 1:
		err_load_byte((1<<ERR_DDRAM)+ERR_START_LINE2+x);
		err_send_cmd();
		break;
	case 2:
		err_load_byte((1<<ERR_DDRAM)+ERR_START_LINE3+x);
		err_send_cmd();
		break;
	case 3:
		err_load_byte((1<<ERR_DDRAM)+ERR_START_LINE4+x);
		err_send_cmd();
		break;
	

}//switch


}/* err_gotoxy */


// Linie Loeschen

void err_clr_line(uint8_t Linie)
{
	err_gotoxy(0,Linie);
	uint8_t i=0;
	for (i=0;i<ERR_DISP_LENGTH;i++)
	{
		err_putc(' ');
	}
	err_gotoxy(0,Linie);
	
}	// Linie Loeschen


// Teil einer Linie loeschen
void err_clr_part(uint8_t Linie, uint8_t von, uint8_t bis)
{
	if (bis>LCD_DISP_LENGTH)
		bis=LCD_DISP_LENGTH;
		
	err_gotoxy(von,Linie);
	uint8_t i=0;
	for (i=von;i<bis;i++)
	{
		err_putc(' ');
	}
	err_gotoxy(von,Linie);
	
}	// Teil einer Linie loeschen




// Display loeschen
void err_cls()   
{
	err_load_byte(0x01);
	err_send_cmd();
//	err_write(0x02,0);   	// B 0000 0010 => Display loeschen
	lcddelay_ms(2);			// dauert eine Weile, Wert ausprobiert

//	err_write(0x01,0);   	// B 0000 0001 => Cursor Home
	err_load_byte(0x02);
	err_send_cmd();
	
	lcddelay_ms(2);			// dauert eine Weile, Wert ausprobiert
}




/*
 
Funktion zur Umwandlung einer vorzeichenlosen 32 Bit Zahl in einen String
 
*/
/* 
void r_uitoa(uint32_t zahl, char* string) {
  int8_t i;                             // schleifenzähler
 
  string[10]='\0';                       // String Terminator
  for(i=9; i>=0; i--) {
    string[i]=(zahl % 10) +'0';         // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
}
*/

/*
 
Funktion zur Umwandlung einer vorzeichenbehafteten 32 Bit Zahl in einen String
 
*/
 /*
void r_itoa(int32_t zahl, char* string) 
{
  uint8_t i;
 
  string[11]='\0';                  // String Terminator
  if( zahl < 0 ) {                  // ist die Zahl negativ?
    string[0] = '-';              
    zahl = -zahl;
  }
  else string[0] = ' ';             // Zahl ist positiv
 
  for(i=10; i>=1; i--) {
    string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
}
*/
 /*
void r_itoa16(int16_t zahl, char* string) 
{
  uint8_t i;
 
  string[7]='\0';                  // String Terminator
  if( zahl < 0 ) {                  // ist die Zahl negativ?
    string[0] = '-';              
    zahl = -zahl;
  }
  else string[0] = ' ';             // Zahl ist positiv
 
  for(i=6; i>=1; i--) {
    string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
    zahl /= 10;
  }
}
*/


/*
 
Funktion zur Anzeige einer 32 Bit Zahl im Stringformat
auf einem LCD mit HD44780 Controller
Quelle: www.mikrocontroller.net/articles/Festkommaarithmetik
Parameter:
 
char* string  : Zeiger auf String, welcher mit my_itoa() erzeugt wurde
uint8_t start : Offset im String, ab der die Zahl ausgegeben werden soll,
                das ist notwenig wenn Zahlen mit begrenztem Zahlenbereich
                ausgegeben werden sollen
                Vorzeichenlose Zahlen      : 0..10
                Vorzeichenbehaftete zahlen : 1..11
uint8_t komma : Offset im String, zeigt auf die Stelle an welcher das virtuelle
                Komma steht (erste Nachkommastelle)
                komma muss immer grösser oder gleich start sein !
 
uint8_t frac  : Anzahl der Nachkommastellen
 
*/
 
void err_put_frac(char* string, uint8_t start, uint8_t komma, uint8_t frac) 
{
 
  uint8_t i;            // Zähler
  uint8_t flag=0;       // Merker für führende Nullen
 
  // Vorzeichen ausgeben  
  if (string[0]=='-') err_putc('-'); else err_putc(' ');
 
  // Vorkommastellen ohne führende Nullen ausgeben
  for(i=start; i;i--) {
    if (flag==1 || string[i]!='0') {
      err_putc(string[i]);
      flag = 1;
    }
    else err_putc(' ');         // Leerzeichen
  }
 
  err_putc('.');                // Komma ausgeben
 
  // Nachkommastellen ausgeben
  for(; i<(komma+frac); i++) err_putc(string[i]);
 
}

void err_put_zeit(uint8_t minuten, uint8_t stunden)
{
	//							13:15
	if (stunden< 10)
	{
		//	err_putc(' ');
	}
	
	char zeitString[6];
	zeitString[5]='\0';
	
	//	Minuten einsetzen
	zeitString[4]=(minuten % 10) +'0';	//hinterste Stelle
	if (minuten>9)
	{
		minuten/=10;
		zeitString[3]=(minuten % 10) +'0';
	}
	else
	{
		zeitString[3]='0';
	}
	 
	zeitString[2]=':';
	
	//	Stunden einsetzen
	zeitString[1]=(stunden % 10) +'0'; 
	if (stunden>9)
	{		
		stunden/=10;
		zeitString[0]=(stunden % 10) +'0';
	}
	else
	{
		zeitString[0]='0';
	}
	 
	
	err_puts(zeitString);
}







/*************************************************************************/






