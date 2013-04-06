//
//  TWI_Master.c
//  TWI_Master
//
//  Created by Sysadmin on 19.03.08.
//  Copyright Ruedi Heimlicher 2008. All rights reserved.
//



#include <avr/io.h>
# include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <inttypes.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <ctype.h>

# include  "TWI_Master.h"
# include "twimaster.c"
# include "lcd.c"
# include "err.c"
# include "adc.c"
# include "slaves.c"
#include "datum.c"
#include "version.c"


#include "sensor.c"


/* ************************************** */
volatile uint8_t rxdata =0;



/* *** SPI *********************************** */
uint8_t testCounterON =0;
uint8_t testCounterOFF =0;
uint8_t loopCounterTWI =0;// Anzahl TWI-Loops
uint8_t loopCounterSPI =0;// Anzhl SPI-Loops
/* *** end SPI *********************************** */


/* ************************************** */
//static char d[3];
//static char* key1;
//static char* sstr;
//char VarString[64];
//static char DataString[48];
//static char EEPROM_String[96];

static char SolarString[48];

//static char d[3];
//static char* key1;
//static char* sstr;


#define test 1 // Start ohne Webserver. Takt fuer TWI von Master generiert


#define SCLPIN		0
#define SDAPIN		1

#define WEB_ON		1
#define IOW_TYP		24

#define PRELL 2

#define HEIZUNG		0
#define WERKSTATT		1
#define WOZI			2
#define BUERO			3
#define LABOR			4
#define OG1				5
#define OG2				6
#define ESTRICH		7

#define FEHLERBYTE	24		//Beginn der Fehlermeldungen

#define data_buffer_size	32

#define buffer_size			8

#define ERRTASK				0xA0	// Fehlertask an Webserver schicken, soll Eintrag ins Log veranlassen
#define ERR_UHR				0xA0



#define NULLTASK				0xB0	// Nichts tun
#define STATUSTASK			0xB1	// Status des TWI aendern
#define STATUSCONFIRMTASK	0xB2	// Statusaendern bestaetigen



#define DATATASK				0xC0	// Normale Loop im Webserver

#define SOLARTASK				0xC1	// Data von solar

#define MASTERERRTASK		0xC7	// Fehlermeldung vom Master senden



// defines fuer Alarm
#define TIEFKUEHLALARM		3
#define WASSERALARMKELLER	4
#define WASSERALARMESTRICH	1
// defines fuer spistatus



#define SPI_SHIFT_IN_OK_BIT	6

// defines fuer BUS-Status
#define SPI_SENDBIT				0
#define TWI_CONTROLBIT			1				// Statusbit fuer TWI
#define WEB_CONTROLBIT			2				// Statusbit fuer Web

// Uhr

volatile uint8_t  uhrstatus =0;

// defines fuer uhrstatus
#define SYNC_OK		0	// Uhr ist synchronisiert
#define SYNC_WAIT		1	// Uhr ist  wartet auf Synchronisation
#define SYNC_READY	2	// DCF77 hat gueltiges Datum 
#define SYNC_CHECK	3	// DCF77 soll Datum bereitstellen, Anzahl korrekte Daten abwarten
#define SYNC_NULL		4	// Uhr ist undefiniert, wartet auf Synchronisation (nach restart)
#define SYNC_NEW		5	// erste Synchrinisation nach Reset, noch keine gueltige Zeit

volatile uint8_t  DCF77_counter =0; // Anzahl gueltige Datumspakete in Folge
#define MIN_SYNC		2	// Anzahl gueltige Daten fuer Synchronisation


// defines fuer EEPROMstatus
#define PWM_READ     0  // Daten fuer PWM lesen


#define TASTE1 38
#define TASTE2 46
#define TASTE3 54
#define TASTE4 72
#define TASTE5 95
#define TASTE6 115
#define TASTE7 155
#define TASTE8 186
#define TASTE9 205
#define TASTEL 225
#define TASTE0 235
#define TASTER 245

#define STARTDELAY 0x08F
//#define STARTDELAY 0

#define WOCHENPLANBREITE 0x40;

#define START_BYTE_DELAY	12		// Timerwert fuer Start-Byte
#define BYTE_DELAY			12		// Timerwert fuer Data-Byte

uint8_t Raum_Thema=0x00;			//	Bit 4-7: Thema		Bit 0-3: Raum
uint8_t Objekt_Wochentag=0;		//	Bit 4-7: Objekt		Bit 0-3: Wochentag
uint8_t Stunde_Minute=0;
uint8_t Menu_Ebene=0;

volatile uint8_t						sensorstatus=0;
uint8_t sensorcounter=0;

unsigned char mcusr_mirror __attribute__ ((section (".noinit"))); // Watchdog ausschalten 

void get_mcusr(void) \
      __attribute__((naked)) \
      __attribute__((section(".init3"))); 
    
	 void get_mcusr(void) 
    { 
     mcusr_mirror = MCUSR; 
     MCUSR = 0; 
    wdt_disable(); 
    } 

/* ************************************** */

// the password string (only the first 5 char checked), (only a-z,0-9,_ characters):
static char *errmsg; // error text

static volatile uint8_t Temperatur;

/* ************************************** */



void timer0(void);



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}


void tempbis99(uint16_t temperatur,char*tempbuffer)
{
	char buffer[8]={};
	//uint16_t temp=(temperatur-127)*5;
	uint16_t temp=temperatur*5;
	
	//itoa(temp, buffer,10);
	
	r_itoa16(temp,buffer);
	
	//lcd_puts(buffer);
	//lcd_putc('*');
	
	//char outstring[7]={};
	
	tempbuffer[6]='\0';
	tempbuffer[5]=' ';
	tempbuffer[4]=buffer[6];
	tempbuffer[3]='.';
	tempbuffer[2]=buffer[5];
	if (abs(temp)<100)
	{
		tempbuffer[1]=' ';
		
	}
	else 
	{
		tempbuffer[1]=buffer[4];
		
	}		
	tempbuffer[0]=buffer[0];
}

void tempAbMinus20(uint16_t temperatur,char*tempbuffer)
{
	
	char buffer[8]={};
	int16_t temp=(temperatur)*5;
	temp -=200;
	char Vorzeichen=' ';
	if (temp < 0)
	{
		Vorzeichen='-';
	}
	
	r_itoa16(temp,buffer);
	//		lcd_puts(buffer);
	//		lcd_putc(' * ');
	
	//		char outstring[7]={};
	
	tempbuffer[6]='\0';
	//outstring[5]=0xDF; // Grad-Zeichen
	tempbuffer[5]=' ';
	tempbuffer[4]=buffer[6];
	tempbuffer[3]='.';
	tempbuffer[2]=buffer[5];
	if (abs(temp)<100)
	{
		tempbuffer[1]=Vorzeichen;
		tempbuffer[0]=' ';
	}
	else
	{
		tempbuffer[1]=buffer[4];
		tempbuffer[0]=Vorzeichen;
	}
	//		lcd_puts(outstring);
}





/*
 Der Buffer, in dem die empfangenen Daten gespeichert werden. Der Slave funktioniert Šhnlich  wie ein normales
 Speicher-IC [I2C-EEPROM], man sendet die Adresse, an die man schreiben will, dann die Daten. Die interne Speicher-Adresse
 wird dabei automatisch hochgezŠhlt
 */
volatile uint8_t rxbuffer[buffer_size];
//volatile uint8_t rxstartbuffer=0;
//volatile uint8_t WebRxDaten[tag_data_size];
//static volatile uint8_t WebRxStartDaten=0;

/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
volatile uint8_t txbuffer[buffer_size];//={0,0,0,0,0,0,0,0};
//volatile uint8_t txstartbuffer=0;
uint8_t senderfolg=0;



static volatile uint8_t StartDaten;
	static volatile uint8_t min=0;
	static volatile  uint8_t std=0;
	static volatile  uint8_t tag=0;
	
	static volatile uint8_t oldmin=0;
	static volatile  uint8_t oldstd=0;
	static volatile  uint8_t oldtag=0;


uint16_t			Brennerzeit=0;

uint16_t EEMEM Brennerlaufzeit;	// Akkumulierte Laufzeit
//uint8_t LaborDaten[8]={};
//uint8_t HeizungDaten[8]={};
//uint8_t EstrichDaten[8]={};

//Status Receive TWI
static volatile uint8_t LeseStatus=0;

//Status Transmit TWI
static volatile uint8_t SchreibStatus=0;

//Status WEB
volatile uint8_t WebStatus=0x00; //	Webserver abfragen


//Status MANUELL
//volatile uint8_t TWI_Status=0x00;//		Anfangsposition: TWI ist OFF

//Status Loop
volatile uint8_t BUS_Status=0x00;//		Anfangspos: TWI OFF, wird nach Startdelay eingeschaltet, WEB ist am Anfang OFF


#pragma mark Signal Status
//Status Timer
volatile uint8_t SIGNAL_Status=19;
volatile uint8_t RW=0; //lesen oder schreiben

//	SIGNAL_Count
volatile uint8_t SIGNAL_Count=0; // Anzahl Interrupts bis TWI abfragen


void BlinkC(uint8_t  PIN, uint8_t anz);

extern unsigned char i2c_readAck(void);
extern unsigned char i2c_readNak(void);


uint8_t Hex2Int(char *s) 
{ 
	long res; 
	char *Chars = "0123456789ABCDEF", *p; 
	
	if (strlen(s) > 8) 
	/* Error ... */ ; 
	
	for (res = 0L; *s; s++) { 
		if ((p = strchr(Chars, toupper(*s))) == NULL) 
		/* Error ... */ ; 
		res = (res << 4) + (p-Chars); 
	} 
	
	return res; 
} 

/*
 ISR (TIMER0_COMPA_vect) 
 { 
 
 if ((SIGNAL_Count > SIGNAL_Status))//&& (Ein0==0)) //Tastendauer ist nicht ein
 { 
 
 delay_ms(20);
 
 }
 else 
 {
 SIGNAL_Count++;
 }
 
 }
 */

//ISR (TIMER0_OVF_vect) 
 



/*
 void timer0()
 {
 OSZIALO;
 delay_ms(5);
 
 
 //----------------------------------------------------
 // Set up timer 0 to generate interrupts @ 1000Hz
 //----------------------------------------------------
 TCCR0A = _BV(WGM01);
 TCCR0B = _BV(CS00) | _BV(CS02);
 OCR0A = 0xFF;
 TIMSK0 = _BV(OCIE0A);
 }
 */


void timer0 (void) 
{ 	
//	OSZIALO;
	//delay_ms(1);
	//OSZIAHI;
	//err_gotoxy(0,10);
	//err_puts("timer0\0");
	TCCR0A |=(1 << WGM01);
	TCCR0B |= (1<<CS00)|(1<<CS02);	//Takt /1024
	//TCCR0B |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
	//	TCCR0 |= (1<<CS00)|(1<<CS01);	//Takt /64
	OCR0A=0xFF;
	
	
	TIFR0 |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);			//Overflow Interrupt aktivieren
	//	TIMSK0 |= OCIE0A;					// Clear Timer on Compare Match, CTC
	TCNT0=0x00;						//RŸcksetzen des Timers
	
} 

#pragma mark TIMER2
void timer2 (uint8_t wert) 
{ 
	err_puts("timer2\0");
	//OSZIALO;
	TCCR2B |= (1<<CS20)|(1<<CS21)|(1<<CS22);	//Takt /1024	Intervall 32 us
	
	TCCR2A = 0;
	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
	
	//OC2 akt
	//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC
	
	TIFR2 |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren
	
	//TCCR2A = 0x00;					//Zaehler zuruecksetzen
	TCNT2=0;
	OCR2A = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 



void eep_write_wochentag(uint8_t *ablauf[24], uint8_t *tag)
{
	eeprom_write_block((void*)ablauf, tag, 24);
}


void setTagplan(uint8_t *Daten)
{
	/*
	 erste halbe Stunde:		Cx	2 bits links
	 zweite halbe Stunde:	3x	2 bits rechts
	 ganze Stunde			Fx	alle 4 bits
	 Schalterstellung: Bit 3
	 Schalter ein			x8
	 Schalter aus			x0
	 */	
	//uint8_t tagblock[24];
	uint8_t i=0;
	for (i=0;i<buffer_size;i++)
	{
		if ((i<7)||(i>22))
		{
			Daten[i]= 0x00;
		}
		else
		{
			Daten[i]= 0xC8; // 200  ganze Stunden, 6 - 21 Uhr
		}
		
	}
	
	Daten[7]=0x48;		// 72
	Daten[22]=0x88;		//136
	
	Daten[9]=0x88;	//	Beginn Pause 9.30
	Daten[10]=0x08;
	Daten[11]=0x08;
	Daten[12]=0x48;	//	Ende Pause	12.30
	/*
	 for (i=0;i<buffer_size;i++)
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
}

void masterinit(void)
{

	DDRB &= ~(1<<5); // Pin 5 von Port B als Eingang fuer Servotest 0
	PORTB |= (1<<5); //HI

	DDRB &= ~(1<<6); // Pin 6 von Port B als Eingang fuer Servotest 1
	PORTB |= (1<<6); //HI

	DDRB &= ~(1<<7); // Pin 7 von Port B als Eingang fuer Servotest 2
	PORTB |= (1<<7); //HI
	
	
	
	
	
// 	DDRA &= ~(1<<DDA0);	//Pin 0 von PORT A als Eingang fuer Tastatur 	
//	PORTA |= (1<<DDA0); //Pull-up
	
	// TWI
	DDRC |= (1<<0);	//Pin 0 von PORT C als Ausgang (SCL)
	PORTC |= (1<<0);	//	ON
	DDRC |= (1<<1);	//Pin 1 von PORT C als Ausgang (SDA)
	PORTC |= (1<<1);	//	ON
	
	
	// erst in HS nach Ablauf Startddelay
	//	DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer TWI SDA
	//	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer TWI SCL
	
	
	DDRB &= ~(1<<DDB0);	//Pin 0 von PORT B als Eingang fuer Taster 0
	PORTB |= (1<<DDB0); //Pull-up
	
	DDRB &= ~(1<<DDB1);	//Pin 1 von PORT B als Eingang fuer Taster 1
	PORTB |= (1<<DDB1); //Pull-up
	
	DDRC |= (1<<2);	//Pin 2 von PORT C als Ausgang fuer TWI-Anzeige
	PORTC |= (1<<2); // ON
	DDRC |= (1<<TWI_CONTROLPIN);	//Pin 3 von PORT C als Ausgang fuer TWI-Anzeige
	PORTC |= (1<<TWI_CONTROLPIN); // ON
	
	//fuer Blinky
	
	DDRC |= (1<<LOOPLEDPIN);	//Pin 2 von PORT C als Ausgang fuer Loopcount
	
	
	
	//*********************************************************************
	//	Definitionen LCD
	//	Definitionen in lcd.h
	//*********************************************************************
	//	char* wochentag[] = {"MO","DI","MI","DO","FR","SA","SO"};
	//volatile int count=0;
	
	//volatile uint8_t Schaltposition=0;
	
	
	DDRC |= (1<<LCD_RSDS_PIN); //PIN 5 von PORT B als Ausgang fuer LCD_RSDS_PIN
	DDRC |= (1<<LCD_ENABLE_PIN); //PIN 6 von PORT B als Ausgang fuer LCD_ENABLE_PIN
	DDRC |= (1<<LCD_CLOCK_PIN); //PIN 7 von PORT B als Ausgang fuer LCD_CLOCK_PIN
	
	
	DDRA |= (1<<ERR_RSDS_PIN); //PIN 5 von PORT A als Ausgang fuer ERR_RSDS_PIN
	DDRA |= (1<<ERR_ENABLE_PIN); //PIN 6 von PORT A als Ausgang fuer ERR_ENABLE_PIN
	DDRA |= (1<<ERR_CLOCK_PIN); //PIN 7 von PORT A als Ausgang fuer ERR_CLOCK_PIN
	
	//DDRD= 0xFF;
	
	SRDDR |= (1<<SR_CLK_PIN);// PIN fuer Ausgang Clock fuer SR
	SRPORT |= (1<<SR_CLK_PIN);// HI
	
	SRDDR |= (1<<SR_LOAD_PIN);// PIN fuer Ausgang Enable fuer SR
	SRPORT |= (1<<SR_LOAD_PIN);// HI
	
	SRDDR &= ~(1<<SR_DATA_PIN);// PIN fuer Eingang Daten von SR
	SRPORT |= (1<<SR_DATA_PIN);// HI
	
	
}

uint8_t Tastenwahl(uint8_t Tastaturwert)
{
	if (Tastaturwert < TASTE1)
		return 1;
	if (Tastaturwert < TASTE2)
		return 2;
	if (Tastaturwert < TASTE3)
		return 3;
	if (Tastaturwert < TASTE4)
		return 4;
	if (Tastaturwert < TASTE5)
		return 5;
	if (Tastaturwert < TASTE6)
		return 6;
	if (Tastaturwert < TASTE7)
		return 7;
	if (Tastaturwert < TASTE8)
		return 8;
	if (Tastaturwert < TASTE9)
		return 9;
	if (Tastaturwert < TASTEL)
		return 10;
	if (Tastaturwert < TASTE0)
		return 0;
	if (Tastaturwert < TASTER)
		return 12;
	
	return -1;
}

void initOSZI(void)
{
	OSZIPORTDDR |= (1<<PULSA);
	OSZIPORT |= (1<<PULSA);	
	OSZIAPORTDDR |= (1<<PULSB);
	OSZIPORT |= (1<<PULSB);

}




uint8_t SensorAbrufen (void)
{
	uint8_t sensorerfolg=0;
   
	sensorerfolg=SlavedatenLesen(SENSOR_ADRESSE,(void*)Sensordaten);
	
	return sensorerfolg;
	// end Uhr lesen
	
}


int main (void)
{
	//JTAG deaktivieren (datasheet 231)
	MCUCR |=(1<<7);
	MCUCR |=(1<<7);
	cli();
	uint8_t ch = MCUSR;
	MCUSR &= ~(1<<WDRF);
	MCUSR = 0;
	
	wdt_disable();
	MCUSR &= ~(1<<WDRF);
	wdt_reset();
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0x00;
	
	// Check if the WDT was used to reset
	//if (! (ch &  _BV(EXTRF))) // if its a not an external reset...
	{
		//
	}
	
	//*********************************************************************
	
	masterinit();
	
	uint8_t TastaturCount=0;
	Brennerzeit=0;
	uint8_t i=0;
	for (i=0;i<8;i++)
	{
		DCF77daten[i]=0x00;
		txbuffer[i] = 0;
	}
	
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	delay_ms(50);
	//	lcd_CGRAMInit_A();
	
	// Zeichensatz laden
	lcd_CGRAMInit_Titel();
	
	delay_ms(50);
	lcd_puts("Guten Tag\0");
	delay_ms(800);
	lcd_cls();
	lcd_puts("Master ready\0");
	
	err_initialize(ERR_FUNCTION_8x2, ERR_CMD_ENTRY_INC, ERR_CMD_ON);
	err_puts("Err ready\0");
	delay_ms(800);
	
	
	//	initADC(0);
	uint8_t Tastenwert=0;
	//	uint8_t Servowert=0;
	
	lcd_cls();
	err_cls();
	/*
	lcd_gotoxy(14,0);
	lcd_puts("V:\0");
	lcd_puts(VERSION);
	delay_ms(800);
	*/
//	lcd_gotoxy(19,1); 
//	lcd_putc(165);	//	Punkt an letzter Stelle
	
	Raum_Thema=0x00;
	
	
	Zeit.minute=0;
	Zeit.stunde=0;
	Zeit.wochentag=0;
	uint8_t AnzeigeWochentag=0;
	
	volatile uint8_t neueZeit=0;
	
	
	uint16_t startdelay=STARTDELAY;
	
	//startdelay=0;
	
	uint8_t Taste=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x1F;
	
	//uint8_t web_request=0;
	
	
	
	volatile uint16_t loopcount0=0;
	volatile uint16_t loopcount1=0;
	
	uint8_t twi_LO_count0=0;
	uint8_t twi_LO_count1=0;
	
	uint8_t twi_HI_count0=0;
	//uint16_t twi_HO_count1=0;
	
	uint8_t twierrcount=0;		//	Anzahl TWI-Resets
	
	
	/*
	 // Bisherige Watchdog-Resets lesen
	 tempWDT_Count=eeprom_read_byte(&WDT_ErrCount);;	// Bisherige Watchdog-Resets
	 
	 if (tempWDT_Count==0xFF) // Erster Start nach neuer programmierung, zuruecksetzen
	 {
	 tempWDT_Count=0;
	 eeprom_write_byte(&WDT_ErrCount,tempWDT_Count);		// Anzahl reset
	 delay_ms(5);
	 }
	 */	
	
	//delay_ms(300);
	
	
	//Websr
	// ******************
	//ByteCounter=0xFF;
	//WebTxStartDaten = 0x00;
	//SolarTxStartDaten = 0x00;
	/*
	 Bus_Status:
	 Bit 0:
	 Bit 1:	TWI_CONTROLBIT
	 Bit 2:
	 
	 
	 */
	/******************************************************************/	
	// set the clock speed to "no pre-scaler" (8MHz with internal osc or 
	// full external speed)
	// set the clock prescaler. First write CLKPCE to enable setting of clock the
	// next four instructions.
	
	// Software-Umstellung der CPU-Frequenz auf 8 MHz
	
	//	CLKPR=(1<<CLKPCE); // change enable
	//	CLKPR=0; // "no pre-scaler"
	_delay_loop_1(0); // 60us
	
	BUS_Status |=  (1<<TWI_CONTROLBIT);			// TWI am Anfang einschalten

	delay_ms(10);
	/******************************************************************/
	
	//lcd_gotoxy(18,1); 
	//lcd_putc('-');	//	Erste Runde, Strich an letzter Stelle
	//timer0();
	/******************************************************************/
	uhrstatus=0;
	uhrstatus |= (1<<SYNC_NULL); // Uhr undef, warten auf DCF77
	
	initOSZI();
	/******************************************************************/
	
	/*** Hauptschleife															***/
	
	/******************************************************************/
#pragma mark while
	while (1)
	{
		
		// Startfunktion: SCL und SDA pruefen
		if (startdelay==STARTDELAY)
		{
			//err_gotoxy(19,1); 
			//err_putc('+');	//	Erste Runde, + an letzter Stelle
			//lcd_puthex(startdelay);
			delay_ms(2);
			err_gotoxy(19,1); 
			err_putc(' ');	//	Erste Runde, Strich an letzter Stelle weg
			
		}
		
		wdt_reset();
		
		//	Startroutine noch im Gang: TWI beide Pins HI also RŸckwaertszaehlen
		if( startdelay && ((PINC & (1<<SCLPIN) && (PINC & (1<<SDAPIN)))))	// SCL UND SDA ist HI
		{
			if (startdelay==1) // Letzter Durchlauf vor einschalten
			{
				DDRC &= ~(1<<DDC1);	//Pin 1 von PORT C als Eingang fuer TWI SDA
				PORTC |= (1<<DDC1); //Pull-up
				DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer TWI SCL
				PORTC |= (1<<DDC0); //Pull-up
				BUS_Status |=(1<<TWI_CONTROLBIT);		// TWI ON
				BUS_Status &=~(1<<WEB_CONTROLBIT);		// WEB OFF
				lcd_clr_line(0);
				err_gotoxy(0,1); 
				err_puts("Start\0");	//	Erste Runde, Strich an letzter Stelle weg
#pragma mark RTC init				
				
            i2c_init();
				
				delay_ms(50);
				sei();
				
				
			}
			
			startdelay--;
			
		}
		else
		{
			//err_gotoxy(10,1); 
			//err_puthex(startdelay);
		}
		
		loopcount0++;
		if (loopcount0 >= 0x0FFF)
		{
			LOOPLEDPINPORT ^=(1<<LOOPLEDPIN); // Blink-LED
			
			loopcount0=0;
			loopcount1++;
			if (startdelay==0)
			{
				//				timer2(0xAF);
				//				sei();
				//SPI_shift();
				
			}
			
			if (loopcount1>=0x0F)
			{
            
            sensorstatus |= (1<<SPI_SHIFT_IN_OK_BIT);
				//lcd_puthex(loopcount1);
				loopcount1=0;
			}
			
		}
		
		
		//	Checken, ob SCL oder SDA lŠngere Zeit low sind. 
		//	In diesem Fall zuerst TWI reseten, bei lŠngerem Fehler reset des Prozessors
		
		
		// Startroutinen sind abgelaufen oder ganz am Anfang  
		// SDA und SCL sind laengere Zeit nicht gleichzeitig HI: Fehlersituation
		
		if (((startdelay==0)||(startdelay==STARTDELAY))&& (((!(PINC & (1<<SDAPIN))) && PINC & (1<<SCLPIN)) ) )// SDA ist LO und SCL ist HI (warten auf Ack)
		{
			err_gotoxy(15,1);
			err_puts("ERR\0");

			/*
			
1. TWI Modul am Master abschalten.
2. Am Master SDA als Input und SCL als Output konfigurieren.
3. SCL im TWI Takt so lange toggeln bis SDA wieder high ist
4. TWI wieder initialisieren und weiter machen ...

Du mu§t also das HW-TWI abschalten und per SW-I2C einen SCL-Puls
generieren und dann versuchen, Stop zu senden. Erst dann ist der Slave
wieder adressierbar.
			*/
         
			// Zaehlen, wieviele Runden der Fehler dauert
			twi_LO_count0++;
			
			// nach einer Anzahl Runden  die zweite Anzahl twi_LO_count1 inkrementieren
			if (twi_LO_count0 >=0xAFF)
			{
				twi_LO_count0=0;
				twi_LO_count1++;
				err_gotoxy(0,1);
				err_puts("lc1 \0");
				err_puthex(twi_LO_count1);
				
				// Erste Grenze von twi_LO_count1 erreicht, also erneut stop senden
				if (twi_LO_count1>=0x0F) //	
				{
					//					PORTC |= (1<<TWICOUNTPIN); //TWI-LED ON
					
					// TWI-Fehler inkrementieren
					twierrcount++;
					// Wenn Fehler andauert, TWI neu starten
					if (twi_LO_count1 >=0xAF) // 
					{
						err_gotoxy(12,0);
						err_puts("deb\0");
						TWBR =0;
						TWCR =0;
						
						uint8_t deb=i2c_debloc();
						err_puthex(deb);
                  delay_ms(10);
						TWI_DDR &= ~(1<<SCL_PIN);	// SCL-Pin wieder als EINgang
						TWI_PORT |= (1<<SCL_PIN);  // HI
						
						delay_ms(10);
						i2c_init();
						rtc_init();
						i2c_stop();
						wdt_reset();
					}
					//err_gotoxy(10,1);
					//err_puts("st\0");
					//err_puthex(twi_LO_count1);
					//delay_ms(10);
					
				}
			}
			
		}
      
		else // alles in Ordnung, Fehler zuruecksetzen
		{
			//		PORTC &= ~(1<<TWICOUNTPIN); //TWI-LED OFF
			twi_LO_count0=0;
			twi_LO_count1=0;
			wdt_reset();
		}
		
		
		if (sensorstatus & (1<<SPI_SHIFT_IN_OK_BIT))	// Shift-Bit ist nach 'neu PASSIVE' gesetzt, Datentausch ist erfolgt und OK
		{
         sensorstatus &= ~(1<<SPI_SHIFT_IN_OK_BIT);
         
         
#pragma mark sensor
         //	******************************
         //	*	Sensor abfragen
         //	******************************
         lcd_cls();
         TWI_Flag=0;
         lcd_gotoxy(0,0);
         lcd_putc('S');
         sensorcounter++;
         uint8_t sensorerfolg = SensorTemperaturLesen(SENSOR_ADRESSE,Sensordaten);
         
         lcd_puthex(sensorerfolg);
         lcd_putc(' ');
         lcd_puthex(TWI_Flag);
         
         //lcd_puthex(Sensordaten[0]);
         //lcd_puthex(Sensordaten[1]&0xFC);
         //lcd_putc(' ');
         //lcd_puthex(Sensordaten[2]);
         //lcd_putc(' ');
         
         //lcd_puthex(sensorcounter);
         uint16_t sensorwert = Sensordaten[0];
         sensorwert <<=8;
         sensorwert += (Sensordaten[1]&0xFC);
        
         float temperatur = -48.85+175.72*sensorwert/0xFFFF;

         lcd_gotoxy(10,0);
         lcd_putc('T');
         lcd_putc(':');
         lcd_putint2_right(temperatur);
         lcd_putc('.');
         temperatur*= 10;
         temperatur= (int)temperatur%10;
         lcd_putint1(temperatur);
         lcd_putc(0xDF);
         lcd_putc('C');
         
         // lcd_putc(' ');
         
         
         sensorerfolg = SensorHumidityLesen(SENSOR_ADRESSE,Sensordaten);
         lcd_gotoxy(1,1);
         lcd_puthex(sensorerfolg);
         lcd_putc(' ');
         lcd_puthex(TWI_Flag);
         //lcd_puthex(Sensordaten[0]);
         //lcd_puthex(Sensordaten[1]&0xFC);
         lcd_gotoxy(10,1);
         lcd_putc('H');
         lcd_putc(':');
         sensorwert=0;
         sensorwert = Sensordaten[0];
         sensorwert <<=8;
         sensorwert += (Sensordaten[1]&0xFC);
         float humidity = -6.0+125.0*sensorwert/0xFFFF;
         
         lcd_putint2_right(humidity);
         lcd_putc('%');
         lcd_putc(' ');
         
           
		} // if (spistatus & (1<<SPI_SHIFT_IN_OK_BIT))
		
      
		
		
		// **************************************************
		
		wdt_disable();
		
		/* *** SPI end **************************************************************/
		
	#pragma mark Taste 0
		
		wdt_reset();
		
		// TWI mit Taste toggeln
		if (!(PINB & (1<<PORTB0))) // Taste 0 TWI Ein/Aus
		{
			//err_gotoxy(12,0);
			//err_puts("P0 Down\0");
			//wdt_reset();
			if (! (TastenStatus & (1<<PORTB0))) //Taste 0 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PORTB0);
				Tastencount=0;
				//err_gotoxy(8,0);
				//err_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount ++;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				wdt_reset();
				if (Tastencount >= Tastenprellen)
				{
					err_gotoxy(8,0);
					err_puts("P0\0");
					//err_putint(Tastencount);
					err_putc(' ');
					err_puthex(BUS_Status);
					//err_putc('v');
					
					if (BUS_Status & (1<<TWI_CONTROLBIT)) //  TWI ist gesetzt > loeschen
					{
						BUS_Status &= ~(1<<TWI_CONTROLBIT);
						LeseStatus=0;
						SchreibStatus=0;
						PORTC &= ~(1<<TWI_CONTROLPIN); // TWI-LED OFF
						
						//web_request &= ~(1<<7);
						//err_putc('n');
						//err_puthex(BUS_Status);
					}
					else
					{
						
						BUS_Status |= (1<<TWI_CONTROLBIT);
						PORTC |= (1<<TWI_CONTROLPIN); // TWI-LED ON
						
					}
					
					
					Tastencount=0;
					TastenStatus &= ~(1<<PORTB0);
					
				}
			}//else
			
		}
		
		wdt_reset();
		
		if (!(PINB & (1<<PB1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P1 Down\0");
			
			if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht gedrueckt
			{
				TastenStatus |= (1<<PB1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
					i2c_stop();
					i2c_debloc();
					LeseStatus=0;
					SchreibStatus=0;

					Tastencount=0;
					TastenStatus &= ~(1<<PB1);
					
					
				}
			}//	else
			
		} // Taste 1
		
		uint8_t dataerfolg=0;
		
		wdt_reset();
		
		// Tastaturabfrage Start mit TWI, ohne WEB-Request, Startdelay abgezŠhlt
			
		if ((!(BUS_Status & (1<<WEB_CONTROLBIT))) && (BUS_Status & (1<<TWI_CONTROLBIT))  && (startdelay==0))	
		{
			
			//err_gotoxy(4,1);
			//err_puts("TWI ");
			
			/************ SCL SDA checken *************/
			
			if (PINC & (1<<0) && PINC & (1<<1)) 	// SCL oder SDA sind beide high
			{
				if (twi_HI_count0< 0xFF)
				{
					twi_HI_count0++; // Zeit messen, waehrend der beide HI sind
				}
			}
			/*************************/
			neueZeit=0;
			//err_gotoxy(8,0);
			//err_putc('i');
			//err_puthex(BUS_Status);
			
			//			err_gotoxy(19,0);
			//			err_puts("T\0");
			
			
			wdt_reset();
			
			
			
			
#pragma mark Tasten 
			
			
			
			
//			Tastenwert=0;
			initADC(tastatur_kanal);
			//err_gotoxy(10,1);
			//err_puts("TR \0");
			
//			Tastenwert=(uint8_t)(readKanal(tastatur_kanal)>>2);
			
			//err_puthex(Tastenwert);
			
			//			closeADC();
			
			//uint8_t Taste=-1;
			//		Tastenwert/=8;
			//		Tastenwert*=3;
			//		err_clr_line(1);
			//		err_gotoxy(0,1);
			//		err_puts("Taste \0");
			//		err_putint(Taste);
			//		delay_ms(200);
			
			
			
			if (Tastenwert>23) // ca Minimalwert der Matrix
			{
				//			wdt_reset();
				/*
				 0: Wochenplaninit
				 1: IOW 8* 2 Bytes auf Bus laden
				 2: Menu der aktuellen Ebene nach oben
				 3: IOW 2 Bytes vom Bus in Reg laden
				 4: Auf aktueller Ebene nach rechts (Heizung: Vortag lesen und anzeigen)									
				 5: Ebene tiefer
				 6: Auf aktueller Ebene nach links (Heizung: Folgetag lesen und anzeigen)									
				 7: 
				 8: Menu der aktuellen Ebene nach unten
				 9: DCF77 lesen
				 
				 12: Ebene hšher
				 */
				TastaturCount++;
				if (TastaturCount>=8)	//	Prellen
				{
					
					//err_clr_line(1);
					//err_gotoxy(0,1);
					//err_puts("ADC:\0");
					//err_putint(Tastenwert);
					
					Taste=Tastenwahl(Tastenwert);
					
					//err_gotoxy(12,1);
					//err_puts("T:\0");
					//err_gotoxy(14,1);
					//err_putint2(Taste);
					/*
					 err_putint2(Taste);
					 //delay_ms(1200);
					 //err_clr_line(1);
					 */
					TastaturCount=0;
					Tastenwert=0x00;
					
					uint8_t inBytes[4]={};
					
					switch (Taste)
					{
						case 0://WochenplanInit
						{ 
							break;
							// Blinken auf C2
							
						}
							break;
							
							
					
							
					}//switch Taste
					
				}
				//TastaturCount=0x00;
			}
			
		}								// end kein WEB-Request
		
		else							// start WEB-Request:  Bit 0 ist LOW
		{
			//err_gotoxy(4,1);
			//err_puts("--- ");
			
			//err_gotoxy(18,0);
			//delay_ms(1);
			//err_puts("I\0");	
			//delay_ms(1);
			
		}								// end mit IOW-Request
	}//while
	return 1;
	
	
}





