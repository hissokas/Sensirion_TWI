RoboterNetz.de
C - Programmierung (GCC u.a.) - Problem Atmega8 als I2C-Slave
Robo2004 - 03.07.2005, 14:12
Titel: Problem Atmega8 als I2C-Slave
Hallo zusammen,
hab ein kleines Problem damit einen ATMega8 als I2C-Slave zu benutzen.
Das ganze soll so funktionieren:
Der Master sendet ein Daten-Byte zum Slave, womit bestimmt wird welches Register der M8-Slave bei der nächsten lese aufforderung des Masters zurückgeben soll.
Jetzt zu mein Problem, wenn der Master als erstes den Wert 0x01 zum Slave sendet, welches dann zu folge haben sollte das der Inhalt der Valiable reg_IR01 beim nächste lesen zurückgegeben werden sollte. Dann wird Null zurück gegeben.
Beim der nächsten lese wird dann der Inhalt geliefert, d.h. Es wird immer um eine lese Operation verzögert der korrekte wert gelesen.
weiss jemand wie man das Problem beheben kann.
TWI_Slave.h
Code:
#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef NULL
#define NULL 0
#endif
/****************************************************************************
 TWI Verbindungs Status
 ****************************************************************************/
#define TWI_CS_NONE 0x00
#define TWI_CS_READ 0x01
#define TWI_CS_WRITE 0x02
#define TWI_CS_GENERAL 0x03
/****************************************************************************
 Bit and byte definitions
 ****************************************************************************/
#define TWI_READ_BIT  0
// Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1
// Bit position for LSB of the slave address bits in the init byte.
#define TWI_GEN_BIT
0
// Bit position for LSB of the general call bit in the init byte.
/****************************************************************************
 TWI State codes
 ****************************************************************************/
// General TWI Master staus codes
#define TWI_START

0x08  // START has been transmitted 
#define TWI_REP_START
0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST

0x38  // Arbitration lost
// TWI Master Transmitter staus codes
#define TWI_MTX_ADR_ACK
0x18  // SLA+W has been tramsmitted and ACK received
#define TWI_MTX_ADR_NACK
0x20  // SLA+W has been tramsmitted and NACK received
#define TWI_MTX_DATA_ACK
0x28  // Data byte has been tramsmitted and ACK received
#define TWI_MTX_DATA_NACK
0x30  // Data byte has been tramsmitted and NACK received
// TWI Master Receiver staus codes 
#define TWI_MRX_ADR_ACK
0x40  // SLA+R has been tramsmitted and ACK received
#define TWI_MRX_ADR_NACK
0x48  // SLA+R has been tramsmitted and NACK received
#define TWI_MRX_DATA_ACK
0x50  // Data byte has been received and ACK tramsmitted
#define TWI_MRX_DATA_NACK
0x58  // Data byte has been received and NACK tramsmitted
// TWI Slave Transmitter staus codes
#define TWI_STX_ADR_ACK
0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK
0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK
0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
// TWI Slave Receiver staus codes
#define TWI_SRX_ADR_ACK
0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK
0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK
0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK
0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK
0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK
0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART
0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave
// TWI Miscellaneous status codes
#define TWI_NO_STATE

0xF8  // No relevant state information available; TWINT = “0”
#define TWI_BUS_ERROR
0x00  // Bus error due to an illegal START or STOP condition

/****************************************************************************
 Ereignis Typen definition
 ****************************************************************************/
/****************************************************************************
 * Zusammenfassung:
 *
 Ereignis Typ für das senden von Daten zum I2C-Master.
 * Rückgabe:
 *
 Liefert das Daten Byte welches zum I2C-Master übertragen werden soll.
 ****************************************************************************/
typedef unsigned char (*TransmitterEvent)(void);
/****************************************************************************
 * Zusammenfassung:
 *
 Ereignis Typ für das empfangen eines Daten Bytes vom I2C-Master.
 * Parameter(ReceiveData):
 *
 Liefert das Daten Byte welches vom I2C-Master empfangen worden ist.
 ****************************************************************************/
typedef void (*TransceiverEvent)(unsigned char ReceiveData);

extern TransmitterEvent OnTransmittingData;
extern TransceiverEvent OnReceivingData;

/****************************************************************************
 Function definitions
 ****************************************************************************/
void __no_operation(void);
void TWI_Slave_Initialise(unsigned char TWI_ownAddress);
unsigned char TWI_Transceiver_Busy(void);
void TWI_Start_Transceiver(void);
unsigned char TWI_ConnectionState(void);

TWI_Slave.c
Code:
#include <avr/io.h>
#include <avr/signal.h>
#include "TWI_slave.h"
TransmitterEvent OnTransmittingData = NULL;
TransceiverEvent OnReceivingData = NULL;
static unsigned char TWI_Conn_State = TWI_CS_NONE;
/****************************************************************************
 Einen Taktzyklus lang nichts tun Nop
 ****************************************************************************/
void __no_operation()
{
	asm volatile ("nop");
}

/****************************************************************************
 Call this function to set up the TWI slave to its initial standby state.
 Remember to enable interrupts from the main application after initializing the TWI.
 Pass both the slave address and the requrements for triggering on a general call in the
 same byte. Use e.g. this notation when calling this function:
 TWI_Slave_Initialise( (TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) );
 The TWI module is configured to NACK on any requests. Use a TWI_Start_Transceiver function to
 start the TWI.
 ****************************************************************************/
void TWI_Slave_Initialise(unsigned char TWI_ownAddress)
{
	TWAR = TWI_ownAddress;
	
	// Set own TWI slave address. Accept TWI General Calls.
	TWDR = 0xFF;
	// Default content = SDA released.
	//TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);
	
	// Enable TWI-interface and release TWI pins.
	// Disable TWI Interupt.
	// Do not ACK on any requests, yet.
	TWCR = (1<<TWEN)|
	
	
	(0<<TWIE)|(0<<TWINT)|
	
	(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
	(0<<TWWC);
}

/****************************************************************************
 Call this function to test if the TWI_ISR is busy transmitting.
 ****************************************************************************/
unsigned char TWI_Transceiver_Busy()
{
	return (TWCR & (1<<TWIE));  // IF TWI interrupt is enabled then the Transceiver is busy
}

/****************************************************************************
 Call this function to start the Transceiver without specifing new transmission data. Usefull for restarting
 a transmission, or just starting the transceiver for reception. The driver will reuse the data previously put
 in the transceiver buffers. The function will hold execution (loop) until the TWI_ISR has completed with the
 previous operation, then initialize the next operation and return.
 ****************************************************************************/
void TWI_Start_Transceiver()
{
	// Wait until TWI is ready for next transmission.
	while(TWI_Transceiver_Busy())
	{
		__no_operation();
	}
	
	//TWI_statusReg.all = 0;
	
	//TWI_state = TWI_NO_STATE ;
	TWCR = (1<<TWEN)|
	
	// TWI Interface enabled.
	
	(1<<TWIE)|(1<<TWINT)|
	// Enable TWI Interupt and clear the flag.
	
	(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|  // Prepare to ACK next time the Slave is addressed.
	
	(0<<TWWC);
}
unsigned char TWI_ConnectionState()
{
	return TWI_Conn_State;
}

// ********** Interrupt Handlers ********** //
/****************************************************************************
 This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is triggered;
 that is whenever a TWI event has occurred. This function should not be called directly from the main
 application.
 ****************************************************************************/
SIGNAL(SIG_2WIRE_SERIAL)
{
	static unsigned char TWI_bufPtr;
	switch(TWSR)
	{
			// Eigene SLA+R empfangen. ACK wurde zurückgegeben
		case TWI_STX_ADR_ACK:
		{
			
			TWI_Conn_State = TWI_CS_READ;
			
		}
			
			/***************************************
			 * Daten Senden
			 ***************************************/
			
			// Daten Byte wurde ins TWDR Register übertragen. ACK wurde empfangen.
		case TWI_STX_DATA_ACK:
		{
			if(OnTransmittingData != NULL)
				
			{
				
				PORTB |= 0x01;
				
				TWDR = OnTransmittingData();
				
			}
			TWCR = (1<<TWEN)|
			
			// TWI Interface enabled
			
			(1<<TWIE)|(1<<TWINT)|
			
			// Enable TWI Interupt and clear the flag to send byte
			
			(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
			(0<<TWWC);
			
			
			break;
		}
			// Das letzte Daten Byte wurde ins TWDR Register übertragen. NACK wurde empfangen.
		case TWI_STX_DATA_NACK:
			
		{
			
			TWI_Conn_State = TWI_CS_NONE;
			PORTB |= 0x02;
			
			/*
			 
			 if(OnTransmittingData != NULL)
			 
			 {
			 TWDR = OnTransmittingData();
			 
			 }*/
			// Put TWI Transceiver in passive mode.
			
			TWCR = (1<<TWEN)|
			
			// Enable TWI-interface and release TWI pins
			
			(0<<TWIE)|(0<<TWINT)|
			
			// Disable Interupt
			
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
			// Do not acknowledge on any new requests.
			
			(0<<TWWC);
			
			break;
			
		}
			
			// Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
			// case TWI_SRX_GEN_ACK_M_ARB_LOST:
			
			// General call Adresse(0x00) wurde empfangen und ACK zurückgegeben
		case TWI_SRX_GEN_ACK:
			
		{
			
			TWI_Conn_State = TWI_CS_GENERAL;
			
			//TWI_statusReg.genAddressCall = TRUE;
		}
			
			// Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
			
			// case TWI_SRX_ADR_ACK_M_ARB_LOST:
			
			// Eigene SLA+W empfangen und ACK zurückgegeben
		case TWI_SRX_ADR_ACK:
			
		{
			TWI_Conn_State = TWI_CS_WRITE;
			
			// Reset the TWI Interupt to wait for a new event.
			
			TWCR = (1<<TWEN) |
			// TWI Interface enabled
			
			(1<<TWIE) | (1<<TWINT) |
			
			// Enable TWI Interupt and clear the flag to send byte
			
			(1<<TWEA) | (0<<TWSTA) | (0<<TWSTO) |  // Expect ACK on this transmission
			
			(0<<TWWC);
			
			break;
		}
			
			// Previously addressed with own SLA+W; data has been received; ACK has been returned
		case TWI_SRX_ADR_DATA_ACK:
			
			// Previously addressed with general call; data has been received; ACK has been returned
		case TWI_SRX_GEN_DATA_ACK:
		{
			
			//TWI_buf[TWI_bufPtr++]
			= TWDR;
			
			//TWI_statusReg.lastTransOK = TRUE;
			
			// Set flag transmission successfull.
			if(OnReceivingData != NULL)
				
			{
				OnReceivingData(TWDR);
				
			}
			// Reset the TWI Interupt to wait for a new event.
			
			TWCR = (1<<TWEN)|
			// TWI Interface enabled
			
			(1<<TWIE)|(1<<TWINT)|
			
			// Enable TWI Interupt and clear the flag to send byte
			
			(1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
			// Send ACK after next reception
			
			(0<<TWWC);
			
			
			break;
		}
			
			// A STOP condition or repeated START condition has been received while still addressed as Slave
			
		case TWI_SRX_STOP_RESTART:
		{
			
			//TWI_Conn_State = TWI_CS_NONE;
			
			PORTB |= 0x04;
			
			// Put TWI Transceiver in passive mode.
			
			TWCR = (1<<TWEN)|
			
			// Enable TWI-interface and release TWI pins
			
			(0<<TWIE)|(0<<TWINT)|
			
			// Disable Interupt
			
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
			// Do not acknowledge on any new requests.
			
			(0<<TWWC);
			break;
			
		}
			
			// Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
		case TWI_SRX_ADR_DATA_NACK:
			
			// Previously addressed with general call; data has been received; NOT ACK has been returned
		case TWI_SRX_GEN_DATA_NACK:
			// Last data byte in TWDR has been transmitted (TWEA = “0”); ACK has been received
		case TWI_STX_DATA_ACK_LAST_BYTE:
			
			// No relevant state information available; TWINT = “0”
			// case TWI_NO_STATE
			// Bus error due to an illegal START or STOP condition
		case TWI_BUS_ERROR:
		default:
			
		{
			
			TWI_Conn_State = TWI_CS_NONE;
			//TWI_state = TWSR;
			// Store TWI State as errormessage, operation also clears the Success bit.
			
			TWCR = (1<<TWEN)|
			// Enable TWI-interface and release TWI pins
			
			(0<<TWIE)|(0<<TWINT)|
			
			// Disable Interupt
			
			(0<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|
			// Do not acknowledge on any new requests.
			
			(0<<TWWC);
		}
	}
}

Main.c
Code:
#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
#include "TWI_slave.h"
//#define NULL 0
#define bit_is_clear(sfr, bit) (!(sfr & _BV(bit)))
#define bit_is_set(sfr, bit) (sfr & _BV(bit))
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
/****************************************************************************************
 Konstanten
 *****************************************************************************************/
#define TWI_SLAVE_ADDRESS 0x10 // Eigene TWI-slave Adresse
// Befehls Konstanten
#define CMD_UNKOWN 0x00
#define CMD_GET_IR01 0x01
#define CMD_GET_IR02 0x02
#define CMD_GET_IR03 0x03
#define CMD_GET_IR04 0x04
#define CMD_GET_IR05 0x05

/****************************************************************************************
 Funktions Prototypen
 *****************************************************************************************/
void DelayMS(unsigned int msec);
unsigned char DoTransmittingData(void);
void DoReceivingData(unsigned char ReceiveData);
// Aktueller befehl welcher zurück gegeben werden soll
static unsigned char CurrentCmd = CMD_UNKOWN;
// Register
static unsigned char reg_IR01 = 10;
static unsigned char reg_IR02 = 20;
static unsigned char reg_IR03 = 30;
static unsigned char reg_IR04 = 40;
static unsigned char reg_IR05 = 50;
int main()
{
	
	OnTransmittingData = DoTransmittingData;
	OnReceivingData = DoReceivingData;
	// PotrtB als Ausgang definieren
	DDRB = 0xFF;// Debug
	PORTB = 0x00;
	// Debug
	
	// Initialisierung des TWI Moduls für slave Operationen.
	TWI_Slave_Initialise((TWI_SLAVE_ADDRESS<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT));
	
	// Interups aktivieren
	sei();
	
	while(1)
	{
		// Wenn der TWI-Empfänger nicht aktiviert ist diesen erneut starten
		
		if(!TWI_Transceiver_Busy())
		{
			
			// TWI Empfänger starten und auf den ersten Befehl des I2C-Master warten
			
			TWI_Start_Transceiver();
		}
		
		switch(TWI_ConnectionState())
		{
				
			case TWI_CS_READ:
				
			{
				//sbi(PORTB, 0);
				
				//PORTB = 0x01;
				__no_operation();
				
				break;
				
			}
				
			case TWI_CS_GENERAL:
				
			{
				
				//sbi(PORTB, 1);
				
				//PORTB = 0x02;
				
				break;
				
			}
			case TWI_CS_WRITE:
				
			{
				
				//sbi(PORTB, 2);
				
				//PORTB = 0x04;
				
				__no_operation();
				
				break;
				
			}
			default:
				
			{
				
				//PORTB = 0x00;
				
				__no_operation();
				
			}
		}
	}
	return 0;
}
void DelayMS(unsigned int msec)
{
	
	while(msec--)
	{
		_delay_ms(1);
	}
}
unsigned char DoTransmittingData()
{
	unsigned char Result = 0x00;
	
	switch(CurrentCmd)
	{
		case CMD_GET_IR01: Result = reg_IR01; break;
		case CMD_GET_IR02: Result = reg_IR02; break;
		case CMD_GET_IR03: Result = reg_IR03; break;
		case CMD_GET_IR04: Result = reg_IR04; break;
		case CMD_GET_IR05: Result = reg_IR05; break;
	}
	return Result;
}
void DoReceivingData(unsigned char ReceiveData)
{
	CurrentCmd = ReceiveData;
}

Robo2004 - 03.07.2005, 18:29
Titel:
Problem gelöst war nur ein Logisches Problem in Main.c

Code:
// Befehl zum Slave senden
twi_Start(AT_SLA + TWI_WRITE);
twi_WriteData(i);
// Hier der Fehler
twi_Stop();

Richtig
Code:
// Befehl zum Slave senden
twi_Start(AT_SLA + TWI_WRITE);
twi_WriteData(Commands[i]);
twi_Stop();
Alle Zeiten sind GMT + 1 Stunde
Ausdruck eines Beitrages vom Roboternetz - www.Roboternetz.de
Dem großen Internet Portal für Elektronik, Mikrocontroller und Robotik
Hier findet man Bauanleitungen, Schaltpläne, Software, Tips und jede Menge Hilfe

