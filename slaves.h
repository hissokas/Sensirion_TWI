/*
 *  slaves.h
 *  TWI_Master
 *
 *  Created by Sysadmin on 16.03.08.
 *  Copyright 2008 Ruedi Heimlicher. All rights reserved.
 *
 */

uint8_t SlaveSchreiben(unsigned char ADRESSE);
uint8_t SlavedatenSchreiben(const unsigned char ADRESSE, const uint8_t *Daten);
uint8_t SlavedatenLesen(const unsigned char ADRESSE, uint8_t *Daten);
uint8_t SlaveLesen2(unsigned char ADRESSE, uint8_t *Daten);
uint8_t SlaveLesen(unsigned char ADRESSE);

uint8_t TagLesen(unsigned char ADRESSE, uint8_t *Daten, uint8_t Raum, uint8_t Tag);
uint8_t WochentagLesen(unsigned char ADRESSE, uint8_t *Daten, uint8_t Raum, uint8_t Objekt, uint8_t Wochentag);
uint8_t EEPROMTagLesen(unsigned char ADRESSE, uint8_t *Daten, uint8_t hbyte,uint8_t lbyte);
uint8_t WebTagSchreiben(unsigned char ADRESSE, uint8_t Daten[], uint8_t flags);


void WochentagZeigen(const uint8_t Daten[], uint8_t Tag, uint8_t Stunde);
uint8_t Tagplanwert(const uint8_t Daten[], const uint8_t Stunde);
