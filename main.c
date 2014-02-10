//--------------------------------------------------------------
// File     : main.c
// Datum    : 20.07.2013
// Version  : 1.0
// Autor    : UB
// EMail    : mc-4u(@)t-online.de
// Web      : www.mikrocontroller-4u.de
// CPU      : STM32F4
// IDE      : CooCox CoIDE 1.7.0
// Module   : CMSIS_BOOT, M4_CMSIS_CORE
// Funktion : Demo der InputCapture PWM-Library
// Hinweis  : Diese zwei Files muessen auf 8MHz stehen
//              "cmsis_boot/stm32f4xx.h"
//              "cmsis_boot/system_stm32f4xx.c"
//--------------------------------------------------------------

#include "stm32_ub_icpwm_tim3.h"
#include <stdio.h>

uint8_t get_dip_setting ( uint8_t code )
{
	switch ( code )
	{
		case 0x00: return ( 0x03 );
		case 0x04: return ( 0x01 );
		case 0x01: return ( 0x02 );
		case 0x05: return ( 0x00 );
	}
	return ( 0xff );
};

uint8_t get_key_code ( uint16_t code )
{
	switch ( code )
	{
	case 0x055:
	case 0xfaa:
		return ( 'A' );
	case 0x015:
	case 0xaea:
		return ( 'B' );
	case 0x045:
	case 0xeba:
		return ( 'C' );
	case 0x051:
	case 0xeae:
		return ( 'D' );
	case 0x154:
	case 0x454:
		return ( 'E' );
	};
	return ( '?' );
};

void print_bin ( uint16_t code, uint8_t buflen )
{
	uint8_t bit;
	for ( bit = 0; bit < buflen; bit++ )
	{
		fputc ( '1' - ( code & 0x01 ), stdout );
		code >>= 1;
	};
};

int main(void)
{
	uint8_t old_datalen = 0;

	SystemInit(); // Quarz Einstellungen aktivieren

	// Init vom Timer2 zur PWM-Messung
	UB_ICPWM_TIM3_Init();

	while ( 1 )
	{
		uint8_t i = 0;
		if ( datalen > old_datalen )
		{
			for ( i = old_datalen; i < datalen; i++ )
			{
				printf ( "%2d: ", i );
				uint8_t dip = get_dip_setting ( data[i] >> 20 );
				print_bin ( dip, 2 );
				dip = get_dip_setting ( ( data[i] >> 16 ) & 0x0f ) & 0x03;
				print_bin ( dip, 2 );
				uint8_t key_byte = ( data[i] >> 4 ) & 0xfff;
				printf ( " %c ", get_key_code ( key_byte ) );

				if ( ( data[i] & 0x0f ) == 0x01 ) fputc ( '1', stdout );
				else if ( ( data[i] & 0x0f ) == 0x04 ) fputc ( '0', stdout );
				else fputc ( '?', stdout );

				printf ( " (%x)\n", data[i] );
			};
			if ( datalen == MAXDATAGRAMS ) datalen = 0;
			old_datalen = datalen;
		};
	};
};
