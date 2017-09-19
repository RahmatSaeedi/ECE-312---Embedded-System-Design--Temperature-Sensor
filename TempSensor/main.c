//////////////////////////////////////////////////////////////////////////
// By Rahmat Saeedi                                                     //
//    Feb 3, 2017                                                       //
// This is written for Atmel-AVR ATmega328P MCU to measure the          //
// temperature using a diode.                                           //
//////////////////////////////////////////////////////////////////////////
/************************************************************************/
/*             Conversion Model                                         */
/************************************************************************/
#define FIXED_POINT_SHIFT		6
#define SLOPE_DIVISION			12
#define LM35_m					445
#define LM35_b					2
#define DIODE_m					-1799
#define DIODE_b					208
#define POINTS_TO_AVERAGE		(1<<FIXED_POINT_SHIFT)
#define DECIMAL_PART_MASK		(0xFFFF>>(16-FIXED_POINT_SHIFT))

/************************************************************************/
/*             FONTS                                                    */
/************************************************************************/
#define	DEGREE_SYMBOL			223

/************************************************************************/
/*             Includes                                                 */
/************************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include "lcd.h"



//Setting up the output stream
int lcd_putchar(char c, FILE *unused)
{
	(void) unused;
	lcd_putc(c);
	return 0;
}
FILE lcd_stream = FDEV_SETUP_STREAM(lcd_putchar, NULL, _FDEV_SETUP_WRITE);


int main(void)
{
	//Initializing LCD and  setting up output stream
	lcd_init(LCD_DISP_ON);
	stdout = &lcd_stream;
	//Power Reduction
	PRR |= 1<<PRTWI | 1<<PRTIM2 | 1<<PRTIM1 | 1<<PRTIM0 | 1<<PRSPI | 1<<PRUSART0;
	
	
	//Setting up ADC
	//Enabling ADC
	ADCSRA |=1<<ADEN;	
	//Voltage Ref: internal 1.1V
	ADMUX |= 1<<REFS0 | 1<<REFS1;
	//Right Adjusted
	ADMUX &=~(1<<ADLAR);
	//Selecting ADC0
	ADMUX &=~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
	//No auto triggering enabled
	//No interrupts enabled
	//Setting ADC clock Pre-scaler N=64
	ADCSRA |=1<<ADPS2 | 1<<ADPS1 ;//| 1<<ADPS2;
	//Using Free Running Mode
	//ADCSRB &= ~(1<<ADTS0 | 1<<ADTS1 | 1<<ADTS2);
	
	//Turning off Digital Input on ADC1..0 pins
	DIDR0 |= 1<<ADC0D | 1<<ADC1D;
	//AD Start first Conversion
	ADCSRA |=1<<ADSC;
	//Wait for first conversion to finish....discarded value
	while(ADCSRA & (1<<ADSC)){}
	ADCSRA |=1<<ADIF;
		
	uint16_t _LM35ADCV, _DiodeADCV;
	int32_t _LM35Temp, _DiodeTemp;
	uint8_t _Decimal;
	uint8_t i;
	

    while (1) 
    {
		lcd_clrscr();
		
		//Read LM35 voltage value and Average
		i=0;
		_LM35Temp=0;
		ADMUX  |=1<<MUX0;
		do {

			ADCSRA |=1<<ADSC;
			while(!(ADCSRA & (1<<ADIF))){}
			ADCSRA |=1<<ADIF;
			_LM35Temp+=ADC;
			i++;		
		} while (i<POINTS_TO_AVERAGE);
		ADMUX  &=~(1<<MUX0);
		_LM35ADCV = (_LM35Temp>>FIXED_POINT_SHIFT);

		//Read Diode voltage value and Average
		i=0;
		_DiodeTemp=0;
		do{
			ADCSRA |=1<<ADSC;
			while(!(ADCSRA & (1<<ADIF))){}
			ADCSRA |=1<<ADIF;
			_DiodeTemp+=ADC;
			i++;	
		} while (i<POINTS_TO_AVERAGE);
		_DiodeADCV = (_DiodeTemp>>FIXED_POINT_SHIFT);
		
		//Convert the voltage to degree C values using the model:
		_LM35Temp*=LM35_m;
		_LM35Temp=_LM35Temp>>SLOPE_DIVISION;
		_LM35Temp +=(LM35_b<<FIXED_POINT_SHIFT);
		
		_DiodeTemp*=DIODE_m;
		_DiodeTemp=_DiodeTemp>>SLOPE_DIVISION;
		_DiodeTemp +=(DIODE_b<<FIXED_POINT_SHIFT);
		
		
		//Output ERROR on temp over ~86C degrees
		if(_LM35ADCV>800){
				printf("ERROR\nTemp. Too Hot\n");	
		}else{
		//Display the LM35 results
			_Decimal=(uint8_t)(_LM35Temp  & DECIMAL_PART_MASK);
			if(_Decimal<61){
				printf("LM35: %3u %2u.%u%cC\n",
					_LM35ADCV,
					(uint16_t)(_LM35Temp>>FIXED_POINT_SHIFT),
					(uint16_t)((_LM35Temp & DECIMAL_PART_MASK)*10+32)>>FIXED_POINT_SHIFT,
					DEGREE_SYMBOL);
			}else{
				printf("LM35: %3u %2u.%u%cC\n",
					_LM35ADCV,
					(uint16_t)(_LM35Temp>>FIXED_POINT_SHIFT)+1,
					0,
					DEGREE_SYMBOL);
			}

			//Display the Diode results
			_Decimal=(uint8_t)(_DiodeTemp  & DECIMAL_PART_MASK);
			if(_Decimal<61){
				printf("Diode:%3u %2u.%u%cC\n",
					_DiodeADCV,
					(uint16_t)(_DiodeTemp>>FIXED_POINT_SHIFT),
					(uint16_t)((_DiodeTemp & DECIMAL_PART_MASK)*10+32)>>FIXED_POINT_SHIFT,
					DEGREE_SYMBOL);
			}else{
				printf("Diode:%3u %2u.%u%cC\n",
					_DiodeADCV,
					(uint16_t)(_DiodeTemp>>FIXED_POINT_SHIFT)+1,
					0,
					DEGREE_SYMBOL);
			}			
		}
		_delay_ms(200);	
    }

}

