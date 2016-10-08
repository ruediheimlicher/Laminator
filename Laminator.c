//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//


#include <stdlib.h>

//#include <avr/io.h>

#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/sleep.h>
#include <inttypes.h>
//#include <avr/wdt.h>

//#include "lcd.c"

#include "adc.c"
#include "defines.h"

volatile    	uint16_t loopcount0=0;
volatile       uint16_t loopcount1=0;

volatile    uint16_t timercount0=0;
volatile    uint16_t timercount1=0;
volatile    uint16_t adccount0=0;
volatile    uint8_t blinkcount=0;

volatile    uint8_t pwmpos=0;
volatile    uint8_t sollwert=0;
volatile    uint8_t istwert=0;
volatile    uint8_t lastwert=0;
volatile    int16_t fehler=0;
volatile    int16_t lastfehler=0;
volatile    int16_t fehlersumme=0;

volatile    double stellwert=200.0;
volatile    uint8_t status=0;




void delay_ms(unsigned int ms);
/*
void r_itoa16(int16_t zahl, char* string)
{
   uint8_t i;
   int16_t original=zahl;
   string[7]='\0';                  // String Terminator
   if( zahl < 0 ) {                  // ist die Zahl negativ?
      string[0] = '-';
      zahl = -zahl;
   }
   else string[0] = ' ';             // Zahl ist positiv
   
   for(i=6; i>=1; i--)
   {
      string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   if (abs(original) < 1000)
   {
      string[1]= ' ';
   }
  if (abs(original) < 100)
   {
      string[2]= ' ';
   }
   if (abs(original) < 10)
   {
      string[3]= ' ';
   }

}
void r_itoa12(int16_t zahl, char* string)
{
   uint8_t i;
   int16_t original=zahl;
   string[5]='\0';                  // String Terminator
   if( zahl < 0 ) {                  // ist die Zahl negativ?
      string[0] = '-';
      zahl = -zahl;
   }
   else string[0] = ' ';             // Zahl ist positiv
   
   for(i=4; i>=1; i--)
   {
      string[i]=(zahl % 10) +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      zahl /= 10;
   }
   
   if (abs(original) < 1000)
   {
      string[1]= ' ';
   }
   if (abs(original) < 100)
   {
      string[2]= ' ';
   }
   if (abs(original) < 10)
   {
      string[3]= ' ';
   }
   
}


void r_itoa8(int8_t zahl, char* string)
{
   uint8_t i;
   int8_t original=zahl;
   string[4] = '\0';                  // String Terminator
   if( zahl < 0 )
   {                  // ist die Zahl negativ?
      string[0] = '-';
      zahl = -zahl;
   }
   else
   {
      string[0] = ' ';             // Zahl ist positiv
   }
   for(i = 3; i >= 1; i--)
   {
      uint8_t temp = zahl % 10;
      string[i] = temp +'0';     // Modulo rechnen, dann den ASCII-Code von '0' addieren
      
      zahl /= 10;
   }
   if (abs(original) < 100)
   {
      string[1]= ' ';
   }
   if (abs(original) < 10)
   {
      string[2]= ' ';
   }
}
*/

void slaveinit(void)
{
 	OUTDDR |= (1<<PWM_OUT_PIN);		//Pin 0 von PORT D als Ausgang fuer PWM
	OUTPORT |= (1<<PWM_OUT_PIN);		//HI
	
   OSZIDDR |= (1<<OSZIA);		//Pin 1 von PORT D als Ausgang fuer OSZI
 	OSZIPORT |= (1<<OSZIA);		//HI
   //DDRD |= (1<<DDD4);		//Pin 4 von PORT D als Ausgang fuer LED
   LOOPLEDDDR |= (1<<LOOPLED_PIN);		//Pin 4 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED_PIN);		//Pin 4 von PORT D als Ausgang fuer LED Loop

   LOOPLEDDDR |= (1<<TOPLED_PIN);		//Pin 5 von PORT D als Ausgang fuer LED Heizung

   
	
	
/*
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
*/
//	DDRC &= ~(1<<DDC0);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
   
   ADCDDR &= ~(1<<ADC_SOLL_PIN);	//Pin 1 von PORT C als Eingang fuer ADC soll-Wert
	ADCDDR &= ~(1<<ADC_IST_PIN);	//Pin 2 von PORT C als Eingang fuer ADC ist-Wert
   
   
//	ADCPORT |= (1<<ADCPIN); //Pull-up
//	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC3); //Pull-up
//	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur
//	PORTC |= (1<<DDC3); //Pull-up


	
	
}



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

#pragma mark Takt
void timer0(void) //Takt der Messung
{
	//----------------------------------------------------
	// Set up timer 0
	//----------------------------------------------------
   /*
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS00) | _BV(CS02);
    OCR0A = 0x2;
    TIMSK0 = _BV(OCIE0A);
    */
   
//   DDRD |= (1<< PORTD6);   // OC0A Output
   /*
   TCCR0A |= (1<<WGM00);   // fast PWM  top = 0xff
   TCCR0A |= (1<<WGM01);   // PWM
   //TCCR0A |= (1<<WGM02);   // PWM
   
   TCCR0A |= (1<<COM0A1);   // set OC0A at bottom, clear OC0A on compare match
   TCCR0B |= 1<<CS02;
   TCCR0B |= 1<<CS00;
   
   OCR0A=10;
   TIMSK0 |= (1<<OCIE0A);
   */
   
   TCCR0B |= (1<<CS00)|(1<<CS01);	//Takt /64
   
   TCCR0A |= (1<<WGM01);   // Enable  compare match
   
   
  // TCCR0B |= (1<<CS02);	//Takt /256 Intervall
   
   //TIMSK |= (1<<TOIE0);			//Overflow Interrupt aktivieren
    TIMSK |= (1<<OCIE0A);			//Overflow Interrupt aktivieren
   
   TCNT0 = 0x00;					//RŸcksetzen des Timers
   
   OCR0A = TIMER2_COMPA; // Compare match A
   TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts

}


ISR(TIMER0_COMPA_vect)
{
   //LOOPLEDPORT ^= (1<<LOOPLED_PIN);
   timercount0++;
   if (timercount0 > 2) // Takt teilen, 1s
   {
      //OSZITOGG;
      //LOOPLEDPORT ^= (1<<LOOPLED_PIN);
      timercount0=0;
      
      timercount1++;
      uint8_t sw = (uint8_t)stellwert;
      uint8_t diff = (timercount1 - sw);
      
      // if ((timercount1 - (uint16_t)stellwert)>=0 && ((timercount1 - (uint16_t)stellwert) < 2))
      //if ((timercount1 >= (uint8_t)stellwert)  )
      if ((diff >=0) && (diff < 2) && sw)
      {
         //OSZITOGG;
         //OSZILO;
         LOOPLEDPORT |= (1<<LOOPLED_PIN);
         OUTPORT &= ~(1<<PWM_OUT_PIN); // Triac off
         // status |= (1<<PWM_ADC);// ADC messen ausloesen
      }
      if (timercount1 == TIMER2_PWM_INTERVALL)
      {
         //OSZITOGG;
         //OSZIHI;
         timercount1 = 0;
         status |= (1<<PWM_ADC);// ADC messen ausloesen
         LOOPLEDPORT &= ~(1<<LOOPLED_PIN);
         if (((uint8_t)stellwert>1) )
         {
            
            OUTPORT |= (1<<PWM_OUT_PIN); // Triac on
         }
      }
      
   }
   
}

ISR(TIMER0_OVF_vect)
{
   
   //LOOPLEDPORT |=(1<<LOOPLED);
}
/*
ISR(TIMER0_OVF_vect)
{
   adccount0++;
   if (adccount0 > 0x78) // Teiler /256
   //if (adccount0 > 0x480) // Teiler /8: *4
   {
      adccount0=0;
      blinkcount++;
      
      //OSZITOG;
      status |= (1<<PWM_ADC);// ADC

      
   }
   
   timercount0++;
   
   if (timercount0 > 0x0004) // Teiler /8
   {
      timercount0=0;
 //     OSZITOG; // 31 Hz
      timercount1++;
      
      if (timercount1==0x200)
      {
         //LOOPLEDPORT ^=(1<<TOPLED);
         status |= (1<<PWM_ON);
         OUTPORT |= (1<<PWM_OUT_PIN);
         timercount1=0;
      }
      
      else if (fehler > 30) // schneller aufheizen
       {
         if (timercount1 > 2*(uint8_t)stellwert)
         {
             
             OUTPORT &= ~(1<<PWM_OUT_PIN);
         }
       }
      else if (timercount1 > (uint8_t)stellwert)
      {
         
         OUTPORT &= ~(1<<PWM_OUT_PIN);
         //status |= (1<<PWM_ON);// ADC
      }
      
   }
   
}
*/
#pragma mark // Timer1 PWM
// Timer1 fuer PWM
void timer1(void)
{
  // TCCR0A = 0;  // normal mode
 //  TCCR0B = 0;
   TCCR1 = 0;                  //stop the timer
   TCNT1 = 0;                  //zero the timer
   GTCCR = _BV(PSR1);          //reset the prescaler
   OCR1A = 100;                //set the compare value
   OCR1C = 255;
   TIMSK |=(1<<OCIE1A);        //interrupt on Compare Match A
   TIMSK |=(1<<TOIE1);
   //start timer, ctc mode, prescaler clk/16384
  // TCCR1 |= (1 << CTC1);
   TCCR1 |= (1<<CS10   | 1<<CS12 | 1<<CS11 );
   sei();

}

// Timer1 fuer PWM: Interrupt bei OCR1A, in
ISR(TIMER1_COMPA_vect)
{
   //comment out one of the two lines below
   //digitalWrite(4, LOW);       //turn the LED off
   //PINB |= _BV(PINB4);         //flash the LED by toggling PB4
   //LOOPLEDPORT &= ~(1<<LOOPLED);
   OUTPORT &= ~(1<<TOPLED_PIN); // LED off off

}

ISR(TIMER1_OVF_vect)
{
  // LOOPLEDPORT |= (1<<LOOPLED);
 //  if (stellwert>10)
   {
      OUTPORT |= (1<<TOPLED_PIN); // LED on
   }
}


int main (void)
{
   MCUSR = 0;
	//wdt_disable();

	slaveinit();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	
	//uint16_t ADC_Wert= readKanal(0);
		
	/* initialize the LCD */
//	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

//	lcd_puts("Guten Tag\0");
//	delay_ms(1000);
//	lcd_cls();
//	lcd_puts("READY\0");
	
	
	//initADC(TASTATURPIN);
	
	//uint16_t startdelay1=0;

	//uint8_t twierrcount=0;
	//LOOPLEDPORT |=(1<<LOOPLED);
	
	//delay_ms(800);

//	lcd_clr_line(0);
   timer0();
   timer1();
   initADC(1);
//   sollwert = readKanal(2)>>2;
   sei();
	while (1)
   {
      //Blinkanzeige
      loopcount0++;
      if (loopcount0 >= LOOPSTEP)
      {
         sollwert = readKanal(ADC_SOLL_KANAL)>>2;
         istwert = readKanal(ADC_IST_KANAL)>>2;
         //sollwert = 88;
         OCR1A = istwert;

         loopcount0=0;
         // LOOPLEDPORT ^=(1<<LOOPLED);
         loopcount1++;
         //OSZITOG;
         if (loopcount1 >= LOOPSTEP)
         {
            
            loopcount1 = 0;
          //  LOOPLEDPORT ^=(1<<LOOPLED);
            /*
            if (sollwert > istwert)
            {
               LOOPLEDPORT ^=(1<<TOPLED_PIN); // TEMPERATUR NOCH ZU KLEIN
            }
            else
            {
               LOOPLEDPORT |=(1<<TOPLED_PIN);
            }
             */
         }
      }
      
      
      if (status & (1<<PWM_ADC)) // soll- und ist-werte lesen, PI aktualisieren
      {
         //OSZILO;
         status &= ~(1<<PWM_ADC);
         // in 328 Kanaele vertauscht
  //       sollwert = readKanal(2)>>2;
  //       istwert = readKanal(1)>>2;

#pragma mark PID
         if (istwert < sollwert/4*3)
         {
            status |= (1<<PID_FIRST_RUN); // K Prop ist beim ersten Aufheizen kleiner
            
         }

        
         fehler = sollwert - istwert; // Fehler positiv wenn temp zu klein
         
         fehlersumme += fehler;
         
         if (fehlersumme < K_WINDUP_LO)
         {
            fehlersumme = K_WINDUP_LO;
         }
         if (fehlersumme > K_WINDUP_HI)
         {
            fehlersumme = K_WINDUP_HI;
         }
         
         
         if (fehlersumme>=0)
         {
//            lcd_putc(' '); // Platz des Minuszeichens
         }
         
#pragma mark stellwert
 
         
         float k_prop = K_PROP_HI;
         if (status & (1<<PID_FIRST_RUN))
         {
            k_prop = K_PROP_LO;
            
            if (istwert > sollwert) // zuruecksetzen wenn soll erreicht
            {
               status &= ~(1<<PID_FIRST_RUN);
               k_prop = K_PROP_HI;
            }
         }
         
         
         stellwert = k_prop * fehler + K_INT * K_DELTA * fehlersumme  + K_DIFF*((fehler-lastfehler) /K_DELTA);

         stellwert *= PWM_FAKTOR;
         //stellwert = 83;
         
         lastfehler = fehler;
         
         
         if (stellwert < 0)
         {
            stellwert = 0;
            //OUTPORT &= ~(1<<PWM_OUT_PIN); // Triac off
         }
         if (stellwert == 0)
         {
            OUTPORT &= ~(1<<PWM_OUT_PIN); // Triac sicher off
         }
         
         if (stellwert > 254)
         {
            stellwert = 254;
         }
         
   //      OCR1A = (uint16_t)stellwert;
  
 
      
      }
      
      
      
      
      
   
      //	LOOPLEDPORT &= ~(1<<LOOPLED);
   }//while


 return 0;
}
