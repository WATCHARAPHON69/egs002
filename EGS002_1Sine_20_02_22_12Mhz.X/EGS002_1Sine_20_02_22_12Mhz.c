/*
 * File: Use WinAVR Compiler
 * Author: seree2004,HS1YWN
 *
 * Created on 14-02-2022 
 */
#define F_CPU 12000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

//==============================================================================
const unsigned char character1[64] = {
0,18,18,18,18,18,27,0,  		 //      0     ·
0,14,17,29,21,21,25,0,  		 //      1     ¥
0,0,4,4,4,4,6,0,       			 //      2     ‡
0,2,31,1,29,19,25,0,             //      3      
1,15,6,9,14,1,6,0,     			 //      4     √’
24,24,7,8,8,8,7,0,     			 //      5     Õß»“C
0,0,20,23,29,22,23,0,            //      6     Hz
0,0,9,21,18,0,0,0}; 			 //      7     √Ÿª´“¬‡«ø   
//==============================================================================
const unsigned char character2[64] = {
0,0,25,9,21,21,10,0,             // 	  0	æ   
26,12,7,8,15,1,14,0,  			 //      1     √È
0,0,14,1,13,9,14,0,  			 //      2     Õ
0,0,25,9,9,25,30,0,       		 //      3     ¡
1,15,6,9,14,1,6,0,     			 //      4     √’
24,24,7,8,8,8,7,0,     			 //      5     Õß»“C
0,0,20,23,29,22,23,0,            //      6     Hz
0,0,9,21,18,0,0,0};  			 //      7     √Ÿª´“¬‡«ø   
//==============================================================================
// LCD interface (should agree with the diagram above)
//   make sure that the LCD RW pin is connected to GND
#define lcd_D7_port     PORTC                   // lcd C5 connection
#define lcd_D7_bit      PORTC5
#define lcd_D7_ddr      DDRC

#define lcd_D6_port     PORTC                   // lcd C4 connection
#define lcd_D6_bit      PORTC4
#define lcd_D6_ddr      DDRC

#define lcd_D5_port     PORTC                   // lcd C3 connection
#define lcd_D5_bit      PORTC3
#define lcd_D5_ddr      DDRC

#define lcd_D4_port     PORTC                   // lcd C2 connection
#define lcd_D4_bit      PORTC2
#define lcd_D4_ddr      DDRC

#define lcd_E_port      PORTD                   //  D4 cd Enable pin
#define lcd_E_bit       PORTD4
#define lcd_E_ddr       DDRD

#define lcd_RS_port     PORTD                   //  D7 lcd Register Select pin
#define lcd_RS_bit      PORTD7
#define lcd_RS_ddr      DDRD

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position
// Use this define sets for easy set,clear register
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define true 255
#define false 0
//==============================================================================
#define VLAVEL     0        // Analog port 0 for AC Feed Back AC Volted analog input (AD0)
#define THERMISTORPIN 1   //Port Analog 1 for read temperature from thermistor (AD1)
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000 
int samples[NUMSAMPLES];
//==============================================================================
#define MAX_SINE 248			// Maximum Sine PWM DATA
#define MAX_AMPLITUDE 100	//Max Amplitude to draw sine wave
#define MIN_AMPLITUDE 80   	// Min Amplitude to draw sine wave
#define WARMUP        95
#define COMPENSATE  160

#define COUNT         4096
#define DT 6                //Dead Time
//==============================================================================
// Reference for PWM Frequency const double refclk=15686;  // =8Mhz 8000000Hz / 510 = 15686
//const float refclk = 15686  ;
// Reference for PWM Frequency const double refclk=15686;  // =12Mhz 12000000Hz / 510 = 23529
const float refclk = 23529  ;
volatile unsigned long sigma;   // phase accumulator
volatile unsigned long delta;  // phase increment
volatile unsigned int  c4ms;  // counter incremented every 4ms
volatile uint8_t EN_status;
volatile uint8_t T_status;
//==============================================================================
uint8_t Table_Buffer[128];
uint8_t  freq, temp_ocr, tlavel, temp, cout, Volts_Show, Old_Volts, DUTY_CYCLE; 
unsigned int V_Read, N_Volts_Show; 
//==============================================================================
void lcd_write(uint8_t);
void lcd_write_cmd(uint8_t);
void lcd_write_char(char);
void lcd_xy_char(uint8_t, uint8_t, char);
void lcd_write_str(char *);
void lcd_xy_str(uint8_t, uint8_t, char *);
void lcd_write_int(int);
void lcd_xy_int(uint8_t, uint8_t, int);
void lcd_gotoxy(uint8_t, uint8_t);
void lcd_clear(void);
void lcd_init(void);
void adc_init(void);
uint16_t read_adc(uint8_t channel);
void(* resetFunc) (void) = 0;//declare reset function at address 0  RESET System
void setup_timer(void);
void LoadModulateBuffer(unsigned char);
void Lost_AC();
//============================================================================== 
int main(void) {
   DDRB = 0B11111111;	//Set PortB for Output
   DDRC = 0B11111100;	// Set Port Input/Output
   DDRD = 0B11111010;	// Set Port Input/Output
   sbi(PORTD,0); // Pull Up Input PortD0(RX)
   sbi(PORTD,2);  //Pull Up Input PortD2 (External Interrupt INT0) 
//============================================================================== 
   cbi(PORTB,3);    // CCR2A H output (PB3)
   cbi(PORTD,3);   // CCR2B L output (PD3)
   cbi(PORTB,1);   //CCR1A L output (PB1)
   cbi(PORTB,2);   //CCR1B H  output (PB2)
 //==============================================================================  
   OCR1A = 0;  // clear PWM_OUT_H
   OCR1B = 0;  // clear_OUT_L
   OCR2A = 0;  // clear PWM_OUT_H
   OCR2B = 0;  // clear PWM_OUT_L
//============================================================================== 
   cbi(PORTB,4);   //clear BYPASS Relay 
   cbi(PORTB,5);   //clear AC OUT Relay ???
   cbi (PORTD,6);  // Enable IGBT OFF
   sbi(PORTB,0);   //digitalWrite(FAN, High); 
   EN_status = false;
   T_status = false;
 //============================================================================== 
   lcd_init();
   adc_init();
//******************************************************************************     
   lcd_write_cmd(64);    //LCD_DRAWN_THAI_FONT           64
   for (cout = 0; cout<=63; cout++) lcd_write_char(character1[cout]);
//******************************************************************************     
    lcd_xy_str(1, 2, "FILM69");
//******************************************************************************  
   lcd_xy_str(2, 1, "11-06-22");		// Disply Version
   _delay_ms(1000);  
   cbi (TIMSK2,TOIE2);              // disable timer2 overflow detect
   setup_timer();		//set up timer 2 interrupt ready
   lcd_gotoxy(2, 1);
   for (temp=0; temp <= 7; temp++){
      lcd_write_str("*");
      _delay_ms(50);
   }
 //==============================================================================  
   freq = 50;
   LoadModulateBuffer(MIN_AMPLITUDE);   
   delta = pow(2,32)*freq/refclk ;
   sbi (PORTB,4);  //**********BYPASS Relay
   _delay_ms (1000);
   cbi(PORTB,0);  //digitalWrite(FAN, LOW);
//==============================================================================
   sbi (TIMSK2,TOIE2);              //enable timer2 overflow detect
   EICRA |= (1 << ISC00)|(1 << ISC10);    // set INT0 to trigger on       Rising     edge
   EIMSK |= (1 << INT0);     // Turns on INT0
   sei();			//Interrupt Start
 //==============================================================================
   EN_status = true; 
   c4ms = 1;    
   while(c4ms);
   tlavel = 30;
//****************************************************************************** 
   lcd_xy_str(1, 1,"--------");
   lcd_write_cmd(64);    //LCD_DRAWN_THAI_FONT           64
   for (cout = 0; cout<=63; cout++) lcd_write_char(character2[cout]);
//****************************************************************************** 
    V_Read = read_adc(VLAVEL);
    for (temp = MIN_AMPLITUDE; temp <= WARMUP; temp = temp + 5){    //Warm Up Sine Wave Amplitude up to WARMUP Value  
      LoadModulateBuffer(temp); 
      lcd_xy_str(1, 1,"S1-23KHz");      //Display PWM
      lcd_xy_int(2, 1, temp);
      lcd_write_char(7);
      lcd_write_str(" ");
      lcd_xy_int(2, 6, tlavel); 
      lcd_write_char(5);
      lcd_write_str(" ");
      while(c4ms);
    }
    if (read_adc(VLAVEL)<= 32)  Lost_AC();     //IF AC Output < 194V Error
    lcd_xy_char(1, 1, 0);        //æ
    lcd_xy_char(1, 2, 1);     	 //√È
    lcd_xy_char(1, 3, 2);  	 //Õ
    lcd_xy_char(1, 4, 3);       //¡
    lcd_write_str(" 50");
    lcd_write_char(6);
    sbi(PORTB,5);  //************** ON AC OUTPUT Relay ‡¡◊ËÕæ√ÈÕ¡·≈È« 
//==============================================================================
    while(1){ 
       uint8_t i;
       float average;
       // take N samples in a row, with a slight delay
       for (i=0; i< NUMSAMPLES; i++) {
         samples[i] = read_adc(THERMISTORPIN);
         _delay_ms(100);
       }
      // average all the samples out
      average = 0;
      for (i=0; i< NUMSAMPLES; i++) {
         average += samples[i];
      }
      average /= NUMSAMPLES;
      average = 1023 / average - 1;
      average = SERIESRESISTOR / average;
      float steinhart;
      steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
      steinhart = log(steinhart);                  // ln(R/Ro)
      steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
      steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
      steinhart = 1.0 / steinhart;                 // Invert
      steinhart -= 273.15;      // convert to C
      tlavel = (uint8_t) steinhart;
 //============================================================================== 
      V_Read = read_adc(VLAVEL);
      Volts_Show = V_Read >> 3;      // ‡Õ“·§Ë‰¡Ë‡°‘π 127
      N_Volts_Show = COMPENSATE - Volts_Show;       //‡æ◊ËÕ™¥‡™¬°“√§«∫§ÿ¡‚«≈µÏ‰ø AC ¢“ÕÕ° AC Adj.
	  if (N_Volts_Show >= MAX_AMPLITUDE) N_Volts_Show = MAX_AMPLITUDE;
	  if (N_Volts_Show <= MIN_AMPLITUDE) N_Volts_Show = MIN_AMPLITUDE;
      if (Volts_Show != Old_Volts){
        LoadModulateBuffer(N_Volts_Show);
        Old_Volts = Volts_Show;
      }
//==============================================================================      
      lcd_xy_int(2, 1, N_Volts_Show);
      lcd_write_char(7);
      lcd_write_str(" ");
      lcd_xy_int(2, 6, tlavel); 
      lcd_write_char(5);
      lcd_write_str(" ");
      lcd_xy_int(2, 10, Volts_Show);
      if (tlavel >= 42) sbi(PORTB,PORTB0);   //digitalWrite(FAN, High);
      if (tlavel <= 38) cbi(PORTB,PORTB0);   //digitalWrite(FAN, LOW);
      if ( read_adc(VLAVEL)<= 32)  Lost_AC();     //IF AC Output < 194V Error	
    }
}   
//==============================================================================
//==============================================================================
ISR(TIMER2_OVF_vect) {
    // Drawn Sine Wave  
    sigma=sigma+delta; // soft DDS, phase accu with 32 bits
    DUTY_CYCLE=sigma >> 25;     // use upper 8 bits for phase accu as frequency information

	temp_ocr = Table_Buffer[DUTY_CYCLE];

    OCR2A = temp_ocr + DT;  // pin B3
    OCR2B = temp_ocr;  // pin D3 

    switch (DUTY_CYCLE){
    case 0:
      cbi(PORTB,1);   //digitalWrite(Sink1, LOW); // ’‡¢’¬« „π ‚§ª
    break;
    case 1:  
      sbi (PORTB,2);  //digitalWrite(Sink2, HIGH);  // ’·¥ß „π ‚§ª
    break;
    case 65:
      cbi (PORTB,2);  //digitalWrite(Sink2, LOW); // ’·¥ß „π ‚§ª
    break;
    case 66:  
      sbi (PORTB,1);  //digitalWrite(Sink1, HIGH);  // ’‡¢’¬« „π ‚§ª
    break;
   }
  
    if (c4ms++ == COUNT){
                c4ms = 0;    
                T_status = !T_status;
                if  (T_status) sbi (PORTD,5); else cbi (PORTD,5);	// Toggle Led_Status
                if (EN_status){
                    sbi (PORTD,6);  // Enable IGBT ON
                    EN_status = false;
                }  
            }
}
//==============================================================================
//==============================================================================
ISR (INT0_vect){
  /* interrupt code here */
        cbi(PORTD,6);   //digitalWrite(EnablePin, LOW); Stop Power Driver
        cli();
        lcd_xy_str(1, 1, "OverLoad"); //Display Error
        lcd_xy_str(2, 1, "ResetOFF");
        _delay_ms(3000);
        //resetFunc(); //call reset
        while(1);		//Stop System
}
//==============================================================================
void Lost_AC(){
        cbi(PORTD,6);   //OFF EnablePin Stop Power Drive
        lcd_xy_str(1, 1, "Lost AC!"); //Display Error
        lcd_xy_str(2, 1, "ResetOFF");    //Stop 3 ms 
        while (1); //Let 's Switch Off/l reset
}
/*============================== 4-bit LCD Functions ======================*/
void lcd_init(void){
    _delay_ms(100);                                 // initial 40 mSec delay
    lcd_RS_port &= ~(1<<lcd_RS_bit);                // select the Instruction Register (RS low)
    lcd_E_port &= ~(1<<lcd_E_bit);                  // make sure E is initially low
    lcd_write(lcd_FunctionReset);                 // first part of reset sequence
    _delay_ms(10);                                  // 4.1 mS delay (min)
    lcd_write(lcd_FunctionReset);                 // second part of reset sequence
    _delay_us(200);                                 // 100uS delay (min)
    lcd_write(lcd_FunctionReset);                 // third part of reset sequence
    _delay_us(200);                                 // this delay is omitted in the data sheet
    lcd_write(lcd_FunctionSet4bit);               // set 4-bit mode
    _delay_us(100);                                  // 40uS delay (min)
    lcd_write_cmd(lcd_FunctionSet4bit);   // set mode, lines, and font
    _delay_us(100);                                  // 40uS delay (min)
    lcd_write_cmd(lcd_DisplayOff);        // turn display OFF
    _delay_us(100);                                  // 40uS delay (min)
    lcd_write_cmd(lcd_Clear);             // clear display RAM
    _delay_ms(4);                                   // 1.64 mS delay (min)
    lcd_write_cmd(lcd_EntryMode);         // set desired shift characteristics
    _delay_us(100);                                  // 40uS delay (min)
    lcd_write_cmd(lcd_DisplayOn);         // turn the display ON
    _delay_us(100);                                  // 40uS delay (min)
}
//==============================================================================
void lcd_gotoxy(uint8_t y, uint8_t x){
volatile uint8_t locat;
switch(y){
case 1: locat = 127 + x; break;
case 2: locat = 191 + x; break;
case 3: locat = 147 + x; break;
case 4: locat = 211 + x; break;
default: break;}
lcd_write_cmd(locat);
_delay_us(100);
}
//==============================================================================
void lcd_write_str(char theString[]){
    volatile uint8_t i = 0;                             // character counter*/
    while (theString[i] != 0)
    {
        lcd_write_char(theString[i]);
        i++;
        _delay_us(100);                              // 100 uS delay (min)
    }
}
//==============================================================================
void lcd_xy_str(uint8_t xstr, uint8_t ystr, char str_data[]){
lcd_gotoxy(xstr, ystr);
lcd_write_str(str_data);
}
//==============================================================================
void lcd_write_char(char theData){
    sbi(lcd_RS_port,lcd_RS_bit);                 // select the Data Register (RS high)
    cbi(lcd_E_port,lcd_E_bit);                  // make sure E is initially low
    lcd_write(theData);                           // write the upper 4-bits of the data
    lcd_write(theData << 4);                      // write the lower 4-bits of the data
}
//==============================================================================
void lcd_xy_char(uint8_t xc, uint8_t yc, char xyData){
    lcd_gotoxy(xc, yc);
    lcd_write_char(xyData);
}
//==============================================================================
void lcd_write_cmd(uint8_t theInstruction){
    cbi(lcd_RS_port,lcd_RS_bit);                // select the Instruction Register (RS low)
    cbi(lcd_E_port,lcd_E_bit);                  // make sure E is initially low
    lcd_write(theInstruction);                    // write the upper 4-bits of the data
    lcd_write(theInstruction << 4);               // write the lower 4-bits of the data
}
//==============================================================================
void lcd_write(uint8_t theByte){
    lcd_D7_port &= ~(1<<lcd_D7_bit);                        // assume that data is '0'
    if (theByte & 1<<7) lcd_D7_port |= (1<<lcd_D7_bit);     // make data = '1' if necessary
    lcd_D6_port &= ~(1<<lcd_D6_bit);                        // repeat for each data bit
    if (theByte & 1<<6) lcd_D6_port |= (1<<lcd_D6_bit);
    lcd_D5_port &= ~(1<<lcd_D5_bit);
    if (theByte & 1<<5) lcd_D5_port |= (1<<lcd_D5_bit);
    lcd_D4_port &= ~(1<<lcd_D4_bit);
    if (theByte & 1<<4) lcd_D4_port |= (1<<lcd_D4_bit);
    sbi(lcd_E_port,lcd_E_bit);                   // Enable pin high
    _delay_us(500);                              // Enable pulse width' (500 uS)
    cbi(lcd_E_port,lcd_E_bit);                 	// Enable pin low
    _delay_us(500);                              // Enable cycle time' (500 uS)
}
//==============================================================================
void lcd_xy_int(uint8_t xpu,uint8_t ypu,int vals){
    lcd_gotoxy(xpu, ypu);
    lcd_write_int(vals);
}
//==============================================================================
void lcd_write_int(int val){
	char txt[5];
    //sprintf(txt, "%02d", val);
    itoa(val, txt, 10);
    lcd_write_str(txt);
}
//==============================================================================
void lcd_clear(void){
        lcd_write_cmd(lcd_Clear);
        _delay_us(100);
        lcd_write_cmd(lcd_Home);
        _delay_us(100);
}
//==============================================================================
void adc_init(void){
 // Select Vref=AVcc,10 bit
   sbi(ADMUX,REFS0);
   cbi(ADMUX,ADLAR);
 //set prescaller to 8000000/64 = 125 Khz and enable ADC
   ADCSRA = 0B11000110;
}
//==============================================================================
uint16_t read_adc(uint8_t channel){
 //select ADC channel with safety mask
 ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
 //single conversion mode
 sbi(ADCSRA,ADSC);	//ADCSRA |= (1<<ADSC);
 // wait until ADC conversion is complete
 while( ADCSRA & (1<<ADSC) );	//wait till ASC=0
 return ADCW;
}
//==============================================================================
void setup_timer(void){
//==============Timer2========================   
   sbi (TCCR2B, CS20);
   cbi (TCCR2B, CS21);
   cbi (TCCR2B, CS22);
// Timer2 PWM Mode set to Phase Correct PWM
   cbi (TCCR2A, COM2B0); //(OCR2B)Sine PWM LOW Norman +
   sbi (TCCR2A, COM2B1);
   sbi (TCCR2A, COM2A0); //(OCR2A)Sine PWM HIGH Invert -
   sbi (TCCR2A, COM2A1);
   sbi (TCCR2A, WGM20); //(Mode 1 / Phase Correct PWM)
   cbi (TCCR2A, WGM21);
   cbi (TCCR2B, WGM22);
   EICRA |= (1 << ISC00)|(1 << ISC10);    // set INT0 to trigger on Rising edge
}
//==============================================================================
//==============================================================================
void LoadModulateBuffer(unsigned char Amplitude){
unsigned int Temp_Amp;
unsigned char address;
double angle;
    Temp_Amp = (MAX_SINE * Amplitude) / MAX_AMPLITUDE;
    for (address = 0; address < 128; address++) {
         angle = address * M_PI / 64;
         Table_Buffer[address] = (unsigned char) Temp_Amp * sin(angle);
    }
}
//==============================================================================
//==============================================================================

