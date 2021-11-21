// Single phase PWM Sine Wave Generator // ดัดแปลงเพิ่มเติม จากต้นฉับ DDS generator ของ KHM 2009 /  Martin Nawrath
// Modified 20-01-2020 By Seree2004 (เสรี เงินประภัศร)
#include <LiquidCrystal.h>
//เรียกโปรแกรมคำสั่งการใช้งานจอแสดงผล LCD เข้ามาร่วมด้วย
PROGMEM const unsigned char Sine_Table[10]  = {"seree2004"};
// เปลี่ยนวิธีการสั่งเซทค่าของรีจีสเตอร์ภายในของ Arduino ใหม่เพื่อให้เรียกใช้ง่ายขึ้น
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
// เปลี่ยนชื่อเรียกขาของ Arduino ใหม่เพื่อให้จำง่ายในการต่อวงจร
#define PWM_OUT_H  11   // CCR2A H output (PB3)
#define PWM_OUT_L  3   // CCR2B L output (PD3)
#define EnablePin 6   // เพื่อปิด/เปิดภาค Power Driver (PD6)
#define FAULT_DETECT     2     //เพื่อตรวจรับสัญญาณผิดพลาดต่างๆจะได้ปิดภาค Power Driver ก่อนที่จะทำให้อุปกรณ์เสียหาย(PD2)
#define L_Led 13  // Board Arduino LED (PB5)
#define Led_Status 5  // (PD5)
#define FAN 8   //(PB0)
#define Sink2 10  //1H  (PB2) สีแดง ในสโคป
#define Sink1 9   //1L  (PB1) สีเขียว ในสโคป
#define VLAVEL     0        // ตรวจจับแรงดันเอ๊าพุต AC Feed Back AC Volted analog input (AD0) 
#define USER1  6    // สำรองไว้  analog input A6
#define USER2  7    // สำรองไว้  analog input A7
// ตั้งชื่อการกำหนดค่า หารเพื่อคำนวน Amplitude
#define MAX_AMPLITUDE 252
#define MIN_AMPLITUDE 140
//----------------------------------------------------------
// การตั้งชื่อตัวแปรต่าง ๆ ที่ใช้ในตัวโปรแกรมจับอุณภูมิ
// which analog pin to connect
#define THERMISTORPIN 1   //Port Analog ตัวจับอุณภูมิ
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
//----------------------------------------------------------
int samples[NUMSAMPLES];
volatile  float freq;
// ค่าความถี่ PWM const double refclk=15686;  // =8Mhz 8000000Hz / 510 = 15686
const float refclk = 15686  ;     //  ค่าตายตัวที่ได้มาจากการคำนวน คริสตอลที่ใช้ 8 MHz/510 = 15686
// ตั้งชื่อตัวแปรที่ใช้ในการคำนวนรอบของสัญญาณซายเวฟและต่อเนื่องกันอย่างสมบูรณ์ interrupt service declared as voilatile
volatile unsigned long sigma;   // phase accumulator
volatile unsigned long delta;  // phase increment
volatile unsigned int  c4ms;  // counter incremented every 4ms
//ตั้งชื่อตัวแปรต่าง ๆ
// ค่าเขียนอักษรภาษาไทย
byte character1[8] = {0, 18, 18, 18, 18, 18, 27, 0};
byte character2[8] = {0, 14, 17, 29, 21, 21, 25, 0};
byte character3[8] = {0, 4, 4, 4, 4, 4, 6, 0};
byte character4[8] = {0, 2, 31, 1, 29, 19, 25, 0};
byte character5[8] = {1, 15, 6, 9, 14, 1, 6, 0};
byte character6[8] = {24, 24, 7, 8, 8, 8, 7, 0};
byte character7[8] = {0, 0, 20, 23, 29, 22, 23, 0};
// ค่าตัวแปรต่าง ๆ
byte Table_Buffer[128];
byte phase1, temp_ocr, ocr_outA, ocr_outB, tlavel, AC_lavel, temp, Volts, Volts_Show, Old_NVolts;
unsigned int V_Read, NVolts;
bool T_status;
bool EN_status = true;
// ตั้งชื่อค่าตายตัวที่เรากำหนดในการต่อขาต่าง ๆ ของ Arduino เข้ากับจอ LCD เพื่อให้จำง่าย
const int RS = 7, EN = 4, D4 = A2, D5 = A3, D6 = A4, D7 = A5;
// แล้วสั่งให้กำหนดตามนี้ได้เลย
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
//*********** เปลี่ยนให้แสดงผล Version ที่นี่ ****************************
char My_Version[9] = "20-01-20";
//*****************************************************************
void(* resetFunc) (void) = 0;//declare reset function at address 0  //ตั้งฟังชั่นการ RESET โปรแกรม








// เริ่มต้นตัวโปรแกรมโดยคำสั่งตั้งค่าต่าง ๆ ก่อน---------------------------
void setup() {
  pinMode(FAN, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(EnablePin, OUTPUT);     // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(Led_Status, OUTPUT);
  pinMode(L_Led, OUTPUT);
  pinMode(Sink1, OUTPUT);       // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(Sink2, OUTPUT);     // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_L, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  pinMode(PWM_OUT_H, OUTPUT); // สั่งเซตให้เป็นขาสัญญาณออก
  // เซ็ตให้ขาที่ไปควบคุมโมดูลขับให้ถูกต้อง สำคัญเพราะจะทำให้โมดูลขับเสียถ้าไม่เซ็ตให้ถูกต้องก่อน
  OCR1A = 0;  // PWM_OUT_H
  OCR1B = 0;  // PWM_OUT_L
  OCR2A = 0;  // PWM_OUT_H
  OCR2B = 0;  // PWM_OUT_L
  T_status = false;
  c4ms = 0;
  digitalWrite(PWM_OUT_L, LOW);
  digitalWrite(PWM_OUT_H, LOW);
  digitalWrite(FAN, LOW);
  pinMode(FAULT_DETECT, INPUT_PULLUP); // สั่งเซตให้เป็นขารับสัญญาณเข้า และ พูลอัพ
  digitalWrite(EnablePin, LOW);   // สั่งให้ขานี้ ปิด (เป็น0) เพื่อทำให้ภาค  Power Drive อย่าเพิ่งทำงานตอนนี้
  //-----------------------------------------------------------------------------
  lcd.begin(16, 2); // เริ่มเปิดใช้งานจอแสดงผล LCD
  // โหลดข้อมูลตัวอักษรภาษาไทยไปเก็บไว้ในจอ LCD
  //-----------------------------------------------------------------------------
  // โหลดข้อมูลตัวอักษรภาษาไทยไปเก็บไว้ในจอ LCD
  lcd.createChar(0, character1);
  lcd.createChar(1, character2);
  lcd.createChar(2, character3);
  lcd.createChar(3, character4);
  lcd.createChar(4, character5);
  lcd.createChar(5, character6);
  lcd.createChar(6, character7);
  lcd.setCursor(1, 0); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัดแรก (0)
  //  lcd.print("DadSeree");  //ถ้าจะใช้ภาษาอังกฤษ
  lcd.write(byte(0));  //ภาษาไทย ไม่เกิน 8 ตัวอักษร
  lcd.write(byte(1));
  lcd.write(byte(1));
  lcd.write(byte(2));
  lcd.write(byte(3));
  lcd.write(byte(4));
  lcd.setCursor(0, 1);   // สังให้แสดงผล เวอร์ชั่น วัน เดือน ปี ที่เขียน คอลั่มที่ 1 (0) ในบรรทัดที่ 2 (1)
  for (temp = 0; temp <= 7; temp++) {
    lcd.print(My_Version[temp]);
    delay(200);
  }
  digitalWrite(FAN, HIGH);
  //-----------------------------------------------------------------------------
  cbi (TIMSK2, TOIE2);             // หยุดการทำงานของไทม์เมอร์ 2  disable timer2 overflow detect
  setup_timer2();
  delay (2000);
  //-----------------------------------------------------------------------------
  digitalWrite(FAN, LOW);
  Old_NVolts = 0;
  freq = 51;
  LoadModulateBuffer(MIN_AMPLITUDE);
  delta = pow(2, 32) * freq / refclk ;
  sbi (TIMSK2, TOIE2);             //เปิดให้ไทม์เมอร์ 2 ทำงาน enable timer2 overflow detect เริ่มสร้างสัญญาณซายเวฟ 10 Hz.
  //-----------------------------------------------------------------------------
  attachInterrupt(0, fault, LOW); //เริ่มให้ทำการตรวจการผิดปรกติ กันโหลดเกิน
  //-----------------------------------------------------------------------------
  lcd.clear();
  for (temp = MIN_AMPLITUDE; temp <= 240; (temp = temp + 10)) { //warm Up 0-10 Amplitude
    LoadModulateBuffer(temp);
    AC_lavel = V_Read >> 2;
    Volts = map(AC_lavel, 0, 255, 0, 50);
    Volts_Show = Volts + 190;
    lcd.setCursor(0, 0);
    lcd.print(Volts_Show);
    lcd.print("V 50");
    lcd.write(byte(6));
    lcd.print(" ");
    lcd.print(temp);
    lcd.print(" ");
    lcd.setCursor(0, 1);
    lcd.print(temp - 152);
    lcd.print("% ");
    lcd.setCursor(5, 1);
    lcd.print(tlavel);
    lcd.write(byte(5));
    lcd.print(" ");
    c4ms = 1;
    while (c4ms);
  }
  if (V_Read <= 50) {      //ถ้าไม่มีไฟ AC ออกมาเลยให้ลองรีเซตใหม่
    digitalWrite(EnablePin, LOW); // สั่งให้ขานี้ ปิด (เป็น0) เพื่อทำให้ภาค Power Drive อย่าเพิ่งทำงานตอนนี้
    cbi (TIMSK2, TOIE2);             // หยุดการทำงานของไทม์เมอร์ 2  disable timer2 overflow detect
    lcd.clear();
    lcd.setCursor(0, 0); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัดแรก (0)
    lcd.print("Lost AC!"); // แสดงผล  Error
    lcd.setCursor(0, 1); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัด 2 (1)
    lcd.print("ResetOFF");
    delay (1000);
    resetFunc(); //call reset
  }
}
//------------------------------- ทำงาน วน อยู่ในส่วนนี้เท่านั้น ---------------------
void loop() {
  uint8_t i;
  float average;
  // take N samples in a row, with a slight delay
  for (i = 0; i < NUMSAMPLES; i++) {
    samples[i] = analogRead(THERMISTORPIN);
    while (c4ms);
    //wait (200);
  }
  // average all the samples out
  average = 0;
  for (i = 0; i < NUMSAMPLES; i++) {
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
  AC_lavel = V_Read >> 2;
  Volts = map(AC_lavel, 0, 255, 0, 50);
  Volts_Show = Volts + 190;
  NVolts = map(AC_lavel, 255, 0, 100, 300) + 80; //ชดเชยให้ได้ค่าเท่ากับบอร์ด EGS002 เดิม
  if (NVolts <= MIN_AMPLITUDE) NVolts = MIN_AMPLITUDE;
  if (NVolts >= MAX_AMPLITUDE) NVolts = MAX_AMPLITUDE;
  if (NVolts != Old_NVolts) {
    LoadModulateBuffer ((uint8_t)NVolts);
    Old_NVolts = NVolts;
  }
  lcd.setCursor(0, 0);
  lcd.print(Volts_Show);
  lcd.print("V 50");
  lcd.write(byte(6));
  lcd.setCursor(0, 1);
  lcd.print(NVolts - 152);
  lcd.print("%   ");
  lcd.setCursor(5, 1);
  lcd.print(tlavel);
  lcd.write(byte(5));
  lcd.print(" ");
  lcd.print(Volts);
  lcd.print("V ");
  lcd.print(NVolts);
  lcd.print("V ");
  while (c4ms);
  if (tlavel >= 45) {
    digitalWrite(FAN, HIGH);
  }
  if (tlavel <= 40) {
    digitalWrite(FAN, LOW);
  }
}
//------------------------------ timer2 setup -----------------------------------------------
void setup_timer2(void) {
  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);

  // Timer2 PWM Mode set to Phase Correct PWM
  cbi (TCCR2A, COM2B0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง 3(OCR2B) ให้ส่งสัญญาณออกเป็น Sine PWM LOW ปรกติ +
  sbi (TCCR2A, COM2B1);
  sbi (TCCR2A, COM2A0); // ตั้งค่าให้ชุดส่งสัญญาณช่อง 3(OCR2A)  ให้ส่งสัญญาณออกเป็น Sine PWM HIGH กลับเฟส Invert เป็น -
  sbi (TCCR2A, COM2A1);
  sbi (TCCR2A, WGM20); //ตั้งค่าให้สัญญาณพัลซ์ทุกลูกอยู่แนวเดียวกันเสมอในทุกเฟส (Mode 1 / Phase Correct PWM)
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);
}
//-------------------------------------------------------------------------------------------------
// ส่งInterrupt Service Routine attached to INT0 vector-------------
void fault() {
  digitalWrite(EnablePin, LOW); // สั่งให้ขานี้ ปิด (เป็น0) เพื่อทำให้ภาค Power Drive อย่าเพิ่งทำงานตอนนี้
  cbi (TIMSK2, TOIE2);             // หยุดการทำงานของไทม์เมอร์ 2  disable timer2 overflow detect
  OCR1A = 0;
  OCR1B = 0;
  OCR2A = 0;  // pin D6
  OCR2B = 0;  // pin D5
  lcd.clear();
  lcd.setCursor(0, 0); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัดแรก (0)
  lcd.print("OverLoad"); // แสดงผล  Error
  lcd.setCursor(0, 1); // สังให้แสดงผลคอลั่มที่ 1 ในบรรทัด 2 (1)
  lcd.print("ResetOFF");
  while (1);
}
//------------------------------------------------------------------------------
ISR(TIMER2_OVF_vect) {
  // สูตรการคำนวน  ความถี่กลับมา แล้วเลื่อน จาก 32 บิต กลับมาใช้เพียง 8 บิตล่าง
  sigma = sigma + delta; // soft DDS, phase accu with 32 bits
  phase1 = sigma >> 25;   // use upper 8 bits for phase accu as frequency information

  temp_ocr = Table_Buffer[phase1];
  ocr_outA = temp_ocr;
  ocr_outB = temp_ocr;
  if (temp_ocr >> 0) ocr_outA = temp_ocr + 3;
  OCR2A = ocr_outA;  // pin D6
  OCR2B = ocr_outB;  // pin D5
  switch (phase1) {
    case 0:
      cbi(PORTB, 1);  //digitalWrite(Sink1, LOW); //สีเขียว ในสโคป
      break;
    case 1:
      sbi (PORTB, 2); //digitalWrite(Sink2, HIGH);  //สีแดง ในสโคป
      break;
    case 65:
      if (EN_status) {
        digitalWrite(EnablePin, HIGH); // เปิด ภาค Power Drive ให้เริ่มทำงานได้
        EN_status = false;
      }
      cbi (PORTB, 2); //digitalWrite(Sink2, LOW); //สีแดง ในสโคป
      break;
    case 66:
      sbi (PORTB, 1); //digitalWrite(Sink1, HIGH);  //สีเขียว ในสโคป
      break;
    case 96:
      V_Read = analogRead(VLAVEL);
      if (c4ms++ == 20) {
        c4ms = 0;
        digitalWrite(Led_Status, T_status);
        T_status = !T_status;
      }
      break;
  }
}
//--------------------------------------------------------------------------------------------------------------------
void LoadModulateBuffer(byte Amplitude) {
  byte tmp, cout;
  double angle;
  if (Amplitude <= MIN_AMPLITUDE) Amplitude = MIN_AMPLITUDE;
  if (Amplitude >= MAX_AMPLITUDE) Amplitude = MAX_AMPLITUDE;
  for (cout = 0; cout < 128; cout++) {
    angle = cout * M_PI / 64;
    Table_Buffer[cout] = Amplitude * sin(angle);
  }
}
//จบโปรแกรมแล้วครับใครจะเขียนเพิ่มเติมอะไรได้ครับ แต่อย่าลืมให้เครดิตผู้ที่เขียนต้นฉบับด้วยครับ
