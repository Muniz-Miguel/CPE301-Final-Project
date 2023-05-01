#include <LiquidCrystal.h>
#include "DHT.h"
#include <Stepper.h>
#include <RTClib.h>

// B register For DC Motor Fan
volatile unsigned char* port_b = (unsigned char*) 0x25; // Setting the port_b (data register) to address 0x25 (sets bit as high or low, outputs data)
volatile unsigned char* ddr_b = (unsigned char*) 0x24;  // Setting the ddr_b (Data Direction Register) to address 0x24 (sets it as input or output)
volatile unsigned char* pin_b = (unsigned char*) 0x23;  // Setting pin_b (Input Pin Address) to 0x23 (Reading a value from a pin)

// C register for LEDs
volatile unsigned char* pin_c = (unsigned char*) 0x28; // Setting the port_c (data register) to address 0x28 (sets bit as high or low, outputs data)
volatile unsigned char* ddr_c = (unsigned char*) 0x27; // Setting the ddr_c (Data Direction Register) to address 0x27 (sets it as input or output)
volatile unsigned char* port_c = (unsigned char*) 0x26; // Setting pin_c (Input Pin Address) to 0x26 (Reading a value from a pin)

// D register for LEDs
volatile unsigned char* pin_d = (unsigned char*) 0x2B; // Setting the port_d (data register) to address 0x2B (sets bit as high or low, outputs data)
volatile unsigned char* ddr_d = (unsigned char*) 0x2A; // Setting the ddr_d (Data Direction Register) to address 0x2A (sets it as input or output)
volatile unsigned char* port_d = (unsigned char*) 0x29; // Setting pin_d (Input Pin Address) to 0x29 (Reading a value from a pin)

// G register for ON/OFF Toggle Button
volatile unsigned char* pin_g = (unsigned char*) 0x34; // Setting the port_g (data register) to address 0x34 (sets bit as high or low, outputs data)
volatile unsigned char* ddr_g = (unsigned char*) 0x33; // Setting the ddr_g (Data Direction Register) to address 0x33 (sets it as input or output)
volatile unsigned char* port_g = (unsigned char*) 0x32; // Setting pin_g (Input Pin Address) to 0x32 (Reading a value from a pin)

// L registers for Vent Control
volatile unsigned char* pin_l = (unsigned char*) 0x10B; // Setting the port_l (data register) to address 0x34 (sets bit as high or low, outputs data)
volatile unsigned char* ddr_l = (unsigned char*) 0x10A; // Setting the ddr_l (Data Direction Register) to address 0x33 (sets it as input or output)
volatile unsigned char* port_l = (unsigned char*) 0x109; // Setting pin_l (Input Pin Address) to 0x32 (Reading a value from a pin)

//MACRO to turn Fan on
#define WRITE_HIGH_PB(pin_num) *port_b |= (0x01 << pin_num) ;
#define WRITE_LOW_PB(pin_num) *port_b &= ~(0x01 << pin_num) ;

// ADC for Water Sensor Module
#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Water Sensor Module Threshold
float waterThreshold = 10 ;

//DHT
#define DHTPIN 46
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//RTC Module
RTC_DS1307 rtc ;

//LCD
LiquidCrystal lcd(8,9,4,5,6,7);

//Stepper Motor
double stepsPerRevolution = 2048 ;

Stepper myStepper(stepsPerRevolution, 29, 25, 27, 23) ;

void setup(){

  Serial.begin(9600) ;
  
  //Initialize ADC
  U0init(9600) ;
  adc_init() ;

  //Initialize DHT
  dht.begin();

  //RTC Module
  rtc.begin();

  //LCD
  lcd.begin(16,2) ;
  //lcd.print("T3STING");

  //Stepper Motor
  myStepper.setSpeed(10) ;

  //DC Motor Fan
  *ddr_b = 0b00000110; //sets PB1 and PB2 to outputs

}

void loop(){

  if(*pin_g == B00000000){ //button is toggled off

    //Serial.println("DISABLED STATE") ;
    rtcModule() ;
    //Serial.println() ;
    //disabledState() ;
  }
  
  rtcModule();
  double waterLevel = waterLevelReading();
  Serial.print(waterLevel) ;
  Serial.print('\n') ;
  dhtRead() ;

  //RTC_Module();
  lcd.setCursor(0, 1) ;
  lcd.print(millis() / 1000);  
  delay(1000) ; 

  //Stepper Testing
  myStepper.step(stepsPerRevolution) ;
  delay(1000) ;

  //Fan Testing
  WRITE_HIGH_PB(50) ;
  WIRTE_HIGH_PB(51) ;
}

double waterLevelReading(){
  unsigned int waterLevel = adc_read(0) ;
  return waterLevel ;
}

double dhtRead(){
  float humidity = dht.readHumidity() ;
  float temperature = dht.readTemperature(true) ;
  Serial.println(F("Temperature: ")) ;
  Serial.println(temperature) ;
  Serial.println(F("Humidity: ")) ;
  Serial.println(humidity) ;

  if(isnan(humidity) || isnan(temperature)){
    Serial.println(F("Failed to Read from DHT Sensor!")) ;
    return ;
  }
  dhtToLCD(humidity, temperature) ;
  return temperature ;
}

void dhtToLCD(float h, float t){
  lcd.setCursor(0,0) ;
  lcd.print("Humidity: ") ;
  lcd.print(h) ;
  lcd.print("%") ;

  lcd.setCursor(0,1) ;
  lcd.print("Temp: ") ;
  lcd.print(t) ;
  lcd.print(" F");
}


void rtcModule(){
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
  delay(1000);

}

void adc_init(){
  // setup the A register
  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0b10000000;
  // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11011111;
  // clear bit 4 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11110111;
  // clear bit 3-0 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= 0b11111000;

  
  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111;
  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0b11111000;

  
  // setup the MUX Register
  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0b01111111;
  // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX |= 0b01000000;
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;
    // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11011111;
  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
}
unsigned int adc_read(unsigned char adc_channel_num){
  // reset the channel and gain bits
  *my_ADMUX  &= 0b11100000;
  
  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;
  
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    
    // set MUX bit 
    *my_ADCSRB |= 0b00001000;
  }
  
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  
  // set bit ?? of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;
  
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char U0kbhit(){
  return *myUCSR0A & RDA;
}
unsigned char U0getchar(){
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
