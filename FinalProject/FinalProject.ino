#include <LiquidCrystal.h>
#include "DHT.h"
#include <Stepper.h>
#include <RTClib.h>

// B register For DC Motor Fan
volatile unsigned char* port_b = (unsigned char*) 0x25; // Setting the port_b (data register) to address 0x25 (sets bit as high or low, outputs data)
volatile unsigned char* ddr_b = (unsigned char*) 0x24;  // Setting the ddr_b (Data Direction Register) to address 0x24 (sets it as input or output)
volatile unsigned char* pin_b = (unsigned char*) 0x23;  // Setting pin_b (Input Pin Address) to 0x23 (Reading a value from a pin)

// C register for LEDs
volatile unsigned char* port_c = (unsigned char*) 0x28; 
volatile unsigned char* ddr_c = (unsigned char*) 0x27;  
volatile unsigned char* pin_c = (unsigned char*) 0x26; 

// D register for LEDs
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d = (unsigned char*) 0x2A;  
volatile unsigned char* pin_d = (unsigned char*) 0x29; 

// E Registers For Vent Control
volatile unsigned char* port_e = (unsigned char*) 0x2D; 
volatile unsigned char* ddr_e = (unsigned char*) 0x2E;  
volatile unsigned char* pin_e = (unsigned char*) 0x2C; 

// G register for ON/OFF Toggle Button
volatile unsigned char* port_g = (unsigned char*) 0x34; 
volatile unsigned char* ddr_g = (unsigned char*) 0x33;  
volatile unsigned char* pin_g = (unsigned char*) 0x32; 

// L registers for Vent Control
volatile unsigned char* port_l = (unsigned char*) 0x10B; 
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;  
volatile unsigned char* pin_l = (unsigned char*) 0x109; 

// Delay variables
volatile unsigned char *myTCCR1A = (unsigned char *)0x80;
volatile unsigned char *myTCCR1B = (unsigned char *)0x81;
volatile unsigned char *myTCCR1C = (unsigned char *)0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *)0x6F;
volatile unsigned int *myTCNT1 = (unsigned int *)0x84;
volatile unsigned char *myTIFR1 = (unsigned char *)0x36;

// Interrupt variables
volatile unsigned char *mySREG = (unsigned char *)0x5F;
volatile unsigned char *myEICRA = (unsigned char *)0x69;
volatile unsigned char *myEIMSK = (unsigned char *)0x3D;

//MACRO to turn Fan on
#define WRITE_HIGH_PB(pin_num) *port_b |= (0x01 << pin_num) ;
#define WRITE_LOW_PB(pin_num) *port_b &= ~(0x01 << pin_num) ;
#define WRITE_HIGH_PC(pin_num)  *port_c |= (0x01 << pin_num);
#define WRITE_LOW_PC(pin_num)  *port_c &= ~(0x01 << pin_num);


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

//Thresholds
float waterThreshold = 200 ;
float tempThreshold = 55 ;

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

// Current State Global Declarations
volatile bool disabled = true ;
volatile bool error = false ;
volatile bool idle = false ; 
volatile bool buttonPressed = false;
volatile bool resetPressed = false;

volatile bool turnVentL = false;
volatile bool turnVentR = false; 


bool ledState = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

void setup(){
  disabled = true ;
  error = false ; 
  idle = false ; 
  
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

  //LEDs
  *ddr_c |= 0b00001111; //Set required port to output
  //DC Motor Fan
  *ddr_b |= 0b00001000; //Set required port to output
  

  //Interrupts
  // *mySREG &= 0b01111111; // turn off global interrupt

  // *myEICRA |= 0b10100000; // falling edge mode for interrupt 2/3
  // *myEICRA &= 0b10101111; // falling edge mode for interrupt 2/3
  // *myEIMSK |= 0b00001100; // turn on interrupt 2/3
  // *mySREG |= 0b10000000;  // turn on global interrupt
  attachInterrupt(digitalPinToInterrupt(19), onOffSwitchISR, FALLING);
  //attachInterrupt(digitalPinToInterrupt(19), onOffSwitch, FALLING) ;
  // attachInterrupt(digitalPinToInterrupt(18), resetSystem, FALLING);
  // attachInterrupt(digitalPinToInterrupt(3), turnVentClockwise, FALLING);
  // attachInterrupt(digitalPinToInterrupt(2), turnVentCounter, FALLING);

  

  Serial.begin(9600) ;
}

void loop(){
  Serial.println();
  Serial.print("Disabled: ");
  Serial.println(disabled);

  // if (buttonPressed) {
  //   Serial.println("Button pressed!");
  //   buttonPressed = false;
  //   WRITE_HIGH_PC(0);
  // }
  // if (buttonPressed) {
  //   //toggleLed();
  //   buttonPressed = false;
  //   WRITE_HIGH_PC(0);
  // }


  // IF Condition for Disabled State
  if(disabled == true){ //button is toggled off
    Serial.println(F("Entered Disabled State!")) ;
    lcd.clear();
    disabledState();
    //detachInterrupt(digitalPinToInterrupt(18));

  }
if (buttonPressed){
  // IF Condition for Running State
  if(disabled == false && error == false && waterLevelReading() > waterThreshold && dht.readTemperature(true) > tempThreshold){
    Serial.println(F("Entered Running State!")) ;
    lcd.clear() ;
    runningState() ;
  }

  // IF Condition for Idle State
  if(disabled == false && error == false && waterLevelReading() > waterThreshold && dht.readTemperature(true) <= tempThreshold){
    Serial.println(F("Entered Idle State!")) ;
    rtcModule() ;
    Serial.println() ;
    lcd.clear() ;
    idleState() ;
  }
}

  // IF Condition for Error State
  if(disabled == false && error == false && waterLevelReading() <= waterThreshold){
    Serial.println(F("Entered Error State!")) ;
    rtcModule() ;
    lcd.clear() ;
    errorState() ;
  }

//If condition for reset button
  if(resetPressed == true && error == true){
    Serial.println("Reset Button pressed!");
    resetProgram();
    resetPressed = false;    
  }

  // Serial.print("Turnning VentL: ");
  // Serial.println(turnVentR);
  if(turnVentL == true && error == false){
    myStepper.step(stepsPerRevolution) ;
  }  

  // Serial.print("Turnning VentR: ");
  // Serial.println(turnVentR);
  if(turnVentR == true && error == false){
    myStepper.step(-stepsPerRevolution) ;
  }
  
  // rtcModule();
  // double waterLevel = waterLevelReading();
  // Serial.print("WaterLevel: ");
  // Serial.println(waterLevel) ;
  // dhtRead() ;

  //RTC_Module();
  // lcd.setCursor(0, 1) ;
  // lcd.print(millis() / 1000);   

  //Stepper Testing //
  //myStepper.step(stepsPerRevolution) ;

}

// void ONOFFbuttonISR() {
//   buttonPressed = true;
// }

void disabledState(){
  //Green LED pin 34
  //Blue LED pin 36
  //Red LED pin 35
  //Yellow LED 37
  //Serial.println("MEOW");
  //turn motor off
 // *port_b &= 0b00000000 ;
  WRITE_LOW_PB(3);

  //Turn other LEDs off
  //*port_c &= 0b11111111 ;
  WRITE_LOW_PC(0);
  WRITE_LOW_PC(1);
  WRITE_LOW_PC(2);
  WRITE_LOW_PC(3);


  //Turn Yellow LED on
  WRITE_HIGH_PC(0);
  //*port_c |= 0b00000001 ;

  while(disabled == true){
    dhtRead() ;
  }

}

void runningState(){
  //turn Fan On
  //*port_b |= 0b00001000 ;
  WRITE_HIGH_PB(3);

  //Turn all LEDs off
  // *port_c &= 0b00000000 ;
  WRITE_LOW_PC(0);
  WRITE_LOW_PC(1);
  WRITE_LOW_PC(2);
  WRITE_LOW_PC(3);
  
  //Turn Blue LED on
  // *port_c |= 0b00000010 ;
  WRITE_HIGH_PC(1);

  // rtcModule();
  // double waterLevel = waterLevelReading();
  // Serial.print("WaterLevel: ");
  // Serial.println(waterLevel) ;
  // dhtRead() ;

  while(disabled == false && error == false){
    if(waterLevelReading() < waterThreshold){
      errorState() ;
    }
    else{
      dhtRead() ;
    }
  }
}

void idleState(){
  //turn fan off
  //*port_b &= 0b00000000 ;
  WRITE_LOW_PB(3);

  //Turn all LEDs off
  // *port_c &= 0b00000000 ;
  WRITE_LOW_PC(0);
  WRITE_LOW_PC(1);
  WRITE_LOW_PC(2);
  WRITE_LOW_PC(3);

  // Turn Green LED on
  // *port_c |= 0b00001000 ;
  WRITE_HIGH_PC(3);
  //check to see conditions for idle state are still true
  while(disabled == false && error == false && waterLevelReading() > waterThreshold && dht.readTemperature(true) > tempThreshold){
    dhtRead() ;
  }
}

void errorState(){
  error = true ;

  // Turn Fan off
  //*port_b &= 0b00000000 ;
  WRITE_LOW_PB(3);

  // Turn all LEDs off
  //*port_c &= 0b00000000 ;
  WRITE_LOW_PC(0);
  WRITE_LOW_PC(1);
  WRITE_LOW_PC(2);
  WRITE_LOW_PC(3);

  // Turn Red LED on
  // *port_c |= 0b00000100 ;
  WRITE_HIGH_PC(2);
  // Throw Error Message to LCD
  lcd.setCursor(0,0) ;
  lcd.print("Water Level") ;
  lcd.setCursor(0,1) ;
  lcd.print("Too Low!") ;
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
  return temperature ;
}

void dhtToLCD(float h, float t){
   float humidity = dht.readHumidity() ;
  float temperature = dht.readTemperature(true) ;
  Serial.println(F("Temperature: ")) ;
  Serial.println(temperature) ;
  Serial.println(F("Humidity: ")) ;
  Serial.println(humidity) ;

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
  //delay(1000);

}

//int i;
void onOffSwitchISR(){
  disabled = false;
  buttonPressed = !buttonPressed;
  //Serial.println(i++);
}

//int i;
void resetSystem(){
  resetPressed = true;
}
//   //Serial.println(i++);
// }

// void resetSystem() {
//   unsigned long currentTime = millis();

//   if (currentTime - lastDebounceTime > debounceDelay) {
//     resetPressed = true;
//   }

//   lastDebounceTime = currentTime;
// }

void resetProgram() {
  // Reset any variables or flags here
  idle == true; 
  error = false;
  disabled = true;
}

void turnVentClockwise(){
  turnVentL = !turnVentL;
}

void turnVentCounter(){
  turnVentR = !turnVentR;
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

// void mydelay(unsigned int freq)
// {
//   // calc period
//   double period = 1.0 / double(freq);
//   // 50% duty cycle
//   double half_period = period / 2.0f;
//   // clock period def
//   double clk_period = 0.0000000625;
//   // calc ticks
//   unsigned int ticks = half_period / clk_period;
//   // stop the timer
//   *myTCCR1B &= 0xF8;
//   // set the counts
//   *myTCNT1 = (unsigned int)(65536 - ticks);
//   // start the timer
//   *myTCCR1B |= 0b00000001;
//   // wait for overflow
//   while ((*myTIFR1 & 0x01) == 0)
//     ; // 0b 0000 0000
//   // stop the timer
//   *myTCCR1B &= 0xF8; // 0b 0000 0000
//   // reset TOV
//   *myTIFR1 |= 0x01;
// }
