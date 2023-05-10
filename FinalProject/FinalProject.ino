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

//MACRO to turn Fan and LED's on
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
float tempThreshold = 75;

//DHT
#define DHTPIN 46
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

//RTC Module
RTC_DS1307 rtc ;

//LCD
LiquidCrystal lcd(8,9,4,5,6,7);

unsigned long lastRefreshTime = 0; // timmer for refresh of lcd

//Stepper Motor
double stepsPerRevolution = 2048 ;

Stepper myStepper(stepsPerRevolution, 29, 25, 27, 23) ;
// Stepper myStepper(stepsPerRevolution, A2, A3, A4, A5) ;


// Current State Global Declarations
#define disabled 0
#define enable 1 //cant use running or run for somereason?
#define idle 2 
#define error 3

int state;
volatile bool buttonPressed = false;
volatile bool resetPressed = false;

volatile bool turnVentL = false;
volatile bool turnVentR = false; 

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void setup(){
  state = 0;
  buttonPressed = false;
  
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

  //Interupts
  attachInterrupt(digitalPinToInterrupt(19), onOffSwitchISR, FALLING) ;
  attachInterrupt(digitalPinToInterrupt(18), resetSystem, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), turnVentClockwise, FALLING);
  attachInterrupt(digitalPinToInterrupt(2), turnVentCounter, FALLING);
}


int i;
void loop(){
  // debug statements 
  // Serial.println();
  // Serial.print("STATE: ");
  // Serial.println(state);
  // Serial.println();
  // Serial.print("SYSTEMENABLED: ");
  // Serial.println(system_enabled);
  printString("STATE: ");
  printInt(state);
  printString("\n");

  if (buttonPressed){

  printString("THE BUTTON WAS PRESSED\n");
  buttonPressed = false;
  }

if(state == 0){
    printString("Disabled State at ") ;
    rtcModule() ;
    lcd.clear();
    disabledState();
}
//} else if (system_enabled){
  //state 0 = diable, 1 = enable(running), 2 = idle, 3  = error
  if(state != disabled && state != error && waterLevelReading() > waterThreshold){ // running condition
    dhtRead(); //needed so new temps can be read
    if(dht.readTemperature(true) > tempThreshold){
      state = enable;

    } else if(dht.readTemperature(true) <= tempThreshold) { // idle condition
      state = idle;
    } 
  } else if (state != disabled && state != error && waterLevelReading() < waterThreshold){ // error condition
    state = error; 

  } else if (state != disabled && state == error && resetPressed == true){
    state = idle;
    resetPressed = false;
  } 

  switch(state){

    case 0: //disabled state
      printString("Entered Disabled State at ") ;
      rtcModule();
      lcd.clear();
      disabledState();
      break;

    case 1: // enable(running) state
    //if(lastButtonState != 1){
      printString("Entered Running State at ") ;
      rtcModule();
      // lcd.clear() ;
      dhtReadLCD();
      runningState() ;
    //}
      break;
    case 2: //idle state
      printString("Entered Idle State! at ") ;
      rtcModule() ;
      Serial.println() ;
      // lcd.clear() ;
      dhtReadLCD();
      idleState() ;
      break;

    case 3: //error state
      printString("Entered Error State at ") ;
      rtcModule() ;
      // lcd.clear() ;
      //dhtReadLCD();
      errorState() ;
      break;
  //}

} 

  if(turnVentL == true && state != error){ //vent control for clockwise 
    printString("Turnning VentL: \n");
    for(int j = 0; i < 1; i++){
    myStepper.step(stepsPerRevolution) ;
    }
    turnVentL = false;
  }  
  
  if(turnVentR == true &&  state != error){ // vent control for counterclockwise
    printString("Turnning VentR \n");
    myStepper.step(-stepsPerRevolution) ;
    turnVentR = false;
  }
  
  //instrument testing
  // rtcModule();
  // double waterLevel = waterLevelReading();
  // Serial.print("WaterLevel: ");
  // Serial.println(waterLevel) ;
  // dhtRead() ;

  // RTC_Module();
  // lcd.setCursor(0, 1) ;
  // lcd.print(millis() / 1000);   

  //Stepper Testing //
  // myStepper.step(stepsPerRevolution) ;

}

void disabledState(){
  //turn motor off
  //*port_b &= 0b00000000 ;
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
}

void errorState(){
  //error = true ;

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
  printString("Temperature: ") ;
  printFloat(temperature, 2) ;
  printString("\n") ;
  printString("Humidity: ") ;
  printFloat(humidity, 2) ;
  printString("\n") ;

  if(isnan(humidity) || isnan(temperature)){
    printString("Failed to Read from DHT Sensor!") ;
    return ;
  }
  return temperature ;
}

double dhtReadLCD(){
  //mydelay(1000);
  float humidity = dht.readHumidity() ;
  float temperature = dht.readTemperature(true) ;
  printString("Temperature: ") ;
  printFloat(temperature, 2) ;
  printString("\n") ;
  printString("Humidity: ") ;
  printFloat(humidity, 2) ;
  printString("\n") ;

  if(isnan(humidity) || isnan(temperature)){
    printString("Failed to Read from DHT Sensor!") ;
    return ;
  }
  unsigned long currrentMillis = millis(); 
  if (currrentMillis - lastRefreshTime >= 60000) {
    // save the current time as the last refresh time
    lastRefreshTime = millis();
    dhtToLCD(humidity, temperature);
  }
  return temperature ;
}

void dhtToLCD(float h, float t){
  // double freq = 1.0 / 60.0;
  // mydelay((unsigned int) (freq * 1000000));
  //delay(1000); 
  float humidity = dht.readHumidity() ;
  float temperature = dht.readTemperature(true) ;
  printString("Temperature: ") ;
  printFloat(temperature, 2) ;
  printString("\n") ;
  printString("Humidity: ") ;
  printFloat(humidity, 2) ;
  printString("\n"); 
  lcd.clear();
  
  lcd.setCursor(0,0) ;
  lcd.print("Humidity: ") ;
  lcd.print(h) ;
  lcd.print("%") ;
  //delay(1000);

  lcd.setCursor(0,1) ;
  lcd.print("Temp: ") ;
  lcd.print(t) ;
  lcd.print(" F");
  // delay(1000);
}


void rtcModule(){
  DateTime now = rtc.now();
  printInt(now.year());
  U0putchar('/');
  printInt(now.month());
  U0putchar('/');
  printInt(now.day());
  U0putchar(' ');
  printInt(now.hour());
  U0putchar(':');
  printInt(now.minute());
  U0putchar(':');
  printInt(now.second());
  U0putchar('\n');

}

//state 0 = diable, 1 = running, 2 = idle, 3  = error
void onOffSwitchISR(){
    unsigned long currentTime = millis();

  if (currentTime - lastDebounceTime > debounceDelay) {
    buttonPressed = true;
    //system_enabled = !system_enabled ;
  }

  lastDebounceTime = currentTime;
    // buttonPressed = true;
    // system_enabled = !system_enabled ;
  if (state == 0){
    state = 2; // set to idle
  } else {
    state = 0;
  }

}

//int i;
void resetSystem(){
  resetPressed = true;
}

void turnVentClockwise(){
  turnVentL = true;
}

void turnVentCounter(){
  turnVentR = true;
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
void printString(const char* s){
  int i = 0;
  while (s[i]) {
    U0putchar(s[i]);
    i++;
  }
}

void printFloat(float f, int precision) {
  char buf[20];
  dtostrf(f, 6, precision, buf);
  printString(buf);
}
void printInt(int n) {
  char buf[10];
  sprintf(buf, "%d", n);
  printString(buf);
}
