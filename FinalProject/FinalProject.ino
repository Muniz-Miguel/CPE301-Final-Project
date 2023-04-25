#include <LiquidCrystal.h>
#include "DHT.h"
#include <Servo.h>
#include <DS3231.h>

//Water Sensor Module
volatile unsigned char* port_b = (unsigned char*) 0x25 ;
volatile unsigned char* ddr_b = (unsigned char*) 0x24 ;
volatile unsigned char* pin_b = (unsigned char*) 0x23 ;

void setup(){

  Serial.begin(9600) ;


}

void loop(){
  waterLevelReading() ;
}

double waterLevelReading(){
  unsigned int waterLevel = adc_read(0) ;
  return waterLevel ;
}