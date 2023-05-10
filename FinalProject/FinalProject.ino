#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

void setup() {
  // put your setup code here, to run once:
  U0init(9600) ;
}
<<<<<<< Updated upstream
=======
int i;
void loop(){
  // Serial.println();
  // Serial.print("Disabled: ");
  // Serial.println(disabled);
  // Serial.println();
  // Serial.print("IDLE: ");
  // Serial.println(idle);
  // Serial.println();
  // Serial.print("ERROR: ");
  // Serial.println(error);
  // Serial.println();
  // Serial.print("RUNNING: ");
  // Serial.println(running);
  // Serial.print("STATE: ");
  // Serial.println(state);
  // Serial.println();
  // Serial.print("SYSTEMENABLED: ");
  // Serial.println(system_enabled);
  printString("Hello World! \n") ;
  int myInt = 5;
  printInt(myInt);
  printString("\n");
  // if(system_enabled){
  //   printString("1");
  // } else if (system_enabled == false){
  //   printString("0");
  // }
  //printString(system_enabled);
  //Serial.println(system_enabled);
  // Serial.println();
  // Serial.print("buttonPressed: ");
  // Serial.println(buttonPressed);
>>>>>>> Stashed changes

void loop() {
  // put your main code here, to run repeatedly:
  printString("Hello World! \n") ;
  int myInt = 5;
  printInt(myInt);
  printString("\n");
  delay(1000) ;
}

void printString(const char* s){
  int i = 0;
  while (s[i]) {
    U0putchar(s[i]);
    i++;
  }
}

void printInt(int n) {
  char buf[10];
  sprintf(buf, "%d", n);
  printString(buf);
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
<<<<<<< Updated upstream

unsigned char U0kbhit(){
  return *myUCSR0A & RDA;
}

unsigned char U0getchar(){
  return *myUDR0;
}

=======
>>>>>>> Stashed changes
void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
<<<<<<< Updated upstream
=======
void printString(const char* s){
  int i = 0;
  while (s[i]) {
    U0putchar(s[i]);
    i++;
  }
}
void printInt(int n) {
  char buf[10];
  sprintf(buf, "%d", n);
  printString(buf);
}
void mydelay(unsigned int freq)
{
  // calc period
  double period = 1.0 / double(freq);
  // 50% duty cycle
  double half_period = period / 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int)(65536 - ticks);
  // start the timer
  *myTCCR1B |= 0b00000001;
  // wait for overflow
  while ((*myTIFR1 & 0x01) == 0)
    ; // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8; // 0b 0000 0000
  // reset TOV
  *myTIFR1 |= 0x01;
}
>>>>>>> Stashed changes
