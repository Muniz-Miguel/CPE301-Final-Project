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
