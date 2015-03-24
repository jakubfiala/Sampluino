/* Arduino Audio Loopback Test
 *
 * Arduino Realtime Audio Processing
 * 2 ADC 8-Bit Mode
 * analog input 1 is used to sample the audio signal
 * analog input 0 is used to control an audio effect
 * PWM DAC with Timer1 as analog output                                                        
 // Modified by Artin
 // Modified by Doug
 // ...added support for Leonardo...
 //... moved output pin to pin 9 from pin 11.
 // added support for 10 bit mode
 * adapted from the very fine code from:
 * KHM 2008 / Lab3/  Martin Nawrath nawrath@khm.de
 * Kunsthochschule fuer Medien Koeln
 * Academy of Media Arts Cologne
 */

#include <SD.h>

File soundfile;
File writefile;


#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


//int ledPin = 13;                 
// LED connected to digital pin 13
//int testPin = 7;

boolean div32;
boolean div16;
// interrupt variables accessed globally
volatile boolean f_sample;
volatile byte badc0;
volatile byte badc1;
volatile byte ibb;

int ii;

boolean led_blink;

int icnt;
int cnt2;

float pi = 3.141592;
float dx ;
float fd ;
float fcnt;
float fcnt2;
int iw;
byte bb;
int svitch = 0;

int c [4] = {3,5,6,7};
int r [3]= {A2, A3, A4};
/*const int bufferSize = 512;

byte dd[bufferSize];  
byte dd2[bufferSize]; */ 
// Audio Memory Array 8-Bit


void setup(){
//  pinMode(ledPin, OUTPUT);      
  // sets the digital pin as output
//  pinMode(testPin, OUTPUT);
  for(int i = 0; i < 4; i++){
    pinMode(c[i], INPUT);
  }
  for(int i = 0; i < 3; i++){
    pinMode(r[i], OUTPUT);
  }
  
  pinMode(8, OUTPUT);                                                                       
  // Added by Artin
  pinMode(9, OUTPUT);
  pinMode(4, INPUT);  
  // Added by Artin
  Serial.begin(57600);        
  // connect to the serial port
  Serial.println("Initializing Sampluino...");
  Serial.println("Initializing SD card...");
  pinMode(10,OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  if(SD.exists("newfile.txt")){
    SD.remove("newfile.txt");
  }
  soundfile = SD.open("newfile.txt", FILE_WRITE);
  
  if (soundfile) {
    Serial.println("Fetched audio data");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error fetching audio data");
  }

    // set adc prescaler  to 64 for 19kHz sampling frequency
  cbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  sbi(ADMUX,ADLAR);  
  // 8-Bit ADC in ADCH Register
  sbi(ADMUX,REFS0);  
  // VCC Reference
  cbi(ADMUX,REFS1);

  cbi(ADMUX,MUX0);   
  // Set Input Multiplexer to Channel 0
  cbi(ADMUX,MUX1);
  cbi(ADMUX,MUX2);
  cbi(ADMUX,MUX3);

  // Timer1 PWM Mode set to fast PWM                                                        // Modified by Artin
  cbi (TCCR1A, COM1A0);                                                                     // Modified by Artin
  sbi (TCCR1A, COM1A1);                                                                     // Modified by Artin
  cbi (TCCR1A, COM1B0);                                                                     // Modified by Artin
  cbi (TCCR1A, COM1B1);                                                                     // Modified by Artin
  sbi (TCCR1A, WGM10);                                                                      // Modified by Artin
  cbi (TCCR1A, WGM11);                                                                      // Modified by Artin
  sbi (TCCR1B, WGM12);                                                                      // Modified by Artin
  cbi (TCCR1B, WGM13);                                                                      // Modified by Artin

  // Timer1 Clock Prescaler to : 1                                                         // Modified by Artin
  sbi (TCCR1B, CS10);                                                                      // Modified by Artin
  cbi (TCCR1B, CS11);                                                                      // Modified by Artin
  cbi (TCCR1B, CS12);                                                                      // Modified by Artin

  //cli();                         
  // disable interrupts to avoid distortion
  cbi (TIMSK0,TOIE0);              
  // disable Timer0 !!! delay is off now                  
  // Commented by Artin
  sbi (TIMSK1,TOIE1);              
  // enable Timer1 Interrupt                              
  // Modified by Artin

}


int playPos = 0;
int playLength = 1;
float realPos = 0;
int iLow, iHigh;
float remainder, lowval, highval;
int previous = 0;
float multiplier = 1;
float iterator = 0;
int note = 0;
float spd = 0;



void loop(){
  
  while (!f_sample) {        
    // wait for Sample Value from ADC
  } 

  if (!previous && digitalRead(4)) {
    if (soundfile.size() > 0) {
      soundfile.close();
      SD.remove("newfile.txt");
      soundfile = SD.open("newfile.txt", FILE_WRITE);
    }
    previous = 1;
    iterator = 0;
    Serial.println("writing ...");
  }
  else if (previous && digitalRead(4)) {
    char c = badc1;
    soundfile.print(c);
  }
  else if (previous && !digitalRead(4)) {
    soundfile.seek(0);
    soundfile.flush();
    Serial.println("playing back ...");
    previous = 0;
  }  


//  digitalWrite(testPin, HIGH);

  f_sample=false;            
  // reset Flag
  
  for(int i = 0 ; i < 3 ; i++){
    digitalWrite(r[i], HIGH);
    for(int j = 0 ; j < 4 ; j++){
      if(digitalRead(c[j]) == HIGH){
        soundfile.seek(0);
        note = (i*4)+j;
        iterator = 0;
      }
    }
    digitalWrite(r[i], LOW);
  }
  
  spd = 1.65 + note*0.13;
  
  if (soundfile.available() && !digitalRead(4)) {
     soundfile.seek((int)round(iterator));
     iterator += spd;
     badc1 = soundfile.read();      
     OCR1AL=badc1;
  }

}


//******************************************************************
// Timer1 Interrupt Service at 62.5 KHz
// here the audio and pot signal is sampled in a rate of:  16Mhz / 256 / 2 / 2 = 15625 Hz
// runtime : xxxx microseconds
ISR(TIMER1_OVF_vect) {                                                                    // Modified by Artin

  digitalWrite(8, HIGH);

  div32=!div32;                            
  // divide timer0 frequency / 2 to 31.25kHz     
  // Modified by Artin
  if (div32){ 
    div16=!div16;  // 
    if (div16) {                       
      // sample channel 0 and 1 alternately so each channel is sampled with 15.6kHz
      badc0=ADCH;                    
      // get ADC channel 0
      sbi(ADMUX,MUX0);               
      // set multiplexer to channel 1
    }
    else{
      badc1=ADCH;                    
      // get ADC channel 1
      cbi(ADMUX,MUX0);               
      // set multiplexer to channel 0
      f_sample=true;
    }
    ibb++; 
    ibb--; 
    ibb++; 
    ibb--;    // short delay before start conversion
    sbi(ADCSRA,ADSC);              // start next conversion
  }

}

