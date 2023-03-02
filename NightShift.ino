#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

// Assign pins of ATTiny85, make sure it runs at 8 MHz internal oscillator

//DigitalPin Assignments
const unsigned int pwm2 = 1; //PWM out on OC1A/PCINT1/Pin6
const unsigned int pwm1 = 4; //PWM out on OC1B/PCINT4/Pin3

//Analog Pin Assignments
const unsigned int rate = A3; //This will be an analog read of speed pot 1 on ADC3/PCINT3/Pin2
const unsigned int phase = A1; //This will be an analog read of speed pot 2 on ADC1/PCINT2/Pin7
const unsigned int modeSwitch = A0;

unsigned long rateTime; // Current time value compared to the previous time to see if we need to move to the next table entry.
unsigned long lastTime; // Used for keeping track of whether we move to the next entry in our sineTable or not
unsigned long maxTime = 10e6; // Max time of 5s. This doesn't seem actually achievable, but whatever.
unsigned long minTime = 3e3; // Min time of 40 ms. This doesn't seem actually achievable, but whatever.
unsigned long rateStep; // The amount of time per pwm step

const uint8_t tableLength = 255; //Number of entries in our tables below
uint8_t inx1 = 0; //Index to read out of the table for PWM1, start at the beginning
uint8_t inx2 = 127; //Index to start reading for table for PWM2, defaults to 180 degrees out of phase compared to PWM1
int dutyCycle1; // Duty cycle for the low frequencies
int dutyCycle2; // Duty cycle for the high frequencies


int checkInterval = 200; // How often to check the user controls in ms
int lastInterval; // When we last checked the interval

int prevPhaseVal = 0; // Previous phase potentiometer analog value for keeping track of whether to update the phase relationship or not
int phaseTol = 5; // Tolerance of phase potentiometer readings

int outMode = 0; // We default to sine


// Create a table for the PWM wave. Values are for 8 bit PWM (max value 255).
// Put it in flash memory so that it doesn't eat up our dynamic SRAM


//Sine
const uint8_t sineTable[] PROGMEM = {127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,179,181,184,187,190,193,195,198,200,203,205,208,210,213,215,217,219,221,223,225,227,229,231,233,235,236,238,239,
241,242,243,245,246,247,248,249,250,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,252,252,251,251,250,249,248,247,246,245,244,243,241,240,239,237,235,234,232,230,228,226,224,222,220,218,216,214,211,209,
207,204,202,199,196,194,191,188,186,183,180,177,174,171,168,166,163,159,156,153,150,147,144,141,138,135,132,129,125,122,119,116,113,110,107,104,101,98,95,91,88,86,83,80,77,74,71,68,66,63,60,58,55,52,50,47,45,43,40,38,36,
34,32,30,28,26,24,22,20,19,17,15,14,13,11,10,9,8,7,6,5,4,3,3,2,2,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,4,5,6,7,8,9,11,12,13,15,16,18,19,21,23,25,27,29,31,33,35,37,39,41,44,46,49,51,54,56,59,61,64,67,70,73,75,78,81,84,87,90,93,
96,99,102,105,108,111,115,118,121,124};

const uint8_t triangleTable[] PROGMEM = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,
247,249,251,253,255,255,253,251,249,247,245,243,241,239,237,235,233,231,229,227,225,223,221,219,217,215,213,211,209,207,205,203,201,199,197,195,193,191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,161,159,157,155,153,151,149,147,
145,143,141,139,137,135,133,131,129,126,124,122,120,118,116,114,112,110,108,106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,68,66,64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0};


const uint8_t risingSawTable[] PROGMEM = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,
67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,
132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,
189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,
246,247,248,249,250,251,252,253,254,255};


//Wave tables commented below for using any wave desired. Arbitrary waveforms can be created and used as well.
//DO NOT DELETE these tables if not using, just leave them commented.
/*
const uint8_t sineTable[] PROGMEM = {127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,179,181,184,187,190,193,195,198,200,203,205,208,210,213,215,217,219,221,223,225,227,229,231,233,235,236,238,239,
241,242,243,245,246,247,248,249,250,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,252,252,251,251,250,249,248,247,246,245,244,243,241,240,239,237,235,234,232,230,228,226,224,222,220,218,216,214,211,209,
207,204,202,199,196,194,191,188,186,183,180,177,174,171,168,166,163,159,156,153,150,147,144,141,138,135,132,129,125,122,119,116,113,110,107,104,101,98,95,91,88,86,83,80,77,74,71,68,66,63,60,58,55,52,50,47,45,43,40,38,36,
34,32,30,28,26,24,22,20,19,17,15,14,13,11,10,9,8,7,6,5,4,3,3,2,2,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,4,5,6,7,8,9,11,12,13,15,16,18,19,21,23,25,27,29,31,33,35,37,39,41,44,46,49,51,54,56,59,61,64,67,70,73,75,78,81,84,87,90,93,
96,99,102,105,108,111,115,118,121,124};
 */



/*
const uint8_t risingSawTable[] PROGMEM = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,
67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,
132,133,134,135,136,137,138,139,140,141,142,143,144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,176,177,178,179,180,181,182,183,184,185,186,187,188,
189,190,191,192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,240,241,242,243,244,245,
246,247,248,249,250,251,252,253,254,255};
*/


/*
const uint8_t fallingSawTable[] PROGMEM = {255,254,253,252,251,250,249,248,247,246,245,244,243,242,241,240,239,238,237,236,235,234,233,232,231,230,229,228,227,226,225,224,223,222,221,220,219,218,217,216,215,214,213,212,211,210,209,
208,207,206,205,204,203,202,201,200,199,198,197,196,195,194,193,192,191,190,189,188,187,186,185,184,183,182,181,180,179,178,177,176,175,174,173,172,171,170,169,168,167,166,165,164,163,162,161,160,159,158,157,156,155,154,153,152,151,
150,149,148,147,146,145,144,143,142,141,140,139,138,137,136,135,134,133,132,131,130,129,128,127,126,125,124,123,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,100,99,98,97,96,95,94,93,92,91,
90,89,88,87,86,85,84,83,82,81,80,79,78,77,76,75,74,73,72,71,70,69,68,67,66,65,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,
12,11,10,9,8,7,6,5,4,3,2,1};
*/


/*
const uint8_t squareTable[] PROGMEM = {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
*/


/*
const uint8_t triangleTable[] PROGMEM = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80,82,84,86,88,90,92,94,96,98,100,102,104,106,108,110,112,114,116,118,120,122,124,
247,249,251,253,255,255,253,251,249,247,245,243,241,239,237,235,233,231,229,227,225,223,221,219,217,215,213,211,209,207,205,203,201,199,197,195,193,191,189,187,185,183,181,179,177,175,173,171,169,167,165,163,161,159,157,155,153,151,149,147,
145,143,141,139,137,135,133,131,129,126,124,122,120,118,116,114,112,110,108,106,104,102,100,98,96,94,92,90,88,86,84,82,80,78,76,74,72,70,68,66,64,62,60,58,56,54,52,50,48,46,44,42,40,38,36,34,32,30,28,26,24,22,20,18,16,14,12,10,8,6,4,2,0};
*/


// This table converts a linear taper speed pot to a logarithmically spaced array of pulse times. This is true logarithmic as opposed to a "log" pot and sounds much nicer
const int logTable[] PROGMEM = {1023,787,699,647,607,575,551,527,507,491,475,463,451,439,427,419,407,399,391,383,375,367,363,355,351,343,339,331,327,323,315,311,307,303,299,295,291,287,283,279,275,271,267,263,259,259,255,251,
247,247,243,239,235,235,231,227,227,223,219,219,215,215,211,207,207,203,203,199,199,195,191,191,187,187,183,183,179,179,179,175,175,171,171,167,167,163,163,163,159,159,155,155,151,151,151,147,147,147,143,143,139,139,139,135,
135,135,131,131,131,127,127,127,123,123,123,119,119,119,115,115,115,111,111,111,107,107,107,107,103,103,103,99,99,99,99,95,95,95,95,91,91,91,91,87,87,87,83,83,83,83,83,79,79,79,79,75,75,75,75,71,71,71,71,67,67,67,67,67,63,63,
63,63,59,59,59,59,59,55,55,55,55,55,51,51,51,51,51,47,47,47,47,47,43,43,43,43,43,39,39,39,39,39,39,35,35,35,35,35,31,31,31,31,31,31,27,27,27,27,27,27,23,23,23,23,23,23,19,19,19,19,19,19,15,15,15,15,15,15,15,11,11,11,11,11,11,
7,7,7,7,7,7,7,3,3,3,3};

void setup() {

  //Define pin modes
  pinMode(pwm1, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(rate, INPUT);
  pinMode(phase, INPUT);
  pinMode(modeSwitch, INPUT);

  lastTime = micros(); // Get an initial value
  lastInterval = millis();

  // Set up timer/PWM items
  PLLCSR |= (1 << PLLE); //Enable PLL

  //Wait for PLL to lock
  while ((PLLCSR & (1<<PLOCK)) == 0x00)
    {
        // Do nothing until plock bit is set
    }

  // Enable clock source bit
  PLLCSR |= (1 << PCKE);
  DDRB |= (1 << PB4) | (1 << PB1);
  
  // Set prescaler to PCK/8, turn on PWM1A, and set COM1A bits to match the COM1B bits due to attiny bug
  TCCR1 = 0;
  TCCR1 |= (1 << CS12) | (1<<PWM1A) | (1 << COM1A1);
  
  // Set OCR1A compare value and OCR1C TOP value
  OCR1C = 159;

  checkMode();
  checkRate();
  checkPhase();
  
  // Enable OCR1B output on PB4, configure compare mode and enable PWM B
  GTCCR |= (1 << PWM1B) | (1 << COM1B1);
  delay(100);
}




void loop() {

  if (millis() - lastInterval >= checkInterval) {
    checkMode();
    checkRate();
    checkPhase();
  }

  updatePWM();

}




//Update PWM outputs based on above settings
void updatePWM() {

  //Compare current time to last time we updated for each PWM out
  if ((micros() - lastTime) > rateStep) {
    //We have met the time threshold for PWM, so go to the next value in the table
    if (outMode == 0) {
      dutyCycle1 = pgm_read_byte(sineTable + inx1);
      dutyCycle2 = pgm_read_byte(sineTable + inx2);
    }
    else if (outMode == 1){
      dutyCycle1 = pgm_read_byte(triangleTable + inx1);
      dutyCycle2 = pgm_read_byte(triangleTable + inx2);
    }
    else {
      dutyCycle1 = pgm_read_byte(risingSawTable + inx1);
      dutyCycle2 = pgm_read_byte(risingSawTable + inx2);
    }
    int mappedPWM1 = map(dutyCycle1,0,255,0,159);
    int mappedPWM2 = map(dutyCycle2,0,255,0,159);
    //analogWrite(pwm,dutyCycle);
    OCR1A = mappedPWM1;
    OCR1B = mappedPWM2;
    inx1+=1; //Increment the read index for PWM1
    inx2+=1; //Increment the read index for PWM1
    
    // Go back to the beginning of the table if we have gotten to the end
    if (inx1 == tableLength) {
      inx1 = 0;
    }

    // Go back to the beginning of the table if we have gotten to the end
    if (inx2 == tableLength) {
      inx2 = 0;
    }
    
    lastTime = micros();
  }
}// end of updatePWM()



//Check what our mode switch is
void checkMode () {
  //Read the switch
  int switchVal = analogRead(modeSwitch);

  //Compare the value to our thresholds and make it the appropriate mode
  if (switchVal > 900) {
    //Sine wave
    if (outMode != 0) {
      //Reset reading to beginning of wave table if we changed modes
      inx1 = 0;
      checkPhase();
    }
    outMode = 0;
  }
  else if (switchVal < 700) {
    //Triangle wave
    if (outMode != 1) {
      //Reset reading to beginning of wave table if we changed modes
      inx1 = 0;
      checkPhase();
    }
    outMode = 1;
  }
  else {
    //Rising Sawtooth Wave
    if (outMode != 2) {
      //Reset reading to beginning of wave table if we changed modes
      inx1 = 0;
      checkPhase();
    }
    outMode = 2;
  }
}//End of checkMode()



void checkRate() {
    
  //Rate pot will be a logarithmic response
  float logTableVal = 0;

  //Calculate out the required time steps to reach before we go to the next step in the table.
  //Times are in microseconds
  float rateVal = 0;
  rateVal = analogRead(rate);

  //Find the closest value in the log table so we can use linear pots for logarithmic time spacing
  logTableVal = pgm_read_word(logTable + round(rateVal/4.0));
  rateTime = round(maxTime * logTableVal/1023.0);

  // Ensure that the time isn't too short
  if (rateTime < minTime) {
    rateTime = minTime;
  }
  
  //Convert microsecond period into amount of time between subsequent samples
  rateStep = round(rateTime/tableLength);
}


void checkPhase() {
  // Phase response will be linear, and will vary from 90 to 270 degrees relative to PWM1
  float phaseVal = analogRead(phase);

  // Calculate the index offset for the phase control
  if (abs(phaseVal - prevPhaseVal) > phaseTol) {
    int offset = round(float(tableLength)/4.0 + float(tableLength)/2.0*phaseVal/1023);
    inx2 = inx1 + offset;
    // Deal with index wrapping around the end of the table cycle
    if (inx2 >= tableLength) {
      inx2 -= tableLength; 
    }
  }
}
