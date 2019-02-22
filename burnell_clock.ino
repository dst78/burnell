/**
 * Anoikis Nomads :: Burnell
 * Eurorack clock and clock divider module
 * 
 * All information on Burnell is on github: https://github.com/dst78/burnell
 * 
 * Released under Creative Commons Attribution-ShareAlike 3.0 Unported
 * https://creativecommons.org/licenses/by-sa/3.0/deed.de 
 * 
 * Pins and settings below are for an Arduino Nano.
 */
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <RBD_MicroTimer.h>

RBD::MicroTimer clockTimer;
RBD::MicroTimer divTimer;

//---- configure pins 
// input pins w/ interrupt
#define DIV_IN            2
#define RESET_IN          3
// digital output pins
#define SHIFTREG_SER      4
#define SHIFTREG_RCLK     5
#define SHIFTREG_SRCLK    6
#define RST_OUT           7
#define RST_LED           A4
// digital input pins
#define GATEMODE_IN       8
#define STARTSTOP_IN      9
// analog input pins
#define SPEED_IN         A2
#define GATELEN_IN       A1
#define DIVMODE_IN       A0


// program constants
#define DIVMODE_CLOCK     0
#define DIVMODE_POW       1
#define DIVMODE_PRIME     2
#define DIVMODE_FIBONACCI 3
#define GATEMODE_VARIABLE true
#define GATEMODE_FIXED    false
#define STARTED           true
#define STOPPED           false
// gate length range in percent
#define GATELEN_MIN       5
#define GATELEN_MAX      95
// clock speed in BPM
#define CLOCK_SPEED_MIN  10
#define CLOCK_SPEED_MAX 300

// array of divisions for clock and clock divider, in 100ths
uint16_t divs[4][8] = {
  {100, 200, 400,  800, 1600, 3200,  6400, 12800}, // internal clock
  {200, 400, 800, 1600, 3200, 6400, 12800, 25600}, // power of two
  {200, 300, 500,  700, 1100, 1300,  1700,  1900}, // prime numbers
  {200, 300, 500,  800, 1300, 2100,  3400,  5500}, // fibonacci sequence
};

double clkSpeed;
uint16_t gateLen;
volatile bool gateMode = GATEMODE_FIXED;
volatile bool started  = STOPPED;

volatile uint8_t clockState;
volatile uint16_t clkResCount;

uint8_t divMode;
uint8_t divState;

volatile uint16_t divResCount;
volatile double divClkSpeed;
volatile uint32_t divInterruptTime1, divInterruptTime2;


void setup() {
  // will be overridden in the first call of loop() so this initial value shouldn't ever be used
  divMode  = DIVMODE_POW; 

  // configure pins
  // interrupt pin
  pinMode(DIV_IN, INPUT);
  // digital output pins
  pinMode(SHIFTREG_SER, OUTPUT);
  pinMode(SHIFTREG_RCLK, OUTPUT);
  pinMode(SHIFTREG_SRCLK, OUTPUT);
  pinMode(RST_OUT, OUTPUT);
  pinMode(RST_LED, OUTPUT);
  // analog input pins
  pinMode(SPEED_IN, INPUT);
  pinMode(GATELEN_IN, INPUT);
  pinMode(DIVMODE_IN, INPUT);
  // digital input pins
  pinMode(STARTSTOP_IN, INPUT_PULLUP);
  pinMode(GATEMODE_IN, INPUT_PULLUP);

  // read startup values
  if (digitalRead(STARTSTOP_IN) == HIGH) {started = STARTED;}
  if (digitalRead(GATEMODE_IN) == HIGH) {gateMode = GATEMODE_VARIABLE;}
  
  // set up pin change interrupts
  attachPCINT(digitalPinToPCINT(GATEMODE_IN), handleGateModeChange, CHANGE);
  attachPCINT(digitalPinToPCINT(STARTSTOP_IN), handleStartStop, CHANGE);
  attachPCINT(digitalPinToPCINT(RESET_IN), reset, RISING);

  /**
   * 1 beat is giving BPM
   * we're outputting up to resolution of 32ths, which is 1/8ths of 1 beat
   * 
   * with adjustable gatelengths we need to be precise to 100th of 1/32ths.
   */
  clkSpeed    = 10000.0 / (map(389, 0, 1023, CLOCK_SPEED_MIN, CLOCK_SPEED_MAX) / 7.5); // 120 BPM in 3200ths resolution
  gateLen     = map(512, 0, 1023, GATELEN_MIN, GATELEN_MAX); // 50% initial gate length
  clockTimer.setTimeout(clkSpeed);

  divClkSpeed = 0;
  divTimer.setTimeout(divClkSpeed);
  divTimer.stop();

  // attach interrupt to clock division input
  attachInterrupt(digitalPinToInterrupt(DIV_IN), handleDivClockTrigger, RISING);
  // reset clock & update registers
  reset();
}

void loop() {
  double rawSpeed, bpm, clockUs;

  // determine clock divider mode
  uint16_t divModeVoltage = analogRead(DIVMODE_IN);

  if (divModeVoltage < 102) {
    divMode = DIVMODE_PRIME;
  } else if (divModeVoltage < 491) {
    divMode = DIVMODE_FIBONACCI;
  } else {
    divMode = DIVMODE_POW;
  }

  // read relative gate length
  gateLen   = (float) map(analogRead(GATELEN_IN), 0, 1023, GATELEN_MIN, GATELEN_MAX);

  // read raw clock speed and calculate interrupts
  rawSpeed  = analogRead(SPEED_IN);
  bpm       = map(rawSpeed, 0, 1023, CLOCK_SPEED_MIN, CLOCK_SPEED_MAX);
  // 10000 is 100ths of 1000000 which is 1 sec in microseconds
  // 7.5 is 1/8ths of 60, which gives us 1/32ths resolution
  clockUs = 10000.0 / (bpm / 7.5);  
  
  if (clockUs != clkSpeed) {
    clkSpeed = clockUs;
    clockTimer.setTimeout(clkSpeed);
  }

  // advance clocks
  if (started) {
    if (clockTimer.onRestart()) {
      advanceClock();
    }
    if (divTimer.onRestart()) {
      advanceDivClock();
    }
  }
}

/**
 * resets all counters
 * 
 * doubles as pin change interrupt handler
 */
void reset() {
  clockState     = 0x00;
  divState       = 0x00;  
  clkResCount    = 0;
  divResCount    = 0;

  digitalWrite(RST_OUT, HIGH);
  digitalWrite(RST_LED, HIGH);
  delay(5);
  digitalWrite(RST_OUT, LOW);
  writeRegisters(clockState, divState);
  delay(195);
  digitalWrite(RST_LED, LOW);
}

/**
 * writes to the 74595 chips
 */
void writeRegisters(uint8_t clk, int8_t divi) {
  digitalWrite(SHIFTREG_RCLK, LOW);
  shiftOut(SHIFTREG_SER, SHIFTREG_SRCLK, LSBFIRST, clk);
  shiftOut(SHIFTREG_SER, SHIFTREG_SRCLK, LSBFIRST, divi);
  digitalWrite(SHIFTREG_RCLK, HIGH);  
}

/**
 * manipulates a bit of the clock mask in a generic way
 *
 * @param uint8_t  mask              mask to manipulate
 * @param uint16_t ticks             current clock ticks
 * @param uint16_t ticksThreshold    threshold when the current bit should go high
 * @param uint16_t hiLen             how many ticks the current bit should remain high
 * @param uint16_t hiLenMult         multiplicator for the hiLen value
 * @param uint8_t bit                which bit to manipulate
 * @return uint8_t                   the manipulate clock mask
 */
int setRegisterBits(uint8_t mask, uint16_t ticks, uint16_t ticksThreshold, uint16_t hiLen, uint16_t hiLenMult, uint8_t bitv) {

  if (gateMode == GATEMODE_VARIABLE) {
    if (ticks % ticksThreshold == hiLen * hiLenMult) {
      // set low
      mask &= ~(1 << bitv);
    } else if (ticks % ticksThreshold == 0) {
      // set high
      mask |= (1 << bitv);
    }
    
  } else {
    // fixed gateMode
    if (ticks % ticksThreshold == hiLen) {
      // set low
      mask &= ~(1 << bitv);
    } else if (ticks % ticksThreshold == 0) {
      // set high
      mask |= (1 << bitv);
    }
  }

  return mask;
}

/**
 * advances the internal clock, called by TimerOne
 */
void advanceClock() {
  clkResCount++;

  // set relevant clock bits hi/lo according to our metric
  clockState = setRegisterBits(clockState, clkResCount, divs[DIVMODE_CLOCK][0], gateLen,  1, 0);
  clockState = setRegisterBits(clockState, clkResCount, divs[DIVMODE_CLOCK][1], gateLen,  2, 1);
  clockState = setRegisterBits(clockState, clkResCount, divs[DIVMODE_CLOCK][2], gateLen,  4, 2);
  clockState = setRegisterBits(clockState, clkResCount, divs[DIVMODE_CLOCK][3], gateLen,  8, 3);
  clockState = setRegisterBits(clockState, clkResCount, divs[DIVMODE_CLOCK][5], gateLen, 32, 5);
  
  writeRegisters(clockState, divState);
}

/**
 * advances the clock divider, called by TimerOne
 */
void advanceDivClock() {
  divResCount++;

  divState = setRegisterBits(divState, divResCount, divs[divMode][0], gateLen, divs[divMode][0] / 100, 0);
  divState = setRegisterBits(divState, divResCount, divs[divMode][1], gateLen, divs[divMode][1] / 100, 1);
  divState = setRegisterBits(divState, divResCount, divs[divMode][2], gateLen, divs[divMode][2] / 100, 2);
  divState = setRegisterBits(divState, divResCount, divs[divMode][3], gateLen, divs[divMode][3] / 100, 3);
  divState = setRegisterBits(divState, divResCount, divs[divMode][4], gateLen, divs[divMode][4] / 100, 4);
  divState = setRegisterBits(divState, divResCount, divs[divMode][5], gateLen, divs[divMode][5] / 100, 5);
  divState = setRegisterBits(divState, divResCount, divs[divMode][6], gateLen, divs[divMode][6] / 100, 6);
  divState = setRegisterBits(divState, divResCount, divs[divMode][7], gateLen, divs[divMode][7] / 100, 7);

  writeRegisters(clockState, divState);
}

/**
 * interrupt handler when a rising signal on the clock division pin was detected.
 */
void handleDivClockTrigger() {
  uint32_t now = micros();
  uint8_t remainder = (divResCount % 100);

  // calculate new clock speed
  divInterruptTime1 = divInterruptTime2;
  divInterruptTime2 = now;
  divClkSpeed = (divInterruptTime2 - divInterruptTime1) / 100.0;

  // update the timer
  divTimer.setTimeout(divClkSpeed);
  divTimer.restart();

  // correct the divisor count to a multiple of 100 to stay in sync with triggers
  if (remainder > 0) {
    divResCount += 99 - remainder;
    // using 99 so the next call to advanceDivClock() rolls it over to 100
  }
}

/**
 * pin change interrupt handler
 */
void handleGateModeChange() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(GATEMODE_IN));

  if (trigger == RISING) {
    gateMode = GATEMODE_VARIABLE;
  } else if (trigger == FALLING) {
    gateMode = GATEMODE_FIXED;
  }
}

/**
 * pin change interrupt handler
 */
void handleStartStop() {
  // Differenciate between RISING and FALLING on mode CHANGE.
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(STARTSTOP_IN));
  
  if (trigger == RISING) {
    started = STARTED;
  } else if (trigger == FALLING) {
    started = STOPPED;
    writeRegisters(0x00, 0x00); // pull all outputs low
  } 
}
