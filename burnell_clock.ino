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
// clock speed and speed range in BPM
#define CLOCK_SPEED_MIN  10
#define CLOCK_SPEED_MAX 300
#define CLOCK_RESOLUTION 22.0

/**
 * raw array of divisions for clock and clock divider
 */
uint16_t divs[4][8] = {
  {1, 2, 4,  8, 16, 32,  64, 128}, // internal clock
  {2, 4, 8, 16, 32, 64, 128, 256}, // power of two
  {2, 3, 5,  7, 11, 13,  17,  19}, // prime numbers
  {2, 3, 5,  8, 13, 21,  34,  55}, // fibonacci sequence
};
/**
 * hiked up divisions, will be calculated dynamically based on CLOCK_RESOLUTION
 */
uint16_t subDivs[4][8] = {
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0}
};
/**
 * least common multiples for all steps in all division modes.
 * used to modulo the clock counters in a way that they never
 * overflow at a position where any of the digits skip a beat.
 */
uint32_t lcms[4] = {128, 256, 9699690, 2042040};

/**
 * interval length of the clock timer in microseconds
 */
double clkSpeed;

/**
 * current gatelength relative to CLOCK_RESOUTION / 100
 */
uint16_t gateLen;
/**
 * adjusted gatelength minimum / maximum in relation to CLOCK_RESOLUTION / 100
 */
uint8_t gateLenMin, gateLenMax;
/**
 * indicates the gate mode, derived from GATEMODE_IN
 */
volatile bool gateMode = GATEMODE_FIXED;
/**
 * whether the clock is started or stopped. derived from STARTSTOP_IN
 */
volatile bool started  = STOPPED;

/**
 * bit mask for the shift register. these are the clock outputs
 */
volatile uint8_t clockState;
/**
 * internal counter of the clock, counting CLOCK_RESOLUTION steps per 32ths
 */
volatile uint32_t clkResCount;

/**
 * indicates the clock divider mode. derived from the voltage at DIVMODE_IN.
 */
uint8_t divMode;

/**
 * bit mask for the shift register. these are the clock divider outputs
 */
volatile uint8_t divState;
/**
 * internal counter of the clock divider. counting CLOCK_RESOLUTION steps per divider input signal.
 */
volatile uint32_t divResCount;
/**
 * internal speed of the timer handling the clock divider, in microseconds.
 * derived from the time passing between HIGH signals on DIV_IN
 */
volatile double divClkSpeed;
/**
 * time keeping variables to calculate divClkSpeed
 */
volatile uint32_t divInterruptTime1, divInterruptTime2;

/**
 * analog pins are read in loop() only when this variable overflows.
 * this is required to save on CPU cycles and allow for fast BPM generation
 */
uint8_t loopCount;

void setup() {
  // correct array of clock substeps according to CLOCK_RESOLUTION
  correctClockResolution();
  
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

  // read startup values
  if (digitalRead(STARTSTOP_IN) == HIGH) {started = STARTED;}
  if (digitalRead(GATEMODE_IN) == HIGH) {gateMode = GATEMODE_VARIABLE;}
  
  // set up pin change interrupts
  attachPCINT(digitalPinToPCINT(GATEMODE_IN), handleGateModeChange, CHANGE);
  attachPCINT(digitalPinToPCINT(STARTSTOP_IN), handleStartStop, CHANGE);
  attachPCINT(digitalPinToPCINT(RESET_IN), reset, CHANGE);

  /**
   * 1 beat is giving BPM
   * we're outputting up to resolution of 32ths, which is 1/8ths of 1 beat
   * 
   * with adjustable gatelengths we need to be precise to CLOCK_RESOLUTION-th of 1/32ths.
   */
  clkSpeed    = (1000000.0 / CLOCK_RESOLUTION) / (map(389, 0, 1023, CLOCK_SPEED_MIN, CLOCK_SPEED_MAX) / 7.5); // 120 BPM in 3200ths resolution
  gateLen     = map(512, 0, 1023, gateLenMin, gateLenMax); // 50% initial gate length
  clockTimer.setTimeout(clkSpeed);

  divClkSpeed = 0;
  divTimer.setTimeout(divClkSpeed);
  divTimer.stop();

  // attach interrupt to clock division input
  attachInterrupt(digitalPinToInterrupt(DIV_IN), handleDivClockTrigger, RISING);
  // initialize clocks & update registers
  initClocks();

  // initialize loopCount
  loopCount = 0;
}

void loop() {
  double rawSpeed, bpm, clockUs;

  // loopCount is used to read values only every so often in order to increase the maximum BPM
  if (loopCount == 0) {
    // determine clock divider mode
    uint16_t divModeVoltage = analogRead(DIVMODE_IN);
  
    if (divModeVoltage < 102) {
      divMode = DIVMODE_PRIME;
    } else if (divModeVoltage < 491) {
      divMode = DIVMODE_FIBONACCI;
    } else {
      divMode = DIVMODE_POW;
    }
  }

  if (loopCount == 85) {
    // read relative gate length
    gateLen   = map(analogRead(GATELEN_IN), 0, 1023, gateLenMin, gateLenMax);
  }

  if (loopCount == 170) {
    // read raw clock speed and calculate interrupts
    rawSpeed  = analogRead(SPEED_IN);
    bpm       = map(rawSpeed, 0, 1023, CLOCK_SPEED_MIN, CLOCK_SPEED_MAX);
    // resolution in 1,000,000ths of a second (1 microsecond) divided by clock resolution
    // 7.5 is 1/8ths of 60, which gives us 1/32ths resolution
    clockUs = (1000000.0 / CLOCK_RESOLUTION) / (bpm / 7.5);
    
    if (clockUs != clkSpeed) {
      clkSpeed = clockUs;
      clockTimer.setTimeout(clkSpeed);
    }
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
  
  loopCount++;
}

/**
 * initializes the timers and registers
 */
void initClocks() {
  clockState     = 0x00;
  divState       = 0x00;  
  clkResCount    = 0;
  divResCount    = 0;
  
  writeRegisters(clockState, divState);
}

/**
 * resets all counters
 * 
 * doubles as pin change interrupt handler
 */
void reset() {
  uint8_t trigger = getPinChangeInterruptTrigger(digitalPinToPCINT(RESET_IN));

  if (trigger == FALLING) {
    clockTimer.stop();
    divTimer.stop();
    
    initClocks();
  
    digitalWrite(RST_OUT, HIGH);
    digitalWrite(RST_LED, HIGH);
    
  } else if (trigger == RISING) {
    clockTimer.restart();
    divTimer.restart();
    
    digitalWrite(RST_OUT, LOW);
    digitalWrite(RST_LED, LOW);
  }
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
    if (ticks % ticksThreshold >= hiLen * hiLenMult) {
      // set low
      mask &= ~(1 << bitv);
    } else if (ticks % ticksThreshold == 0) {
      // set high
      mask |= (1 << bitv);
    }
    
  } else {
    // fixed gateMode
    if (ticks % ticksThreshold >= hiLen) {
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
 * advances the internal clock
 */
void advanceClock() {
  // set relevant clock bits hi/lo according to our metric
  clockState = setRegisterBits(clockState, clkResCount, subDivs[DIVMODE_CLOCK][0], gateLen, divs[DIVMODE_CLOCK][0], 0);
  clockState = setRegisterBits(clockState, clkResCount, subDivs[DIVMODE_CLOCK][1], gateLen, divs[DIVMODE_CLOCK][1], 1);
  clockState = setRegisterBits(clockState, clkResCount, subDivs[DIVMODE_CLOCK][2], gateLen, divs[DIVMODE_CLOCK][2], 2);
  clockState = setRegisterBits(clockState, clkResCount, subDivs[DIVMODE_CLOCK][3], gateLen, divs[DIVMODE_CLOCK][3], 3);
  clockState = setRegisterBits(clockState, clkResCount, subDivs[DIVMODE_CLOCK][5], gateLen, divs[DIVMODE_CLOCK][5], 5);
  
  writeRegisters(clockState, divState);
  
  // modulo operation on clkResCount to prevent overflow at fractions of CLOCK_RESOLUTION
  clkResCount = (clkResCount + 1) % lcms[DIVMODE_CLOCK];
}

/**
 * advances the clock divider, called by TimerOne
 */
void advanceDivClock() {
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][0], gateLen, divs[divMode][0], 0);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][1], gateLen, divs[divMode][1], 1);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][2], gateLen, divs[divMode][2], 2);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][3], gateLen, divs[divMode][3], 3);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][4], gateLen, divs[divMode][4], 4);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][5], gateLen, divs[divMode][5], 5);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][6], gateLen, divs[divMode][6], 6);
  divState = setRegisterBits(divState, divResCount, subDivs[divMode][7], gateLen, divs[divMode][7], 7);

  writeRegisters(clockState, divState);
  
  // modulo operation on clkResCount to prevent overflow at fractions of CLOCK_RESOLUTION
  divResCount = (divResCount + 1) % lcms[divMode];
}

/**
 * interrupt handler when a rising signal on the clock division pin was detected.
 */
void handleDivClockTrigger() {
  uint32_t now = micros();
  uint8_t remainder = (divResCount % (uint8_t) CLOCK_RESOLUTION);

  // calculate new clock speed
  divInterruptTime1 = divInterruptTime2;
  divInterruptTime2 = now;
  divClkSpeed = (divInterruptTime2 - divInterruptTime1) / CLOCK_RESOLUTION;

  // update the timer
  divTimer.setTimeout(divClkSpeed);
  divTimer.restart();

  // correct the divisor count to a multiple of CLOCK_RESOLUTION to stay in sync with triggers
  if (remainder > 0) {
    divResCount += CLOCK_RESOLUTION - remainder;
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

/**
 * corrects the substep count in subDivs array according to CLOCK_RESOLUTION.
 * will also recalculate gate length range 
 */
void correctClockResolution() {
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      subDivs[i][j] = divs[i][j] * CLOCK_RESOLUTION;
    }

    lcms[i] = lcms[i] * CLOCK_RESOLUTION;
  }

  // map gatelengths relative to CLOCK_RESOLUTION as well
  gateLenMin = (GATELEN_MIN * CLOCK_RESOLUTION) / 100;
  gateLenMax = (GATELEN_MAX * CLOCK_RESOLUTION) / 100;
}
