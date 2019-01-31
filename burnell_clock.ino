#include <TimerOne.h>

// input pins w/ interrupt
#define DIV_IN          2
#define RESET_IN        3

// digital output pins
#define SHIFTREG_SER    4
#define SHIFTREG_RCLK   5
#define SHIFTREG_SRCLK  6
#define RST_OUT         7

// digital input pins
#define GATEMODE_IN     8
#define STARTSTOP_IN    9

// analog input pins
#define SPEED_IN        A2
#define GATELEN_IN      A1
#define DIVMODE_IN      A0

// array of divisions for clock divider
float divs[3][8] = {
  {2, 4, 8, 16, 32, 64, 128, 256}, // power of two
  {2, 3, 5,  7, 11, 13, 17, 19},   // prime numbers
  {2, 3, 5,  8, 13, 21, 34, 55},   // fibonacci sequence
};

uint16_t speedMin = 10;
uint16_t speedMax = 300;
double clkSpeed;
uint16_t const gateMin = 5;  // 5% minimum gate length, increased to resolution
uint16_t const gateMax = 95; // 95% maximum gate length, increased to resolution
uint16_t gateLen;

volatile uint8_t clockState;
volatile uint8_t divState;
volatile uint16_t clkResCount;


void setup() {
  // set up interrupt pins
  pinMode(DIV_IN, INPUT);
  pinMode(RESET_IN, INPUT);
  
  // set up digital output pins
  pinMode(SHIFTREG_SER, OUTPUT);
  pinMode(SHIFTREG_RCLK, OUTPUT);
  pinMode(SHIFTREG_SRCLK, OUTPUT);
  pinMode(RST_OUT, OUTPUT);
  
  // set up digital input pins
  pinMode(GATEMODE_IN, INPUT_PULLUP);
  pinMode(STARTSTOP_IN, INPUT_PULLUP);
  
  // set up analog input pins
  pinMode(SPEED_IN, INPUT);
  pinMode(GATELEN_IN, INPUT);
  pinMode(DIVMODE_IN, INPUT);

  /**
   * 1 beat is giving BPM
   * we're outputting up to resolution of 32ths, which is 1/8ths of 1 beat
   * 
   * with adjustable gatelengths we need to be precise to 100th of 1/32ths.
   */
  clkSpeed    = 10000.0 / (map(389, 0, 1023, speedMin, speedMax) / 7.5); // 120 BPM in 3200ths resolution
  gateLen     = map(512, 0, 1023, gateMin, gateMax);

  Timer1.initialize(clkSpeed);
  Timer1.attachInterrupt(advanceClock);
  
  // reset clock & update registers
  reset();

  Serial.begin(9600);

  Serial.print("gate length range is: ");
  Serial.print(gateMin);
  Serial.print(" - ");
  Serial.println(gateMax);
  Serial.println("-------------------");
 
}

void loop() {
  double raw, bpm, clockUs;

  gateLen = map(analogRead(GATELEN_IN), 0, 1023, gateMin, gateMax);
  
  raw     = analogRead(SPEED_IN);
  bpm     = map(raw, 0, 1023, speedMin, speedMax);
  // 10000 is 100ths of 1000000 which is 1 sec in microseconds
  // 7.5 is 1/8ths of 60, which gives us 1/32ths resolution
  clockUs = 10000.0 / (bpm / 7.5);  
  
  if (clockUs != clkSpeed) {
    clkSpeed = clockUs;
    Timer1.setPeriod(clkSpeed);
    
    Serial.print(" raw: ");
    Serial.print(raw);
    Serial.print(" bpm: ");
    Serial.print(bpm);
    Serial.print(" clock uS: ");
    Serial.print(clockUs);
    Serial.print(" gateLen: ");
    Serial.println(gateLen);
    
  }
}

void reset() {
  clockState     = 0x00;
  divState       = 0x00;  
  clkResCount    = 100;
  
  writeRegisters(clockState, divState);
}

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
 * @param uint8_t bit                which bit to manipulate
 * @return uint8_t                   the manipulate clock mask
 */
int setRegisterBits(uint8_t mask, uint16_t ticks, uint16_t ticksThreshold, uint16_t hiLen, uint8_t bit) {

  if (ticks % ticksThreshold == hiLen) {
    // set low
    mask &= ~(1 << bit);
    
  } else if (ticks % ticksThreshold == 0) {
    // set high
    mask |= (1 << bit);
  }

  return mask;
}

void advanceClock() {
  clkResCount++;
  
  // set relevant clock bits hi/lo according to our metric
  clockState = setRegisterBits(clockState, clkResCount, 100, gateLen, 0);
  clockState = setRegisterBits(clockState, clkResCount, 200, gateLen * 2, 1);
  clockState = setRegisterBits(clockState, clkResCount, 400, gateLen * 4, 2);
  clockState = setRegisterBits(clockState, clkResCount, 800, gateLen * 8, 3);
  clockState = setRegisterBits(clockState, clkResCount, 3200, gateLen * 32, 5);
  
  writeRegisters(clockState, divState);
}
