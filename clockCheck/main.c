#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

/**
 * This routine verifies the clock generation algorithm with custom
 * gate lengths.
 */
const int GATELEN_MIN = 5;
const int GATELEN_MAX = 95;
const int ITERATIONS = 6400;

int iterations;
int gateLen;
int clockState;
int progress;

void reset() {
  clockState = 0x00;
  progress   = 0;
}

void printBinary(int n) {
  for (int i = 7; i >= 0; i--) {
    if (n & (1 << i)) {
      printf("1");
    } else {
      printf("0");
    }
  }
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
int setRegisterBits(int mask, int ticks, int ticksThreshold, int hiLen, int bit) {

  if (ticks % ticksThreshold == hiLen) {
    // set low
    mask &= ~(1 << bit);

  } else if (ticks % ticksThreshold == 0) {
    // set high
    mask |= (1 << bit);
  }

  return mask;
}

void writeRegister(int prog, int clock) {
  static int oldData = 0;

  //if (oldData != clock) {
  if (oldData != clock) {
    printf("%06d  ", prog);
    printBinary(clock);
    printf("\n");

    oldData = clock;
  }
}

void advanceClock() {
  progress++;

  if (progress % 100 == 0) {clockState++;;}

  // set relevant clock bits hi/lo according to our metric
  clockState = setRegisterBits(clockState, progress, 100, gateLen, 0);
  clockState = setRegisterBits(clockState, progress, 200, gateLen * 2, 1);
  clockState = setRegisterBits(clockState, progress, 400, gateLen * 4, 2);
  clockState = setRegisterBits(clockState, progress, 800, gateLen * 8, 3);
  clockState = setRegisterBits(clockState, progress, 3200, gateLen * 32, 5);

  writeRegister(progress, clockState);
}

int main(int argc, char *argv[]) {
  int opt;
  gateLen    = GATELEN_MIN;
  iterations = ITERATIONS;

  // parse command line arguments
  while((opt = getopt(argc, argv, "l:i:")) != -1) {
    switch(opt) {
      case 'l':
        gateLen = (int) fmin(GATELEN_MAX, fmax(GATELEN_MIN, atoi(optarg)));
        break;
      case 'i':
        iterations = (int) fmax(0, atoi(optarg));
        break;
      case ':':
        printf("option needs a value\n");
        return 1;
        break;
      case '?':
        printf("unknown option: %c\n", optopt);
        return 1;
        break;
    }
  }

  reset();

  printf("testing clock progression\n");
  printf("gate length: %d/100\n", gateLen);
  printf("iterations : %d\n", iterations);
  printf("ticks not printed means no change in clock signal\n\n");

  printf("ticks   clock   \n");
  printf("----------------\n");

  for (int c = 0; c < iterations; c++) {
    advanceClock();
  }
}
