# Burnell :: Clock and Clock Divider

## Abstract
This is a combined clock generator and clock divider module based on an Arduino Nano.

## Clock features
- Clock speed adjustable via potentiometer between 10 and 300 BPM.
- Gate length adjustable via potentiometer and gate / trigger toggle switch.
- Switches for start/stop and reset.
- Reset trigger output.
- Clock outputs for beat, bar, 8ths, 16ths and 32ths.

## Function of the gate length control
The gate length potentiometer varies the length of the high-state of an output between 5% and 95% of its interval.

This makes the module useful for both triggers and to control envelopes and similar modules which require long gates.

When the gate / trigger switch is in the trigger position then the high-state length of all outputs equals that of the 32ths output instead. You can still control the 32ths output gate length with the gate length potentiometer as before.

## Clock divider features
The clock divider section of the module has an external clock input so you can use it to divide an external pulse just as well as the internal clock. By default the clock divider is normalled to the bar output of the clock section.

The divider can be switched between three modes, which changes the outputs to:
- power of two: 2, 4, 8, 16, 32, 64, 128, 256
- primes      : 2, 3, 5, 7, 11, 13, 17,  19
- fibonacci   : 2, 3, 5, 8, 13, 21, 34,  55

The gate / trigger toggle switch of the clock section affects the gate length of the clock divider in the same way that it affects the clock outputs.


## Technical Specifications
The module is 12 HE wide.

Power consumption:
- 5V: 20mA
- +12V: 25mA
- -12V: 25mA

## What's in the name
The module was named after [Jocelyn Bell Burnell](https://en.wikipedia.org/wiki/Jocelyn_Bell_Burnell), who together with Antony Hewish first discovered Pulsars when she was a postgraduate student. Antony Hewish, her supervisor, was awarded the Nobel price in Physics for the discovery. Burnell herself was omitted.

Pulsars are highly magnetized rotating neutron stars which emit beams of electromagnetic radiation. The beam can only observed when their beam is directed at earth. The so generated pulses are highly precise.
