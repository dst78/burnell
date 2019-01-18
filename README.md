# Burnell :: Clock and Clock Divider

- gatelength adjustable with pot

## Abstract
This is a combined clock generator and clock divider module based on an Arduino Nano.

I decided to combine the two to make use of the available pins more than anything else.


## Clock features
Clock speed adjustable via potentiometer. (duh)
Gate length adjustable via potentiometer.

Pushbuttons for start/stop and reset.

Reset trigger output.

Clock outputs for beat, bar, 8ths and 16ths.

Toggle switch to change modes between trigger and gate
- In trigger mode the high state of any output is restricted to the length of the high state of the 16ths signal. 
- In gate mode any output the high-time depends on the gate length pot.


## Clock divider features
The clock divider part has an external clock input though so you can use it to divide an external pulse just as well as the internal clock. By default the clock divider is normalled to the bar output of the clock.

The divider can be switched between three modes, which changes the outputs to:
- power of two: 2, 4, 8, 16, 32
- prime       : 2, 3, 5,  7,  9
- finbonacci  : 2, 3, 5,  8, 13

When the main module is in trigger mode, all outputs will be high for the amount of time that the clock divider input is high. Is it in gate mode, then the high-time depends on the gate length pot.


## What's in the name
The module was named after [Jocelyn Bell Burnell](https://en.wikipedia.org/wiki/Jocelyn_Bell_Burnell), who together with Antony Hewish first discovered Pulsars when she was a postgraduate student. Antony Hewish, her supervisor, was awarded the Nobel price in Physics for the discovery. Burnell herself was omitted.

Pulsars are highly magnetized rotating neutron stars which emit beams of electromagnetic radiation. The beam can only observed when it is directed at earth. The so generated pulses are highly precise.
