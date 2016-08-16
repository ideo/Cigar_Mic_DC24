### Cigar Mic Hardware
Schematic and layout created using [Eagle 7.4.0](https://cadsoft.io/)

The Cigar Mic is a fairly simple analog device comprised of a microphone, an amplifier, and a low pass filter. The cutoff frequency of the LPF is ~10kHz and the gain of the amplifier is 100. The circuit requires and 3.3V source, and the output signal is centered around ~1.65V.

The top side of the board includes 3 orange LEDs that run at 20mA each (this is very bright, and I’d suggest raising the value of their resistors, R7,8,9, to make the LEDs a little more subtle). These are positioned to make it look like the cigar is lit.

This layout differs slightly from what was released at DEF CON 24 in order to fix two errors with the original design. The first being the + and - flipped, hence the flywires soldered to these pins, and the other being the mic leads were flipped. Both of these were a result of quick design and not paying attention/double checking parts designed by others.