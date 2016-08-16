### Cigar Mic Software
This software performs a 1024-point FFT with a sampling rate of ~40kHz on analog data provided by the cigar mic to an STM32F103C series microcontroller. The audio/FFT data is then displayed on an SSD1306 (SPI) display, and three LEDs on the cigar mic are turned off or on depending on the bin levels from the FFT.

#### IDE
Software is compatible with the SMT32F103C series, and can be uploaded to a compatible microcontroller using the [Arduino 1.6.9](https://www.arduino.cc/en/Main/Software) IDE. 

Make sure the Arduino IDE is configured to work with STM32 chipsets. Details and necessary files can be found at the [Arduino STM32 project](https://github.com/rogerclarkmelbourne/Arduino_STM32).

#### Libraries
Most libraries necessary are included with the Arduino STM32 setup. The only library missing is the FFT library written by STM, and that is included in the libraries folder. Make sure you copy `libraries/stm32_cr4_fft_1024_lib` into the Arduino libraries folder.

#### To Do
* Amplitude wave mode works when uploading firmware using a serial connection, but not when uploading via bootloader.
* Windowing function needs to be further tested to make sure it’s performing correctly.
* Integrate software with AND!XOR’s Bender Badge Code. Currently the 1024-point FFT requires too much memory and therefore crashes.

#### Credits
The software for cigar mic is based on the following work:
* [Beherith’s STM32_Spectrometer](https://github.com/Beherith/STM32_Spectrometer)
* [pingumacpenguin’s STM32-O-Scope](https://github.com/pingumacpenguin/STM32-O-Scope)