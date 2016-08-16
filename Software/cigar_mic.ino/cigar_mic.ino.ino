////////////////////////////////////////////////////////////////////
//
//  Real-time spectrum analyzer for 0-20kHz
//   
//  Designed for DEF CON 24 and AND!XOR's Bender Badge
//
//  1024-point FFT
//  ~40kHz sampling rate
//
//  Hardware and software developed by rob rehr (@mediumrehr)
//  Last modified: Aug 16, 2016
//
//  To Do:
//  - find/fix the error with showing the audio wave in DFU mode
//  - further test windowing function
//  - integrate with AND!XOR Bender Badge code
//    - reduce SRAM usage
//
//  MIT License
//
//  Credits
//  FFT library written by STMicroelectronics
//  Software based on projects by Beherith and pingumacpenguin
//
////////////////////////////////////////////////////////////////////

//-FFT-------------------------------------------------------------------------
#define FFTLEN 1024
#define LOGSCALE 1 // display bins logarithmicallly
#define LINSCALE 0 // display bins linearly
#include <cr4_fft_1024_stm32.h>
uint16_t data16[FFTLEN];
uint32_t data32[FFTLEN];
uint32_t y[FFTLEN];
uint16_t hammingwindow[FFTLEN/2];
// my best attempt at logarithmically spacing 511 bins
uint16_t logBins[32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,15,18,22,27,33,40,49,59,72,87,106,129,156,190,230,280,340,413};
uint16_t logBinsGap[32] = {1,1,1,1,1,1,1,1,1,1,1,1,1,2,3,4,5,6,7,9,10,13,15,19,23,27,34,40,50,60,73,98};
uint16_t FFTMode = 1; // 1 for frequecy display; 0 for amplitude display

//-Display---------------------------------------------------------------------
#include <Adafruit_GFX_AS.h>
#include <Adafruit_SSD1306_STM32.h>
#include <SPI.h>

#define SCREENWIDTH 128
#define SCREENHEIGHT 64

#define OLED_MOSI  PB15
#define OLED_CLK   PB13
#define OLED_DC    PB3
#define OLED_CS    PB12
#define OLED_RESET PB4
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

//-DMA-------------------------------------------------------------------------
volatile static bool dma1_ch1_Active;
#include <libmaple/pwr.h>
#include <libmaple/scb.h>
#include <libmaple/rcc.h>
#include <libmaple/adc.h>

//-Other-----------------------------------------------------------------------
// cigar LEDs
#define LED1 PA3
#define LED2 PA4
#define LED3 PA5
#define LEDThresh 200
const int8_t analogInPin = PA1;
uint16_t ampWave[SCREENWIDTH] = {0};

void init_hamming_window(uint16_t *windowtarget, int len) {
  for(int i=0; i<len/2; i++){ windowtarget[i] = (0.54-0.46*cos((2*i*3.141592)/(len-1)))*65536; }
}

// not certain windowing is working correctly. more testing is needed
void window(uint32_t *data, uint16_t *weights, int len, int scale) {
  for(int i=0; i<len; i++){
    int weight_index = i;
    if(i >= len/2) weight_index = (len-1)-i;
    data[i] = ((data[i]*scale*weights[weight_index]) >> 16) & 0xFFFF;
  }
}

uint16_t asqrt(uint32_t x) {
  /*   From http://medialab.freaknet.org/martin/src/sqrt/sqrt.c
   *   Logically, these are unsigned. We need the sign bit to test
   *   whether (op - res - one) underflowed.
   *   good enough precision; 10x faster than regular sqrt
   */
  int32_t op, res, one;

  op = x;
  res = 0;
  /* "one" starts at the highest power of four <= than the argument. */
  one = 1 << 30;   /* second-to-top bit set */
  while(one > op) one >>= 2;
  while(one != 0) {
    if(op >= res+one) {
      op = op-(res+one);
      res = res+2*one;
    }
    res /= 2;
    one /= 4;
  }
  return (uint16_t)(res);
}

void fill(uint32_t *data, uint32_t value, int len) {
  for(int i=0; i<len; i++) { data[i] = value; }
}

void fill(uint16_t *data, uint32_t value, int len) {
  for(int i=0; i<len; i++) { data[i] = value; }
}

void real_to_complex(uint16_t *in, uint32_t *out, int len) {
  for(int i=0; i<len; i++) { out[i] = in[i]*8; }
}

void setADCs () {
  rcc_set_prescaler(RCC_PRESCALER_ADC, RCC_ADCPRE_PCLK_DIV_8 );

  int pinMapADCin = PIN_MAP[analogInPin].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_239_5); // ~37.65 khz sample rate
 
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // set continuous mode
  ADC1->regs->CR2 |= ADC_CR2_SWSTART; // regular channels
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}

void adc_dma_enable(const adc_dev *dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}

void takeSamples() {
  if(FFTMode) {
    // perform DMA, 
    dma_init(DMA1);
    dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);

    adc_dma_enable(ADC1);
    dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_16BITS, data16, DMA_SIZE_16BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT)); // receive buffer DMA
    dma_set_num_transfers(DMA1, DMA_CH1, FFTLEN);
    dma1_ch1_Active = 1;
  
    dma_enable(DMA1, DMA_CH1); // enable the channel and start the transfer.
    uint32_t time_left_in_dma = micros();
  
    while(dma1_ch1_Active){ }; // wait for the DMA to complete
    dma_disable(DMA1, DMA_CH1); // end of trasfer, disable DMA and continuous mode.
    real_to_complex(data16, data32, FFTLEN);
    perform_fft(data32, y, FFTLEN);
    displayBins(y, FFTLEN, LOGSCALE);
  } else {
    // only works for serial upload. doesn't work with DFU mode for some reason
    int analogIn = analogRead(analogInPin);
    displayWave(analogIn);
  }
}

void setup() {
  // enable Serial
  Serial.begin(115200); // only works with serial upload mode
  
  // initialize the display
  display.begin(SSD1306_SWITCHCAPVCC);
  display.clearDisplay();
  display.display();

  // initialize FFT variables
  init_hamming_window(hammingwindow, FFTLEN);
  fill(y,0,FFTLEN);
  fill(data32,1,FFTLEN);
  fill(data16,1,FFTLEN);

  // calibrate and setup ADC
  adc_calibrate(ADC1);
  setADCs();

  // initialize cigar LEDs
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
}

void inplace_magnitude(uint32_t *target, uint16_t len){
  uint16_t * p16;
  for(int i=0; i<len; i++){
     int16_t real = target[i] & 0xFFFF;
     int16_t imag = target[i] >> 16;
     uint32_t magnitude = asqrt(real*real + imag*imag);
     target[i] = magnitude; 
  }
}
  
uint32_t perform_fft(uint32_t *indata, uint32_t *outdata, const int len){
	uint32_t timetaken = micros();
//	window(indata,hammingwindow,len,1); //scaling factor of 4 for 4095> 16 bits
	cr4_fft_1024_stm32(outdata, indata, len);
	inplace_magnitude(outdata, len);
	return micros() - timetaken;
}

// dispaly audio as an amplitude wave
// only works with serial upload mode
void displayWave(uint16_t newAnalog) {
  display.clearDisplay();
  // move previous amplitudes forward
  for(int i=(SCREENWIDTH-1); i>0; i--) {
    display.drawLine(i, (32-ampWave[i-1]), i, (32+ampWave[i-1]), WHITE);
    ampWave[i] = ampWave[i-1];
  }
  // append newest applitude
  int newAmp = newAnalog*SCREENHEIGHT/4096;
  newAmp -= 32;
  display.drawLine(0, 32-newAmp, 0, 32+newAmp, WHITE);
  ampWave[0] = newAmp;
  display.display();
}

// display audio as bin levels after FFT
void displayBins(uint32_t *data, int len, bool scale) {
  uint32_t maxvalue = 1024;
  uint32_t myBins[32] = {0};
  uint32_t LED1Max = 0;
  uint32_t LED2Max = 0;
  uint32_t LED3Max = 0;
  if(scale) {
    // logarithmic x axis
    for(int i=0; i<32; i++) {
      myBins[i] = 0;
      for(int j=0; j<logBinsGap[i]; j++) {
        int newData = data[logBins[i]+j];
        myBins[i] = max(myBins[i], newData);
      }
    }
  } else {
    // linear x axis
    for(int i=0; i<32; i++) {
      myBins[i] = 0;
      for(int j=0; j<16; j++) {
        myBins[i] = max(myBins[i], data[(i*32)+j]);
      }
    }
  }
  display.clearDisplay();
  // we toss the first bin since it's 0Hz
  for(int i=1; i< 32; i++) {
    float FFTBarHeight = myBins[i]*SCREENHEIGHT/maxvalue;
    int startXPos = 4*(31-i);
    display.drawLine(startXPos+0, 0, startXPos+0, (int)FFTBarHeight, WHITE);
    display.drawLine(startXPos+1, 0, startXPos+1, (int)FFTBarHeight, WHITE);
    display.drawLine(startXPos+2, 0, startXPos+2, (int)FFTBarHeight, WHITE);
    display.drawLine(startXPos+3, 0, startXPos+3, (int)FFTBarHeight, WHITE);

    if(i < 11) {
      if(LED1Max < myBins[i]) LED1Max = myBins[i];
    } else {
      if(i < 21) {
        if(LED2Max < myBins[i]) LED2Max = myBins[i];
      } else {
        if(LED3Max < myBins[i]) LED3Max = myBins[i];
      }
    }
  }
  
  display.display();

  // turn cigar LEDs on/off depending on bin levels
  if(LED1Max > LEDThresh) {
    digitalWrite(LED1, HIGH);
  } else {
    digitalWrite(LED1, LOW);
  }
  if(LED2Max > LEDThresh) {
    digitalWrite(LED2, HIGH);
  } else {
    digitalWrite(LED2, LOW);
  }
  if(LED3Max > LEDThresh) {
    digitalWrite(LED3, HIGH);
  } else {
    digitalWrite(LED3, LOW);
  }
}

void loop() { 
  while(1){ // loop seems to run only once, hence the while 
	  takeSamples();
  }
}
