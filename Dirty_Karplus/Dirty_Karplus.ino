// Fixed point Karplus-Strong + LPF + Logistic chaos + XOR drive shaper //

#define SIZE 256
#define OFFSET 16
#define SAMPLERATE 22050

  float r = 3.7f;
  float x = 0.5f;

  int out;
  int last = 0;
  int curr = 0;
  uint16_t delaymem[SIZE];
  uint16_t locat = 0;
  uint16_t bound = SIZE;
  uint16_t xout;
  int accum = 0;
  int lowpass = 0;
  bool shaper = false;


bool tcIsSyncing()
{
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

void resetTc()
{
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing());
    while (TC5->COUNT16.CTRLA.bit.SWRST);
}

void disableTc()
{
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (tcIsSyncing());
}

void configureTc(uint32_t sampleRate)
{
    // Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
    while (GCLK->STATUS.bit.SYNCBUSY);

    resetTc();

    // Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

    // Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    while (tcIsSyncing());
    
    // Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

    // Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing());
}


void setup() {

  pinMode(PIN_A0, OUTPUT);
  analogWriteResolution(10);
  configureTc(SAMPLERATE);
  
}

void loop() {

  shaper = rand()%2;

  lowpass = map(xout, 0, 1023, 7, 0);

  for (int i = 0; i < SIZE; i++){
    
    float nx = x;
    x = r * nx * (1.0f - nx);
      
    xout = 1023.0f * x;  
    delaymem[i] = xout;
    
  }

  bound = map(xout, 0, 1023, OFFSET, SIZE);

  delay (160);
  
}

void TC5_Handler (){
  
  analogWrite(PIN_A0, out);

  if (shaper)delaymem[locat++] = out^xout;
  else delaymem[locat++] = out%xout;

  if (locat >= bound) locat = 0;
    
  curr = delaymem[locat];
    
  out = accum >> lowpass;
    
  accum = accum - out + ((last>>1) + (curr>>1));

  last = curr;  
  
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  
}
