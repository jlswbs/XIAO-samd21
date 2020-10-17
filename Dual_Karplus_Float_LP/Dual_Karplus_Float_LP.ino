// Floating point Dual Karplus-Strong + LPF //

#define SIZE 256
#define OFFSET 16
#define SAMPLERATE 22050

  float out1 = 0;
  float last1 = 0;
  float curr1 = 0;
  float tmp1 = 0;
  float delaymem1[SIZE];
  uint8_t locat1 = 0;
  uint8_t bound1 = SIZE;
  float LPF_Beta1 = 0.0f;

  float out2 = 0;
  float last2 = 0;
  float curr2 = 0;
  float tmp2 = 0;
  float delaymem2[SIZE];
  uint8_t locat2 = 0;
  uint8_t bound2 = SIZE;
  float LPF_Beta2 = 0.0f;
  

float randomf(float minf, float maxf) {return minf + random(1UL << 31)*(maxf - minf) / (1UL << 31);}

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

  for (int i = 0; i < SIZE; i++) delaymem1[i] = randomf(0.000f, 0.999f);
 
  bound1 = random(OFFSET, SIZE);

  LPF_Beta1 = randomf(0.009f, 0.999f);

  delay(240);

  for (int i = 0; i < SIZE; i++) delaymem2[i] = randomf(0.000f, 0.999f);
 
  bound2 = random(OFFSET, SIZE);

  LPF_Beta2 = randomf(0.009f, 0.999f);

  delay(120);
  
}

void TC5_Handler (){
  
  analogWrite(PIN_A0, 511.0f * (out1 + out2));

  delaymem1[locat1++] = out1;
  if (locat1 >= bound1) locat1 = 0;
  curr1 = delaymem1[locat1];
  tmp1 = 0.5f * (last1 + curr1);
  last1 = curr1;
  out1 = out1 - (LPF_Beta1 * (out1 - tmp1));
     
  delaymem2[locat2++] = out2;
  if (locat2 >= bound2) locat2 = 0;
  curr2 = delaymem2[locat2];
  tmp2 = 0.5f * (last2 + curr2);
  last2 = curr2;
  out2 = out2 - (LPF_Beta2 * (out2 - tmp2));

  // Clear the interrupt
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
  
}
