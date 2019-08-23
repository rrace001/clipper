/*
 
                                  1-------------------
  SCK`/RX2  TCC01    *     *    5 | A5             A4 | 4    *    *  TCC00 MOSI`/TX2  REF
    MOSI*   TCC02          *    8 | A8 (XIN)       A2 | 2    *    *                   DAC
    SCK*    TCC03          *    9 | A9 (XOUT)     Vdd |
  SDA/MISO*  TC10    *    NMI  14 | A14           Gnd |
   SCL/SS*   TC11    *     *   15 | A15           A25 | 25                    USB/DP
BOOT                           28 | A28/RST       A24 | 24                    USB/DM
SWDCLK  TX1/MISO*              30 | A30           A31 | 31   *            RX1/SS*   SWDIO
                                   -------------------                            ------------------

*/


#define PORTS_HIGH(PORT_XX) REG_PORT_OUTSET0 = PORT_XX
#define PORTS_LOW(PORT_XX) REG_PORT_OUTCLR0 = PORT_XX   
#define PORTS_OUT(PORT_XX) REG_PORT_DIRSET0 = PORT_XX
#define PORTS_IN(PORT_XX) REG_PORT_DIRCLR0 = PORT_XX
#define PORTS_DIGITAL_READ(PORT_XX) REG_PORT_IN0 &= PORT_XX

volatile byte instrument_index = 1;

unsigned int sleep_millis = 12*1000;
byte PIN_BUTTON = 15;
byte PIN_PIEZO = 14;

unsigned int GLCK0_FREQ = 48000000; 

unsigned int PORT_TSP[] = {PORT_PA02, PORT_PA04, PORT_PA05, PORT_PA30, PORT_PA31};
unsigned int PIN_TSP[] = {2, 4, 5, 30, 31};
const byte NUM_TSP_PINS = 5;

byte ORDER_LEDA[] = {0,  1,  0,  2,  1,  2,  0,  3,  1,  3,  2,  3,  0,  4,  1,  4,  2,  4,  3,  4};
byte ORDER_LEDB[] = {1,  0,  2,  0,  2,  1,  3,  0,  3,  1,  3,  2,  4,  0,  4,  1,  4,  2,  4,  3};
const byte NUM_LEDS = 20;
const byte NUM_STROBE_LEDS = 8;

unsigned int PORTS_LED_PAIRS[NUM_LEDS] = {0};
unsigned int PORTS_LED_A[NUM_LEDS] = {0};
unsigned int PORTS_LED_B[NUM_LEDS] = {0};
unsigned int PORTS_MASK_ALL = 0;
unsigned int previous_microseconds = 0;

volatile byte index_led_on = 0;
byte FRAMES_PER_SECOND = 55;
volatile unsigned int frame = 0xFFFFFFFF;

byte led_state[20] = {0};
byte ALPHA_NUM_OFFSET = 8;
byte NUMBER_LEDS[]={
  0b01110111, //0
  0b00010100, //1
  0b01011011, //2
  0b01101011, //3
  0b00101110, //4
  0b01101101, //5
  0b01111101, //6
  0b00100011, //7
  0b01111111, //8
  0b00101111  //9
};
byte ALPHA_LEDS[]={
  0b00111111, //A
  0b01111100, //b
  0b01010101, //C
  0b01111010, //d
  0b01011101, //E
  0b00011101, //F
  0b01111101  //G
};
unsigned int OUTER_RING_MASK = 0xFF;
unsigned int ALPHA_NUM_MASK = 0xFF << ALPHA_NUM_OFFSET;
unsigned int GREEN_HI_MASK = 1<<16;
unsigned int GREEN_LO_MASK = 1<<19;
unsigned int RED_LEFT_MASK = 1<<15;
unsigned int RED_RIGHT_MASK = 1<<17;
unsigned int SHARP_MASK = 1<<18;

unsigned int outer_ring =0;
unsigned int alpha_num = 0;

bool temp = true;
bool button_press = false;

enum states  { auto_tune, manual_tune, deep_sleep };
states state = manual_tune;

unsigned int previous_millis = 0;
unsigned int current_millis = 0;
unsigned int delta_millis = 0;



unsigned int INSTRUMENT_STROBE_PERIODS[] = {
  72809,  54545,  40862,  30612,  24297,  18202
};

byte INSTRUMENT_ALPHAS[]={ 4, 0, 3, 6, 1, 4};
byte INSTRUMENT_ALPHAS_QTY = sizeof(INSTRUMENT_ALPHAS);

volatile byte strobe_index = 0;
volatile byte strobe_values = 0;
volatile bool piezo_previous_hi = 0;
byte piezo_level_threshold_hi = 75JO;
byte piezo_level_threshold_lo = 50;

int8_t qx[] = {1,1,0,-1,-1,-1,0,1};
int8_t qy[] = {0,1,1,1,0,-1,-1,-1};
int8_t S_LEDS[] ={
0,  0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  2,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,  6,  6,  6,  6,  6,  7,  7,  7,  7
};
int8_t XYZ[]={
-7, 1,  2,  3,  13, 12, 11, 22, 21, 31, 10, 20, 30, 29, 19, 9,  8,  7,  -1, -2, -3, -13,  -12,  -11,  -21,  -31,  -10,  -20,  -30,  -29,  -19,  -9, -18,  -8,
};

volatile byte sled = 0;
const byte MEM_QTY = 3;
byte mem_counter = 0;
volatile byte smem[MEM_QTY]={0};

void TCC0_Handler(){  
if(REG_TCC0_INTFLAG & TCC_INTFLAG_CNT){  

//31.8.14 Result
 byte piezo_level = REG_ADC_RESULT;
// 31.8.13 Status
 while (REG_ADC_STATUS & ADC_STATUS_SYNCBUSY);

  if(piezo_level > piezo_level_threshold_hi){//  PORTS_DIGITAL_READ(PORT_PA15)
    strobe_values |= (1 << strobe_index);
    piezo_previous_hi = true;
  } else if (piezo_level < piezo_level_threshold_lo) {
    strobe_values &= ~(1 << strobe_index);
    piezo_previous_hi = false;
  } else {
    if(piezo_previous_hi) strobe_values |= (1 << strobe_index);
    else strobe_values &= ~(1 << strobe_index);
  }
  
  frame&=~OUTER_RING_MASK;
  frame|=strobe_values;

  strobe_index++;
  strobe_index%=NUM_STROBE_LEDS;
 
  REG_TCC0_INTFLAG |= TCC_INTFLAG_CNT;  
  }
}
void TC2_Handler(){
  
  if (TC2->COUNT16.INTFLAG.bit.MC0)             
  {
    PORTS_IN(PORTS_MASK_ALL);  
    if(frame & (1 << index_led_on)){
    PORTS_OUT(PORTS_LED_PAIRS[index_led_on]);
    PORTS_HIGH(PORTS_LED_A[index_led_on]);
    PORTS_LOW(PORTS_LED_B[index_led_on]);  
    }
    index_led_on++;
    index_led_on%=NUM_LEDS;
  TC2->COUNT16.INTFLAG.bit.MC0 = 1;
  }
}
void button_ISR(){
  button_press = true;

  switch (state){
    case auto_tune:
  
      break;
    case manual_tune:
      instrument_index++;
      instrument_index%=INSTRUMENT_ALPHAS_QTY;
      update_TCC0_period(INSTRUMENT_STROBE_PERIODS[instrument_index]);
      frame&=~ALPHA_NUM_MASK;
      alpha_num = ALPHA_LEDS[INSTRUMENT_ALPHAS[instrument_index]] << ALPHA_NUM_OFFSET;    
      frame |= alpha_num;  
      break;
    case deep_sleep:
  
      break;
  }
  
}

 void setup() { // setup setup setup setup setup setup setup setup setup setup setup setup setup setup
  analogReadResolution(8);
  pinMode(PIN_BUTTON,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), button_ISR, RISING);

  pinMode(PIN_PIEZO,INPUT);
  
  for(int i=0; i<NUM_TSP_PINS; i++){
    pinMode(PIN_TSP[i],OUTPUT);
    PORTS_MASK_ALL |= PORT_TSP[i];
  }
  for(int i=0; i<NUM_LEDS; i++){
    PORTS_LED_PAIRS[i] = PORT_TSP[ORDER_LEDA[i]] | PORT_TSP[ORDER_LEDB[i]];
    PORTS_LED_A[i] = PORT_TSP[ORDER_LEDA[i]];
    PORTS_LED_B[i] = PORT_TSP[ORDER_LEDB[i]];
  }

     Serial.begin(115200);
  init_TCC0();
  init_TC2();
//    init_TC1();
//  init_AC();
  init_ADC();
  enable_EIC_sleep();
  
  delay(10000);
  frame = 0;
  frame&=~ALPHA_NUM_MASK;
  alpha_num = ALPHA_LEDS[INSTRUMENT_ALPHAS[instrument_index]] << ALPHA_NUM_OFFSET;    
  frame |= alpha_num;  

}

void loop() { 
  
  switch (state){
    case auto_tune:
  
      break;
    case manual_tune:

    Serial.println(sled);

      if(button_press){
        previous_millis = current_millis;
        button_press = false;
      }
      current_millis = millis();
      delta_millis = current_millis - previous_millis;
      if(delta_millis > sleep_millis) state = deep_sleep;
      
      break;
    case deep_sleep:

      for(int i=0; i<NUM_TSP_PINS; i++){
        pinMode(PIN_TSP[i],INPUT);
      }
      instrument_index=INSTRUMENT_ALPHAS_QTY-1;
      state = manual_tune;
      goto_sleep();
         
      break;
      }
}

void init_TC2(){  // init_TC *** init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***init_TC ***

//  15. PM – Power Manager 
//  16.8.10 APBC Mask - Name:  APBCMASK
  PM->APBCMASK.bit.TC2_=1; // Bit 12 – TC2: TC2 APB Clock Enable : 1 = The APBC clock for the TC2 is enabled.

//14. GCLK - Generic Clock Controller 
//14.8.3 Generic Clock Control
  GCLK->CLKCTRL.reg=0x0; // clear register  
  GCLK->CLKCTRL.bit.GEN=2; // Bits 11:8 – GEN[3:0]: Generic Clock Generator : GENERIC_CLOCK_MULTIPLEXER_FDPLL_32K (2u)
  GCLK->CLKCTRL.bit.ID=0x12; // Bits 5:0 – ID[5:0]: Generic Clock Selection ID : 
  GCLK->CLKCTRL.bit.CLKEN=1; // Bit 14 – CLKEN: Clock Enable 
  //14.8.2 Status
  while (GCLK->STATUS.bit.SYNCBUSY==1);
//  28. TC – Timer/Counter
//  28.8.1 Control A : Name:  CTRLA
  TC2->COUNT16.CTRLA.bit.ENABLE=0; // Bit 1 – ENABLE: Enable : 1 = The peripheral is enabled.
  TC2->COUNT16.CTRLA.bit.MODE=0x0; // Bits 3:2 – MODE[1:0]: Timer Counter Mode : 0x0 = COUNT16
  TC2->COUNT16.CTRLA.bit.WAVEGEN=0x1;//  Bits 6:5 – WAVEGEN[1:0]: Waveform Generation Operation 0x1 MFRQ Match frequency CC0
//  28.8.11 Status
  while (TC2->COUNT16.STATUS.bit.SYNCBUSY==1);  

//  28.8.9 Interrupt Enable Set - Name:  INTENSET
  TC2->COUNT16.INTENSET.bit.MC0 = 1; // Bits 5,4 – MCx: Match or Capture Channel x Interrupt Enable [x = 1..0] 1 = The Match or Capture Channel x interrupt is enabled.

NVIC_EnableIRQ(TC2_IRQn);  // Enable intrupt vector
//  28.8.14 Compare/Capture
  TC2->COUNT16.CC[0].bit.CC = 32768/NUM_LEDS/FRAMES_PER_SECOND;
  TC2->COUNT16.CTRLA.bit.ENABLE=1; // Bit 1 – ENABLE: Enable : 1 = The peripheral is enabled.
//  28.8.11 Status
  while (TC2->COUNT16.STATUS.bit.SYNCBUSY==1);

}


void init_TCC0(){
  
NVIC_EnableIRQ(TCC0_IRQn);

//  15. PM – Power Manager 
//  15.8.11 APBC Mask
  REG_PM_APBCMASK |= PM_APBCMASK_TCC0; //Bit 8 – TCC0: TCC0 APB Clock Enable
  
//14. GCLK - Generic Clock Controller 
//14.8.3 Generic Clock Control - Name:  CLKCTRL
  REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_GEN(GCLK_CLKCTRL_GEN_GCLK0)| // Bits 11:8 – GEN[3:0]: Generic Clock Generator : GCLKGEN0 Generic clock generator 0
                      GCLK_CLKCTRL_ID(GCM_TCC0) |            // Bits 5:0 – ID[5:0]: Generic Clock Selection ID 
                      GCLK_CLKCTRL_CLKEN;                       // Bit 14 – CLKEN: Clock Enable 
//14.8.2 Status
while (REG_GCLK_STATUS & GCLK_STATUS_SYNCBUSY);
// 29.8.1 Control A Name:  CTRLA
  REG_TCC0_CTRLA &=  ~TCC_CTRLA_ENABLE; //Bit 1 – ENABLE: Enable
    while(REG_TCC0_SYNCBUSY&TCC_SYNCBUSY_ENABLE);

// 29.8.17 Waveform Control
  REG_TCC0_WAVE |= TCC_WAVE_WAVEGEN(TCC_WAVE_WAVEGEN_NFRQ);
    while(REG_TCC0_SYNCBUSY&TCC_SYNCBUSY_WAVE);

// 29.8.18 Period  
;

  REG_TCC0_PER = TCC_PER_PER(INSTRUMENT_STROBE_PERIODS[instrument_index]);
    while(REG_TCC0_SYNCBUSY&TCC_SYNCBUSY_PER);
    
// 29.8.12 Interrupt Enable Set
  REG_TCC0_INTENSET |= TCC_INTENSET_MC(TCC_INTENSET_CNT);
    
// 29.8.1 Control A
  REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE; //Bit 1 – ENABLE: Enable 
    while(REG_TCC0_SYNCBUSY&TCC_SYNCBUSY_ENABLE);
  
}

void update_TCC0_period(unsigned int period){
  // 31.8.18 Compare/Capture Channel x  
  REG_TCC0_PER = TCC_PER_PER(period); 
  while(REG_TCC0_SYNCBUSY&TCC_SYNCBUSY_PER);
}

void enable_EIC_sleep(){

//14.8.4 Generic Clock Generator Control
  REG_GCLK_GENCTRL = (GCLK_GENCTRL_RUNSTDBY | 
                      GCLK_GENCTRL_GENEN | 
                      GCLK_GENCTRL_SRC_OSCULP32K | 
                      GCLK_GENCTRL_ID(5));  //source for GCLK5 is OSCULP32K
  while (REG_GCLK_STATUS & GCLK_STATUS_SYNCBUSY); 
//14.8.3 Generic Clock Control
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK5 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK5
  while (REG_GCLK_STATUS & GCLK_STATUS_SYNCBUSY); 
//20.8.9 Wake-Up Enable
  REG_EIC_WAKEUP |= (1 << digitalPinToInterrupt(PIN_BUTTON));
}

void goto_sleep() {  

  USBDevice.detach();
  // Disable systick interrupt:  See https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; 
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  // Enable systick interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  
  USBDevice.attach();
  
}


void init_ADC(){

//14. GCLK - Generic Clock Controller 
//14.8.3 Generic Clock Control - Name:  CLKCTRL
  REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_GEN_GCLK0| // Bits 11:8 – GEN[3:0]: Generic Clock Generator : GCLKGEN0 Generic clock generator 0
                      GCLK_CLKCTRL_ID_ADC |            // Bits 5:0 – ID[5:0]: Generic Clock Selection ID 
                      GCLK_CLKCTRL_CLKEN;                       // Bit 14 – CLKEN: Clock Enable 
//14.8.2 Status
while (REG_GCLK_STATUS & GCLK_STATUS_SYNCBUSY);

// 15. PM – Power Manager 
// 15.8.11 APBC Mask
REG_PM_APBCMASK |= PM_APBCMASK_ADC; // Bit 17 – AC: AC APB Clock Enable; The APBC clock for the AC is enabled.

// Set PA14 as AIN[0] for the AC
//22. PORT - I/O Pin Controller 
//22.8.13 Pin Configuration n
PORT->Group[0].PINCFG[PIN_PIEZO].bit.PMUXEN=0x1; //Bit 0 – PMUXEN: Peripheral Multiplexer Enable : The peripheral multiplexer selection is enabled, and the selected peripheral function controls the direction and output drive value.
//22.8.12 Peripheral Multiplexing n
PORT->Group[0].PMUX[PIN_PIEZO/2].bit.PMUXE=0x1; // Bits 3:0 – PMUXE[3:0]: Peripheral Multiplexing for Even-Numbered Pin : 0x1 B Peripheral function B selected

//31.8.2 Reference Control
REG_ADC_REFCTRL |= ADC_REFCTRL_REFSEL_INTVCC1;

//31.8.5 Control B
REG_ADC_CTRLB |= ADC_CTRLB_RESSEL(ADC_CTRLB_RESSEL_8BIT_Val) |
                 ADC_CTRLB_FREERUN;

//31.8.8 Input Control  //PA14 VDD I 2 C NMI AIN[6] CMP[0] X[0]/Y[6]
REG_ADC_INPUTCTRL |= ADC_INPUTCTRL_MUXNEG(ADC_INPUTCTRL_MUXNEG_IOGND_Val)|
                    ADC_INPUTCTRL_MUXPOS(ADC_INPUTCTRL_MUXPOS_PIN6_Val);

//31.8.1 Control A
REG_ADC_CTRLA |= ADC_CTRLA_ENABLE;                   

  
}
