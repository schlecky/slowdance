#include <msp430.h>
#include <msp430g2553.h>

#define S1PIN BIT2
#define S1OUT P2OUT
#define S1DIR P2DIR
#define S2PIN BIT1
#define S2OUT P2OUT
#define S2DIR P2DIR
#define SENPIN BIT0
#define SENOUT P2OUT
#define SENDIR P2DIR

#define LEDPIN BIT5
#define LEDDIR P1DIR
#define LEDOUT P1OUT
#define LEDENPIN  BIT3
#define LEDENOUT  P1OUT
#define LEDENDIR  P1DIR

#define BTNIN P2IN
#define BTNDIR P2DIR
#define BTNOUT P2OUT
#define BTNPIN BIT6
#define BTNREN P2REN
#define BTNIE P2IE
#define BTNIES P2IES
#define BTNIFG P2IFG
#define BTNSEL P2SEL

#define CPU_FREQ 16000000

#define NB_MODES 3
// Fréquences en dHz
unsigned int freqVib=800;
unsigned int freqLed=810;
volatile unsigned int perVib;
volatile unsigned int perLed;
volatile unsigned int perLedOn;
volatile unsigned int perLedOff;

volatile int ledOn=0;
volatile int out=1;
volatile int adc=0;
volatile int doShift;
volatile int phaseShift;
volatile int timer=0;

volatile int btnTimer=0;

int mode = 0;


NAKED(_reset_vector__)
{
	/* place your startup code here */

	/* Make sure, the branch to main (or to your start
	   routine) is the last line in the function */
	__asm__ __volatile__("br #main"::);
}

void updateFreq()
{
  perVib = (10*CPU_FREQ/(freqVib*8));
  perLed = (10*CPU_FREQ/(freqLed*8));
  perLedOn = perLed/10;
  perLedOff = perLed-perLedOn+perLed;
}
 

void toggleSolenoid(){
  static int out=0;
  out=(out==0)?1:0;
  if(out==1){
    S1OUT|=S1PIN;
    S2OUT&=~S2PIN;
  }
  else{
    S1OUT&=~S1PIN;
    S2OUT|=S2PIN;
  }
}

void solenoidOff(){
  S1OUT |= S1PIN;
  S2OUT |= S2PIN;
}

void solenoidOn(){
 if(out==1){
    S1OUT|=S1PIN;
    S2OUT&=~S2PIN;
  }
  else{
    S1OUT&=~S1PIN;
    S2OUT|=S2PIN;
  }
}

void switchLed(int on){
  if(on==0){
    LEDOUT|=LEDPIN;
  }
  else{
    LEDOUT&=~LEDPIN;
  }
}



void triggerADC(){
  ADC10CTL0 |= ENC | ADC10SC ;
}

void readADC(){
  long int val=ADC10MEM;
  static int freqs[50];
  static int currSum = 0;
  static int oldSum = 0;
  static int i = 0;

  freqVib=(int)(600+(500*val)/1023);
  
  currSum = currSum + freqVib - freqs[i];
  freqs[i] = freqVib;
  i = (i+1)%50;


  if(mode == 0){
    freqLed = freqVib+10;
  }
  else if((mode==1) || (mode==2)){
    if(mode==1)
      freqLed = freqVib;
    if(mode==2)
      freqLed = freqVib<<1;

    long int deph = (1<<15)+TACCR1-TACCR2-perVib/2; 
    if(phaseShift==0){
      if((deph%perVib) > perVib/8){
        phaseShift = 10;
        oldSum = currSum;
      } 
  
    /*if((deph%perVib) < -perVib/8){
        phaseShift = -10;
        oldSum = currSum;
      }*/
    }
  }
  updateFreq();
}

void main (void)
{
  WDTCTL = WDTPW + WDTHOLD;        // Stop watchdog timer
  BCSCTL1 = CALBC1_16MHZ;           // run at 16Mhz
  DCOCTL = CALDCO_16MHZ;

  // Solenoid
  S1DIR |= S1PIN;
  S2DIR |= S2PIN;
  SENDIR|= SENPIN;
  SENOUT |= SENPIN;

  // Leds
  LEDDIR |= LEDPIN;
  LEDENDIR |= LEDENPIN;
  LEDENOUT |= LEDENPIN;

  // Button
  BTNDIR &= ~BTNPIN;
  BTNREN |= BTNPIN; // pullup
  BTNOUT |= BTNPIN; // pullup
  BTNIE |= BTNPIN;  // Interrupt enable
  BTNIES |= BTNPIN; // high to low
  BTNSEL &= ~BTNPIN;
  
   
  /*********************/
  /* ADC Configuration */
  /*********************/
  ADC10AE0 = BIT1;
  ADC10CTL1 = 0;
  //ADC10CTL0 = SREF_0 | ADC10ON | ADC10IE | ADC10SHT_3;
  ADC10CTL0 = SREF_0 | ADC10ON  | ADC10SHT_3;
  updateFreq();
  __eint();
  
  WRITE_SR(GIE); 		            //Enable global interrupts
  TACTL = TASSEL_2 | ID_2 | MC_2;  //SMCLK, DIV by 8, count UP
  TACCTL0=CCIE;
  TACCTL1=CCIE;
  TACCTL2=CCIE;
  S2OUT|=S2PIN;
  S1OUT&=~S1PIN;
  freqVib = 700;
  freqLed=freqVib;
  updateFreq();

  triggerADC();
  while(ADC10CTL1 & ADC10BUSY){
  }
  readADC();  

  while(1){
    if(adc){
      adc=0;
      triggerADC();
      while(ADC10CTL1 & ADC10BUSY){
      }
      readADC();  
    }
  }
}




/***********************************
 * TimerA1 CCR1/Overflow Interrupt *
 ***********************************/
__attribute__((interrupt(TIMER0_A1_VECTOR))) void TimerA1(void)
{
  // CCR1
  if(TACCTL1 & CCIFG == CCIFG){
    TACCTL1 &= ~CCIFG;
    if(ledOn){
      TACCR1+=perLedOff;
      if(phaseShift){
        int inc = perVib/10;
        //if(phaseShift<0)
        //  inc = -inc;
        int delta = phaseShift>inc?inc:phaseShift;
        if(delta){
          TACCR1+= delta;
          phaseShift-=delta;
        }
      }
      ledOn=0;
      switchLed(ledOn);
    }
    else{
      TACCR1+=perLedOn;
      ledOn=1;
      switchLed(ledOn);
    }
  } 
  // CCR2
  if(TACCTL2 & CCIFG == CCIFG){
    TACCTL2 &= ~CCIFG;
    TACCR2+=perVib;
    toggleSolenoid();
  } 

}

/***********************************
 * TimerA1 CCR1/Overflow Interrupt *
 ***********************************/
__attribute__((interrupt(TIMER0_A0_VECTOR))) void TimerA0(void)
{
      //triggerADC();
      adc=1;
      if(btnTimer){
      if(--btnTimer==0){
        BTNIFG &= ~BTNPIN;
        BTNIE |= BTNPIN;
      }}
      static int cpt=0;
      if(mode == 1){
        if(cpt++ == 50){
          cpt = 0;
          phaseShift+=perVib;
        }
      }       
}


/***********************************
 * ADC10 Interrupt                 *
 ***********************************/
__attribute__((interrupt(ADC10_VECTOR))) void ADC10(void)
{
  //readADC();
  adc=1;
}


__attribute__((interrupt(PORT2_VECTOR))) void PORT2(void){

    switchLed(1);

  if(BTNIFG & BTNPIN){
    BTNIFG &= ~BTNPIN;
    BTNIE &= ~BTNPIN;
    btnTimer = 10;
    mode = (mode+1)%NB_MODES;
    if(mode>0)
      doShift=1;
  }  

}

