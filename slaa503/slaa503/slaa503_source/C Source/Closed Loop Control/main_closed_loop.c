/*******************************************************************************
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *  main_closed_loop C Source.
 *  Description: This firmware configures MSP430 as 3-ph BLDC motor controller. 
 *              Closed Loop control is implemented. Port1 GPIO are used as hall 
 *              inputs and are used to detect the rotor state. TimerB outputs 6 
 *              PWM signals that control the motor drive circuit. Speed Input 
 *              is provided by a pot via ADC channel A14. TBISR is used as heart 
 *              beat interval function. PI controller is implemented to provide
 *              closed loop control and TimerA is used to track the interval 
 *              between successive hall events in order to track the actual 
 *              motor speed 
 *
 * 			
 *                 MSP430F552x
 *             -----------------
 *         /|\|              XIN|-
 *          | |                 | 
 *          --|RST          XOUT|-
 *            |                 |            (NFETs)
 *            |            TB0.1|--> PWM1 --> LS_U
 *    Speed   |            TB0.2|--> PWM2 --> HS_U
 *    Input-> |A14         TB0.3|--> PWM3 --> LS_V
 *            |            TB0.4|--> PWM4 --> HS_V
 *            |            TB0.5|--> PWM5 --> LS_W
 *            |            TB0.6|--> PWM6 --> HS_W
 *            |                 |
 *            |             P1.3|<-- Hall U  | Hall  
 *            |             P1.2|<-- Hall V  | Sensor
 *            |             P1.1|<-- Hall W  | Inputs
 *            |                 |
 *
 * ACLK = REF0, MCLK = SMCLK = 16MHz
 * TimerB clock = SMLCK;
 *
 * Bhargavi Nisarga 
 * Texas Instruments Inc. 
 * Dec 2010
 * Built with IAR Version:4.2 and CCS v4.2

********************************************************************************/

#include <msp430f5529.h>
#include "main_closed_loop.h"
#include "..\F5xx_F6xx_Core_Lib\HAL_PMM.h"
#include "..\F5xx_F6xx_Core_Lib\HAL_UCS.h"
#include "hall_sensor.h"

/* Relation between the hall sequence 000-111 and the HS/LS pedriver 
   o/p - THIS IS FIXED for 120deg 
*/
#ifdef DIRECTION_CCW                                        
  // Direction = 0x01 
unsigned char Hall_DIR_sequence[] = { 0x00,            
                                      HS_W|LS_V,       // Hall position 001
                                      HS_V|LS_U,       // Hall position 010
                                      HS_W|LS_U,       // Hall position 011
                                      HS_U|LS_W,       // Hall position 100
                                      HS_U|LS_V,       // Hall position 101
                                      HS_V|LS_W,       // Hall position 110
                                        0x00   };
#endif

#ifdef DIRECTION_CW
  // Direction = 0x00 
unsigned char Hall_DIR_sequence[] = {  0x00,          
                                      HS_V|LS_W,       // Hall position 001
                                      HS_U|LS_V,       // Hall position 010
                                      HS_U|LS_W,       // Hall position 011
                                      HS_W|LS_U,       // Hall position 100
                                      HS_V|LS_U,       // Hall position 101
                                      HS_W|LS_V,       // Hall position 110
                                        0x00   };
#endif

// Motor and Commutation Variables
unsigned int Desired_PWM_DutyCycle, PWM_Update_Counter, ADC_Sample_Counter;
unsigned char PreDriver_Sequence, Hall_IN, Motor_Status, Hall_State_Unknown;
unsigned char Motor_status;

// ADC Variables
unsigned long ADC_Results[4];
unsigned int Avg_vBUS, Avg_vPOT;
unsigned char SampleADC;

// Closed Loop Variables
unsigned char PID_Execute_Counter, ExecutePID;
unsigned int Expected_Hall_ISRs_1sec;
unsigned int Expected_COMM_PWM_Counts, Measured_COMM_PWM_Counts, JJ[100],j=0;
unsigned int Kp, Ki;
signed int s16_Proportional_Error, s16_Integral_Error, s16_PID_ControlLoop_Output;

// Function definition
void PWM_update (unsigned char Next_Hall_Sequence);
void Start_Motor(void);
void Stop_Motor(void);
void Start_ADC_Conversion(void);

void main (void)
{
  volatile unsigned int i;
  unsigned int PID_Applied_PWM_DutyCycle;

  WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
	
  // Configure System Clock = DCO = 16Mhz
  // Increase Vcore setting to level2 to support fsystem=16MHz
  SetVCore(PMMCOREV_2);                     // Set VCore=2 for 16MHz clock
  
  SELECT_FLLREF(SELREF__REFOCLK);           // Set DCO FLL reference = REFO
  SELECT_ACLK(SELA__REFOCLK);               // Set ACLK = REFO
  
  Init_FLL_Settle(16000, 488);    

  // Configure Port pins (P1.0-P1.3) as interrupt capable input pins, Fault IN 
  // and Hall Sensor Inputs
  P1DIR &= ~(BIT0+BIT1+BIT2+BIT3);          // Input DIR
  P1IES = 0x0;			            // Trigger on rising edge
  P1IFG = 0x0;
   
  // Configure TimerB PWMs  
  // Configure Port I/O as Timer PWM output pins 
  P3DIR |= BIT5 + BIT6;
  P3SEL |= BIT5 + BIT6;
  P5DIR |= BIT7;
  P5SEL |= BIT7;
  P7DIR |= BIT4 + BIT5 + BIT6;
  P7SEL |= BIT4 + BIT5 + BIT6;
  
  // Configure TimerB
  TBCTL = TBSSEL_2 + TBCLR;                 // SMCLK, clear TBR    
  
  // HSide NMOS (TB0.2/4/6) => PWM (set/reset); LSide NMOS => logic 'high'
  TBCCR0 = TIMER_PWM_PERIOD-1;              // PWM Period 
  TBCCTL1 = OUTMOD_0;                       // OUT mode and Low                     
  TBCCTL2 = OUTMOD_7;                       // CCR2 reset/set
  TBCCTL3 = OUTMOD_0;                       // OUT mode and Low                     
  TBCCTL4 = OUTMOD_7;                       // CCR4 reset/set
  TBCCTL5 = OUTMOD_0;                       // OUT mode and Low                     
  TBCCTL6 = OUTMOD_7;                       // CCR6 reset/set

  // Initialize PWM outputs with min dutycycle value
  TBCCR2 = MIN_PWM_DUTYCYCLE;
  TBCCR4 = MIN_PWM_DUTYCYCLE;
  TBCCR6 = MIN_PWM_DUTYCYCLE;
  
 
#ifdef ANALOG_SPEEDIN
  // Configure ADC12 to read Analog pot input values at A14 and Vbus at A13
  P7SEL |= BIT2;                            // P7.2/A14 ADC option select
  
  //Init_ADC()
  REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to 
                                            // ADC12_A ref control registers    
  ADC12CTL0 = ADC12SHT0_2+ADC12ON+ADC12MSC; // 16x ADCCLK, ADC12 on
  ADC12MCTL0 = ADC12INCH_13+ADC12SREF_0;    // channel = A13, Vr+=AVCC and Vr-=AVss
  ADC12MCTL1 = ADC12INCH_14+ADC12SREF_0+ADC12EOS; // channel = A14, Vr+=Vref+ and Vr-=AVss, end of sequence
  ADC12CTL1 = ADC12SHP+ADC12CONSEQ_3;       // Use sampling timer, repeated seq of channels
  for (i=0; i<0x30; i++);                   // Delay for reference start-up

#endif 

  /*
  // Check with Vbus is good to proceed with motor start
  if (Avg_vBUS < 0xFF)
  {
      // Wait until vBUS increases 3.3V?
    
  }
  */
  
  // Configure Timer TA0 as Counter with same timer base as PWM timer
  TA0CTL = TASSEL_2 + ID_2 + TACLR;         // SMCLK/4, Timer clear 
  
  // Init Variables
  ExecutePID = false;                       
  PID_Execute_Counter = 0x0;
  Measured_COMM_PWM_Counts = 0x0;
  Expected_COMM_PWM_Counts = 0x0;
  
  s16_Proportional_Error = 0x0;
  s16_Integral_Error = 0x0;
  s16_PID_ControlLoop_Output = 0x0;
  
  Motor_Status = Stopped;  
  Hall_State_Unknown = true;
  SampleADC = false; 

  // Start Motor
  if ((Motor_Status == Stopped)&&(Hall_State_Unknown == true))
    Start_Motor(); 
	
  __enable_interrupt();                                 // enable interrupts   
 
  while(1)
  {
      if(SampleADC == true)
      {
          // Trigger ADC Sampling
          #ifdef ANALOG_SPEEDIN
              Start_ADC_Conversion();  
                              
              __bis_SR_register(LPM0_bits + GIE);       // Enter LPM4, Enable interrupts
              __no_operation();                         // For debugger  
                              
              Avg_vBUS = ADC_Results[0] >> 2;
              Avg_vPOT = ADC_Results[1] >> 2;  
              Desired_PWM_DutyCycle = Avg_vPOT/SPEEDIN_PWM_FACTOR;       
                                                        // ADC12 => 4096 ADC counts = 100% speed
                                                        // 100% Dutycyle counts = TIMER_PWM_PERIOD
                                                        // ADC Pot Counts/SPEEDIN_PWM_FACTOR = Dutycycle counts required              
          #endif  
              
          if (Desired_PWM_DutyCycle < MIN_PWM_DUTYCYCLE)    // if < Min DutyCycle %age - latch to min value, 1023           
              Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;       

          // Apply Kp, Ki values depending on desired dutycycle
          if (Desired_PWM_DutyCycle < LOW_KP_KI_DUTYCYCLE)  // Dutycycle threshold which requires different Kp/Ki values
          {
              Kp = 500;
              Ki = 0;
          }
          else
          {
              Kp = 4000;
              Ki = 1;
          }
          
          SampleADC = false;
          
          // Since Expected_COMM_PWM_Counts change only when ADC input value that is read changes, 
          // the expected value is computed in main while loop
          Expected_Hall_ISRs_1sec = ((unsigned long)(MAX_HALL_ISRs_1SEC)* (unsigned long)(Desired_PWM_DutyCycle))/(TIMER_PWM_PERIOD);          
          Expected_COMM_PWM_Counts = (unsigned long)(TIMER_COUNTER_FREQ)*(unsigned long)(1000)/(Expected_Hall_ISRs_1sec);
                                              // Expected PWM counts based on max speed and speed input         
      }   

      if((ExecutePID == true) && (Measured_COMM_PWM_Counts!=0))
      {
          // PID Motor Speed Control Loop 
            
          // Compute Proportional and Integral Errors
          s16_Proportional_Error = Expected_COMM_PWM_Counts - Measured_COMM_PWM_Counts; // signed int P error = desired-actual
          s16_Integral_Error += s16_Proportional_Error;                                 // signed int I error = Accumulate P errors
              
          s16_PID_ControlLoop_Output = ((((long)Kp*(long)s16_Proportional_Error) + (((long)Ki*(long)s16_Integral_Error))) >> 14)/3; 
                                                                            // PID Control Loop Equation; Kd = 0
              
          PID_Applied_PWM_DutyCycle = Desired_PWM_DutyCycle + (s16_PID_ControlLoop_Output>>3);
          JJ[j++] = PID_Applied_PWM_DutyCycle;
          if(j>100)
            j=0;
              
          if (PID_Applied_PWM_DutyCycle < MIN_PWM_DUTYCYCLE)   // 1023      
          {
              PID_Applied_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;   // < Min DutyCycle %age - latch to min value, 1023   
              s16_Integral_Error = 0x0;
          }
          if (PID_Applied_PWM_DutyCycle > TIMER_PWM_PERIOD)   // 1023
          {
              PID_Applied_PWM_DutyCycle = TIMER_PWM_PERIOD;   // > Max PWM DutyCycle - latch to 100% or max value, 1023   
              s16_Integral_Error = 0x0;
          }
          
          // Initialize PWM outputs with initial dutycycle counts
          TBCCR2 = PID_Applied_PWM_DutyCycle;
          TBCCR4 = PID_Applied_PWM_DutyCycle;
          TBCCR6 = PID_Applied_PWM_DutyCycle;     
          
          ExecutePID = false;
      }
  }
}

void Start_Motor(void)
{
    // Read Speed Input and update dutycycle variable
#ifdef ANALOG_SPEEDIN
    Start_ADC_Conversion();  
    
    __bis_SR_register(LPM0_bits + GIE);       // Enter LPM4, Enable interrupts
    __no_operation();                         // For debugger  
    
    Avg_vBUS = ADC_Results[0] >>2;
    Avg_vPOT = ADC_Results[1] >>2;  
    Desired_PWM_DutyCycle = Avg_vPOT >> 2;    // ADC12 => 4096 ADC counts = 100% speed
                                              // 100% Dutycyle counts = 1066
                                              // ADC Pot Counts/4 = Dutycycle counts required    
#endif  
    
    if (Desired_PWM_DutyCycle < MIN_PWM_DUTYCYCLE)  // if < Min DutyCycle %age - latch to min value, 1023           
        Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;  
    
    if (Desired_PWM_DutyCycle < LOW_KP_KI_DUTYCYCLE)   // Dutycycle threshold which requires different Kp/Ki values
    {
        Kp = 500;
        Ki = 1;
    }
    else
    {
        Kp = 4000;
        Ki = 1;
    }
    // Since Expected_COMM_PWM_Counts change only when ADC input value that is read changes, 
    // the expected value is computed in main while loop
    Expected_Hall_ISRs_1sec = ((unsigned long)(MAX_HALL_ISRs_1SEC)* (unsigned long)(Desired_PWM_DutyCycle))/(TIMER_PWM_PERIOD);          
    Expected_COMM_PWM_Counts = (unsigned long)(TIMER_COUNTER_FREQ)*(unsigned long)(1000)/(Expected_Hall_ISRs_1sec);
                                              // Expected PWM counts based on max speed and speed input         
      
    // Read Hall inputs
    Hall_IN = P1IN;
    Hall_IN = ((Hall_IN & 0x0E)>>1);

    // Start PWM TimerB
    PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
    PWM_update(PreDriver_Sequence);
    P1IFG = 0x0;                        // Clear IFGs
    P1IE |= BIT1+BIT2+BIT3;             // Enable Port ISRs
   
    TBCCTL0 = CCIE;                     // TBCCR0 interrupt enabled    
    TBCTL |=  MC_1;                     // Timer Start - UP mode 

    TA0CTL |= MC_2 + TACLR;             // Start Counter Timer 
    Motor_Status = Running;
}

void Stop_Motor(void)
{
    P1IFG = 0x0;			// Clear Pending interrupts
    TBCCTL0 &= ~CCIE;                   // TBCCR0 interrupt enabled
    TBCTL &= ~(MC_3);			// Stop Pre-Driver outputs
    P1IE &= ~(BIT1+BIT2+BIT3);		// Disable Hall Interrupts    
    TA0CTL &= ~MC_2;                    // Stop Counter Timer    
    Motor_Status = Stopped;	
}
 
// Timer B0 interrupt service routine
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB1_ISR(void)
{       
    // heart beat signal = PWM period
    PID_Execute_Counter++;   
    ADC_Sample_Counter++;

    if(PID_Execute_Counter > PID_EXECUTE_PWM_PERIODS)
    {
        ExecutePID = true;
        PID_Execute_Counter = 0x0;
    }
       
    if(ADC_Sample_Counter > ADC_SAMPLING_PWM_PERIODS)
    {
        ADC_Sample_Counter = 0x0; 
        SampleADC = true;  
    }    
}

// Port1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
        switch(__even_in_range(P1IV,14))
	{
		case 2: // Fault Input - Stop Motor
                    Stop_Motor();                   		    
		    break;
		case 4:
		case 6:
		case 8: 
                    TA0CTL &= ~MC_2;                            // Stop Timer Counter -- is it required????????
                    Measured_COMM_PWM_Counts = (Measured_COMM_PWM_Counts + TA0R)>>1;       
                                                                // Accumulate and avg measured PWM counts per commutation cycle with prev value
                    TA0CTL |= MC_2 + TACLR;                     // Start Timer Again
                    
                    // Hall input interrupts
	            P1IE &= ~(BIT1+BIT2+BIT3);              // Disable Port ISRs -- is it required????????
			    
		    // Read Hall inputs
		    Hall_IN = P1IN;
		    Hall_IN = ((Hall_IN & 0x0E)>>1);
			    
    	            PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];  
			    
                    // Start PWM TimerB			    
                    PWM_update(PreDriver_Sequence);			    
                    P1IFG = 0x0;                             // Clear IFGs - added........			    
                    P1IE |= BIT1+BIT2+BIT3;                  // Enable Port ISRs  -- is it required???????				
                    break;		
		default: break;
	}    
} 

/*
 PxIES = bit 0 => low to high transition capture (rising)
       = bit 1 => high to low transition capture (falling)
   		 
   Hall_U=> P1.3; Hall_V=> P1.2; Hall_W=> P1.1
   HS3=> P1.3; HS2=> P1.2; HS1=> P1.1 
   
   PxIES "not the" same for both DIR0 and DIR1 (as the LUT picks 
   different PreDriver sequence for completing commutation)
   Add if(Direction){} for P1IES bit setting - done
*/

void PWM_update (unsigned char Next_Hall_Sequence)
{
  //Hall_U=> P1.3; Hall_V=> P1.2; Hall_W=> P1.1
  //HS3=> P1.3; HS2=> P1.2; HS1=> P1.1 
      switch(Next_Hall_Sequence)
      {
      case HS_W|LS_V:            // Hall_IN DIR1_001 DIR0_110
        // Next edge => HS3 rising, HS2 rising, HS1 falling DIR1
        // Next edge => HS3 falling, HS2 falling, HS1 rising DIR0  
        #ifdef DIRECTION_CCW
           P1IES = BIT1;         // DIR1
        #else
           P1IES = BIT2+BIT3;    // DIR0
        #endif
        
        TBCCTL1 &= ~OUT;
        TBCCTL2 = OUTMOD_0;
        TBCCTL3 |= OUT;
        TBCCTL4 = OUTMOD_0;
        TBCCTL5 &= ~OUT;   
        TBCCTL6 = OUTMOD_7;   
        break;
        
      case HS_V|LS_U:           // Hall_IN DIR1_010 DIR0_101
        // Next edge => HS3 rising, HS2 falling, HS1 rising DIR1
        // Next edge => HS3 falling, HS2 rising, HS1 falling DIR0
        #ifdef DIRECTION_CCW
           P1IES = BIT2;        // DIR1
        #else
           P1IES = BIT1+BIT3;   // DIR0
        #endif
           
        TBCCTL1 |= OUT; 
        TBCCTL2 = OUTMOD_0;
        TBCCTL3 &= ~OUT;     
        TBCCTL4 = OUTMOD_7;  
        TBCCTL5 &= ~OUT;    
        TBCCTL6 = OUTMOD_0;        
        break;
    
      case HS_W|LS_U:            // Hall_IN DIR1_011 DIR0_100
        // Next edge => HS3 rising, HS2 falling, HS1 falling DIR1
        // Next edge => HS3 falling, HS2 rising, HS1 rising DIR0 
        #ifdef DIRECTION_CCW
           P1IES = BIT1+BIT2;    // DIR1
        #else
           P1IES = BIT3;         // DIR0
        #endif
           
        TBCCTL1 |= OUT;  
        TBCCTL2 = OUTMOD_0;
        TBCCTL3 &= ~OUT;  
        TBCCTL4 = OUTMOD_0;
        TBCCTL5 &= ~OUT; 
        TBCCTL6 = OUTMOD_7;         
        break;
    
      case HS_U|LS_W:            // Hall_IN CCW_100 CW_011
        // Next edge => HS3 falling, HS2 rising, HS1 rising DIR1
        // Next edge => HS3 rising, HS2 falling, HS1 falling DIR0
        #ifdef DIRECTION_CCW
           P1IES = BIT3;         // DIR1
        #else
           P1IES = BIT1+BIT2;    // DIR0
        #endif        
        TBCCTL1 &= ~OUT;
        TBCCTL2 = OUTMOD_7;  
        TBCCTL3 &= ~OUT;    
        TBCCTL4 = OUTMOD_0;
        TBCCTL5 |= OUT;     
        TBCCTL6 = OUTMOD_0;              
        break;
    
      case HS_U|LS_V:            // Hall_IN CCW_101 CW_010
        // Next edge => HS3 falling, HS2 rising, HS1 falling DIR1
        // Next edge => HS3 rising, HS2 falling, HS1 rising DIR0
        #ifdef DIRECTION_CCW
           P1IES = BIT1+BIT3;    // DIR1
        #else
           P1IES = BIT2;         // DIR0
        #endif        
        TBCCTL1 &= ~OUT; 
        TBCCTL2 = OUTMOD_7;
        TBCCTL3 |= OUT;     
        TBCCTL4 = OUTMOD_0;
        TBCCTL5 &= ~OUT;   
        TBCCTL6 = OUTMOD_0;        
        break;
    
      case HS_V|LS_W:            // Hall_IN CCW_110 CW_001
        // Next edge => HS3 falling, HS2 falling, HS1 rising DIR1
        // Next edge => HS3 rising, HS2 rising, HS1 falling DIR0
        #ifdef DIRECTION_CCW
           P1IES = BIT2+BIT3;    // DIR1
        #else
           P1IES = BIT1;         // DIR0
        #endif        
        TBCCTL1 &= ~OUT;
        TBCCTL2 = OUTMOD_0;
        TBCCTL3 &= ~OUT;
        TBCCTL4 = OUTMOD_7;    
        TBCCTL5 |= OUT;     
        TBCCTL6 = OUTMOD_0;         
        break;
        
      default:
        Hall_State_Unknown = true;
        Stop_Motor();
        break;
      }
  //TBR = TIMER_PWM_PERIOD-1;
  //TBCCR0 = TIMER_PWM_PERIOD-1;
  Hall_State_Unknown = false;
}    

void Start_ADC_Conversion(void)
{
  ADC_Results[0] = 0x0;
  ADC_Results[1] = 0x0;
  ADC12IE = 0x02;                           // Enable interrupt, ADC12IFG.1 (MCTL1)
  ADC12CTL0 |= ADC12ON + ADC12ENC;          // Enable conversion
  ADC12CTL0 |= ADC12SC;                     // Start conversion  
}

#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
{
  static unsigned char index = 0;

  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6: break;                           // Vector  6:  ADC12IFG0
  case  8:                                  // Vector  8:  ADC12IFG1
    ADC_Results[0] += ADC12MEM0;            // Move results
    ADC_Results[1] += ADC12MEM1;
    index++;                                // Increment results index
    if (index == 4)
    {
      ADC12IE = 0x0;
      ADC12CTL0 &= ~ADC12ON;
      index = 0;
      __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
    }      
    break;  
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14

  default: break; 
  }
}

