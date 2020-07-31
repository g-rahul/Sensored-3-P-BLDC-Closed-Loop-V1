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
 * Over Current Protection - Auto Disable/Enable External Circuitry
 *
 * Description: Use CompB and shared reference to determine if Rsense voltage is
 *    >/< current limit threshold. Shared reference is configured to generate 
 *    hysteresis and 32 different reference thresholds are possible. When 
 *    Vrsense exceeds Vref0 CBOUT goes high and when Vrsense drops below Vref1 
 *    then CBOUT goes low. Connect P1.6/CBOUT external ciruitry to handle over 
 *    current condition to either turn off supply or motor current path. 
 *
 *    This allows for automatic enabling and disabling of external circuitry 
 *    during an over current condition without the intervention of device/CPU
 *                                                   
 *                 MSP430x552x
 *             ------------------                        
 *         /|\|                  |                       
 *          | |                  |                       
 *          --|RST      P6.0/CB0 |<---- Voltage @Rsense (represents Motor Current)           
 *            |                  |                                         
 *            |        P1.6/CBOUT|----> 'high'(Vrsesnse > Vref0) 
 *            |                  |      'low'(Vrsesnse < Vref1)
 *            |                  |      Connect to external ciruitry to handle 
 *            |                  |      Over Current Condition in h/w
 *
 *
 *    Note: Vref0 = 1.17V and Vref1 = 0.23V threshold levels selected as an 
 *          example only . The reference threshold should be changed according  
 *          to application requirement.
 *
 *   B.Nisarga
 *   Texas Instruments Inc.
 *   May 2011
 *   Built with CCE v4.0 and IAR Embedded Workbench Version: 4.21
 *******************************************************************************/

#include  "msp430f5529.h"

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;     // Stop WDT                     
 
  P1DIR |= BIT6;                // P1.6 output direction
  P1SEL |= BIT6;                // Select CBOUT function on P1.6

  // Setup ComparatorB                                            
                                               
  CBCTL0 |= CBIPEN + CBIPSEL_0; // Enable V+, input channel CB0              
  CBCTL1 |= CBPWRMD_0;          // CBMRVS=0 => select VREF1 as ref when CBOUT 
                                // is high and VREF0 when CBOUT is low  
                                // High-Speed Power mode        
  CBCTL2 |= CBRSEL;             // VRef is applied to -terminal  
  CBCTL2 |= CBRS_2 + CBREFL_1 + CBREF0_24 + CBREF1_4;   
                                // VREF0 is 1.5V*(24+1)/32 =  1.17V            
                                // VREF1 is 1.5V*(4+1)/32 =  0.23V          
  CBCTL3 |= BIT0;               // Input Buffer Disable @P6.0/CB0    
  CBCTL1 |= CBON;               // Turn On ComparatorB           

  __delay_cycles(75);           // delay for the reference to settle
  
  __bis_SR_register(LPM4_bits); // Enter LPM4
  __no_operation();             // For debug 
} 


