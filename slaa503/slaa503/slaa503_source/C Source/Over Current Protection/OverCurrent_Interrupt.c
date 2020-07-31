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
 * Over Current Condition - Over Current Interrupt Implementation
 *
 * Description: CompB is configured to compare Rsense voltage against internal 
 *    reference threshold. Shared Ref is used to provide the internal reference 
 *    threshold. 32 different reference thresholds are possible. When Rsense
 *    voltage is > threshold voltage, the rising edge of the comp output triggers
 *    the CompB ISR. necessary actions should be taken inside the ISR to handle 
 *    the over-current condition in application
 *
 * Note: The reference threshold level selected in this code is only an example.
 *    The reference threshold should be changed according to application 
 *    requirement
 *
 *                 MSP430x552x
 *             ------------------                        
 *         /|\|                  |                       
 *          | |                  |                       
 *          --|RST      P6.0/CB0 |<-- Rsense Voltage            
 *            |                  |    (Imotor or amplified Imotor voltage)                                     
 *            |                  |
 *
 *   Bhargavi Nisarga
 *   Texas Instruments Inc.
 *   May 2011
 *   Built with CCE v4.0 and IAR Embedded Workbench Version: 4.21
 ******************************************************************************/

#include  "msp430f5529.h"

void main(void)
{
  WDTCTL = WDTPW + WDTHOLD;     // Stop WDT
  P1DIR |= BIT6;                // P1.6/CBOUT output direction
  P1SEL |= BIT6;                // Select CBOUT function on P1.6/CBOUT
  
  // Setup ComparatorB                           
  CBCTL0 |= CBIPEN + CBIPSEL_0; // Enable V+, input channel CB0            
  CBCTL1 |= CBPWRMD_0 + CBMRVS; // High speed mode, CBOUT Rising Edge Trigger         
  CBCTL2 |= CBRSEL;             // VREF is applied to -terminal 
  CBCTL2 |= CBRS_2 + CBREFL_1 + CBREF0_5;    
                                // Shared Ref Voltage (Vcref=1.5V) applied to 
                                // R-ladder 5+1/32
                                // Vref0 = ((5+1)/32)*1.5V = 0.28V
  CBCTL3 |= BIT0;               // Input Buffer Disable @P6.0/CB0    

  __delay_cycles(75);           // delay for the reference to settle
  
  CBINT &= ~(CBIFG + CBIIFG);   // Clear any errant interrupts  
  CBINT  |= CBIE;               // Enable CompB Interrupt on rising edge of CBIFG (CBIES=0)
  CBCTL1 |= CBON;               // Turn On ComparatorB    
  
  __bis_SR_register(LPM4_bits+GIE);    
                                // Enter LPM4 with interrupts enabled
  __no_operation();             // For debug 
}

// Comp_B ISR - Over Current Interrupt
#pragma vector=COMP_B_VECTOR
__interrupt void Comp_B_ISR (void)
{
  // Over Current Condition Detected  
  // Take necessary actions
  __no_operation();             // Place Breakpoint
  CBINT &= ~CBIFG;              // Clear Interrupt flag  
}

  