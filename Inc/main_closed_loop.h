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
 *  main_closed_loop Header File.
 *  Description: This header file includes user configurable definitions for 
 *    open loop motor control. 
 *
 * Bhargavi Nisarga 
 * Texas Instruments Inc. 
 * Dec 2010
 * Built with IAR Version:4.2

********************************************************************************/

//****************************************************************************//
// Clock Timing Definitions
//****************************************************************************//

#define SYSTEM_FREQ 72000               // (in kHz) System Freq, Timer PWM Time Base
#define PWM_FREQ 17578                  // (in Hz) PWM Freq for switching the H-Bridge FETs 
#define TIMER_PWM_PERIOD  4096
                                        // Timer PWM Counts = Sytem Freq/(2xPWM Freq)

#define SPEEDIN_PWM_FACTOR  1           // Calculate this value and define the constant 
                                        // = 4096/TIMER_PWM_PERIOD, rounded off to the closest whole digit

#define DUTYCYCLE_MIN 10                 // (in %age) Min Dutycycle percentage - latching percentage used initially
#define MIN_PWM_DUTYCYCLE 410
                                        // Min PWM Dutycycle; also used for configuring initial dutycycle 

#define ADC_SAMPLING_PWM_PERIODS 2000   // Every #PWM periods when ADC is sampled

//****************************************************************************//
// Motor Speed Control - Close Loop Definitions
//****************************************************************************//

#define TIMER_COUNTER_FREQ 4000          // in kHz ---- not required here???? CHECK

#define NUM_MOTOR_POLES   8             // # of motor poles
#define MAX_MECH_SPEED_IN_RPM  3000     // Max Motor Speed in rpm
#define MAX_ELEC_SPEED_IN_RPS  (MAX_MECH_SPEED_IN_RPM/60)*(NUM_MOTOR_POLES/2)
                                        // Max num of electrical rotations per second
#define MAX_HALL_ISRs_1SEC  (MAX_ELEC_SPEED_IN_RPS*6)
                                        // Max num of commutation steps or num of Hall/Port ISR entries in 1sec
//#define MIN_PWM_COUNTS_PER_COMM  416  // Compute and enter: ((TIMER_COUNTER_FREQ*1000)/MAX_HALL_ISRs_1SEC)
                                        // Max num of Timer PWM counts per commutation period or between two Hall/Port ISRs

#define PID_EXECUTE_PWM_PERIODS 10      // #PWM periods PID Control Loop is Executed

#define LOW_DUTYCYCLE_PERCENTAGE  50    // %age dutycycle below which different Kp, Ki values are used
                                        // This value is usually close to the first increment of PWM dutycycle during the
                                        // min-max step response (refer excel document)
#define LOW_KP_KI_DUTYCYCLE (((unsigned long)(TIMER_PWM_PERIOD)*(unsigned int)(LOW_DUTYCYCLE_PERCENTAGE)))/100

//****************************************************************************//
// Motor Speed Input Defintions
//****************************************************************************//

#define ANALOG_SPEEDIN
//#define PWM_SPEEDIN

//****************************************************************************//
// Motor Status Definitions
//****************************************************************************//

#define Stopped 0x00
#define StartUp 0x01
#define Running 0x02

//#define DIRECTION_CCW 
#define DIRECTION_CW

#define true 0x01
#define false 0x0


