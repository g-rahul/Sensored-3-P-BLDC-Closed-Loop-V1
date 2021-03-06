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
 *  hall_sensor Header File.
 *  Description: This header file includes the hall commutation states with 
 *          respect to timer PWM outputs.
 *
 * Bhargavi Nisarga 
 * Texas Instruments Inc. 
 * Dec 2010
 * Built with IAR Version:4.2

********************************************************************************/

// Hall Commuttaion States mapped to respective TimerB PWM Outputs
/*#define LS_U 	0x01     	//A1
#define HS_U 	0x02     	//A9
#define LS_V 	0x04     	//A2
#define HS_V 	0x08     	//A10
#define LS_W 	0x10     	//A3
#define HS_W 	0x20     	//A11*/
#define HS_V_LS_W 0x01       // Hall position 001
#define HS_U_LS_V 0x02      // Hall position 010
#define HS_U_LS_W 0x03      // Hall position 011
#define HS_W_LS_U 0x04      // Hall position 100
#define HS_V_LS_U 0x05     // Hall position 101
#define HS_W_LS_V 0x06     // Hall position 110

// Motor Direction Defintions
#define CW 	0x01		// clockwise direction 
//#define CCW 	0x00		// counter clockwise direction




