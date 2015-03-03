/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file motor_management.h
 *
 * Definition of the motor management uORB topic.
 */

#ifndef MOTOR_MANAGEMENT_H_
#define MOTOR_MANAGEMENT_H_

#include "../uORB.h"
#include <stdint.h>

/**
 * @addtogroup topics
 * @{
 */

/**
 * Motor management data, tachometer speed, temperatures, fuel level
 */
struct motor_management_s {
	uint64_t	timestamp;		/**< microseconds since system boot, needed to integrate 	*/
	float		ppm_1;			/**< first tach pulses per minute, -1 if unknown			*/
	float		ppm_2;			/**< second tach pulses per minute, -1 if unknown			*/
	float		ppm_3;			/**< third tach pulses per minute, -1 if unknown			*/
	float		ppm_4;			/**< forth tach pulses per minute, -1 if unknown			*/
	float		temp_1;			/**< first temperature transducer volts						*/
	float		temp_2;			/**< second temperature transducer volts					*/
	float		fuel_1;			/**< first fuel sensor reading								*/
	float		fuel_2;			/**< second fuel sensor reading								*/
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(motor_management);

#endif
