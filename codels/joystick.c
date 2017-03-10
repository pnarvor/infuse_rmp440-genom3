/*
 * Copyright (c) 2005-2017 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rmp440/rmp440.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"

#include "orMathLib.h"
#include "codels.h"

/*----------------------------------------------------------------------*/

/*
 * joystick axis and buttons assignements
 */
#define V_AXIS 1
#define W_AXIS 0
#define MOVE_BUTTON_SLOW 2
#define MOVE_BUTTON_MIDDLE 0
#define MOVE_BUTTON_FAST 1
#define QUIT_BUTTON 3

/*
 * Convert joystick values to reference speed
 *
 * A square function is used to get more precision at low speeds.
 *
 */
void
getJoystickSpeeds(struct or_joystick_state *joy, rmp440_feedback *data,
    double *v, double *w, double *avmax, double *awmax)
{
	double vmaxw = -2, vmaxv = -2, againw = -2, againv = -2;

#if DEBUG > 1
	printf("%s: %d %d\n", __func__,
	    joy->axes._buffer[V_AXIS], joy->axes._buffer[W_AXIS]);
#endif
	if (joy->buttons._buffer[MOVE_BUTTON_SLOW]) {
		vmaxw = 0.7; vmaxv = 0.5; againw = 0.5; againv = 0.6;
	}
	if (joy->buttons._buffer[MOVE_BUTTON_MIDDLE]) {
		vmaxw = 0.9; vmaxv = 1.3; againw = 0.6; againv = 0.8;
	}
	if (joy->buttons._buffer[MOVE_BUTTON_FAST]) {
		vmaxw = 1.1; vmaxv = 2.0; againw = 0.8; againv = 1.0;
	}
	if (vmaxv > -1) {
		*v = -SIGN(joy->axes._buffer[V_AXIS]) *
		    joy->axes._buffer[V_AXIS] * joy->axes._buffer[V_AXIS]
		  * rmp440_joystick_vgain * vmaxv;
		*w = -SIGN(joy->axes._buffer[W_AXIS]) *
		    joy->axes._buffer[W_AXIS] * joy->axes._buffer[W_AXIS]
		  * rmp440_joystick_wgain * vmaxw;
		if (avmax) *avmax = data->fram_accel_limit * againv;
		if (awmax) *awmax = data->fram_yaw_accel_limit * againw;
	}
	else {
		*v = 0;
		*w = 0;
	}
}

/*
 * Check if done 
 */
bool
joystickQuit(struct or_joystick_state *joy)
{
	if (joy->buttons._buffer[QUIT_BUTTON]) {
		return true;
	}
	return false;
}
