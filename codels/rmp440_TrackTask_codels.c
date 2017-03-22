/*
 * Copyright (c) 2017 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
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
/**
 ** Author: Matthieu Herrb
 ** Date: January 2017
 **/

#include <sys/param.h>
#include <math.h>
#include <stdio.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"


/* --- Task TrackTask --------------------------------------------------- */

static genom_event
pumpSpeedReference(const or_genpos_cart_state *robot,
    const rmp440_cmd_vel *cmd_vel, or_genpos_cart_speed *ref,
    genom_context self)
{
	or_genpos_cart_speed *orders;

	if (cmd_vel->read(self) != genom_ok)
		return rmp440_port_not_found(self);
	orders = cmd_vel->data(self);

	ref->v = orders->v;
	ref->vt = 0;
	ref->w = orders->w;
	ref->vmax = orders->vmax;
	ref->wmax = orders->wmax;
	ref->linAccelMax = orders->linAccelMax;
	ref->angAccelMax = orders->angAccelMax;
	return rmp440_pause_track_main;
}


/* --- Activity Track --------------------------------------------------- */

/** Codel trackStart of activity Track.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_track_main, rmp440_end.
 * Throws rmp440_port_not_found, rmp440_bad_ref,
 *        rmp440_cmd_stop_track, rmp440_motors_off,
 *        rmp440_emergency_stop, rmp440_power_cord_connected.
 */
genom_event
trackStart(const rmp440_cmd_vel *cmd_vel, genom_context self)
{
	if (cmd_vel->read(self) != genom_ok)
		return rmp440_port_not_found(self);
	return rmp440_track_main;
}

/** Codel pumpReference of activity Track.
 *
 * Triggered by rmp440_track_main.
 * Yields to rmp440_pause_track_main, rmp440_end.
 * Throws rmp440_port_not_found, rmp440_bad_ref,
 *        rmp440_cmd_stop_track, rmp440_motors_off,
 *        rmp440_emergency_stop, rmp440_power_cord_connected.
 */
genom_event
pumpReference(const or_genpos_cart_state *robot, rmp440_mode rs_mode,
              const rmp440_cmd_vel *cmd_vel, or_genpos_cart_speed *ref,
              genom_context self)
{

	/* Check if mode changed */
	switch (rs_mode) {
	case rmp440_mode_emergency:
		return rmp440_emergency_stop(self);
	case rmp440_mode_motors_off:
		return rmp440_motors_off(self);
	}

	return pumpSpeedReference(robot, cmd_vel, ref, self);
}

/** Codel smoothStopTrack of activity Track.
 *
 * Triggered by rmp440_end.
 * Yields to rmp440_ether.
 * Throws rmp440_port_not_found, rmp440_bad_ref,
 *        rmp440_cmd_stop_track, rmp440_motors_off,
 *        rmp440_emergency_stop, rmp440_power_cord_connected.
 */
genom_event
smoothStopTrack(const or_genpos_cart_state *robot,
                const rmp440_dynamic_str *dynamics,
                rmp440_mode *rs_mode, or_genpos_cart_speed *ref,
                genom_context self)
{
	printf("rmp440 smoothStopTrack\n");

	/* Set a null speed and stop the tracking */
	/* XXX should compute a decceleration ramp */
	ref->v = 0.0;
	ref->w = 0.0;
	*rs_mode = rmp440_mode_idle;
	return rmp440_ether;
}
