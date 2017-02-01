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

#include <math.h>
#include <stdio.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"


/* --- Task TrackTask --------------------------------------------------- */

static void
pumpSpeedReference(const or_genpos_cart_state *robot,
    const rmp440_cmd_vel *cmd_vel, or_genpos_cart_ref *ref,
    genom_context self)
{
	or_genpos_cart_speed *orders = cmd_vel->data(self);

	/* Transmettre la consigne */
	ref->backFlag = (orders->v > 0 ?
	    or_genpos_forward_motion : or_genpos_backward_motion);
	ref->v = fabs(orders->v);
	/* printf("pumpSpeedRef: %lf\n", ref->v); */
	ref->vt = 0;
	ref->w = orders->w;
	ref->vmax = orders->vmax;
	ref->wmax = orders->wmax;
	ref->linAccelMax = orders->linAccelMax;
	ref->angAccelMax = orders->angAccelMax;
	ref->dataType = or_genpos_speed_data;
	return;
}


/* --- Activity Track --------------------------------------------------- */

/** Codel trackStart of activity Track.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_main, rmp440_end.
 * Throws rmp440_poster_not_found, rmp440_bad_ref,
 *        rmp440_cmd_stop_track, rmp440_motors_off,
 *        rmp440_emergency_stop, rmp440_power_cord_connected.
 */
genom_event
trackStart(const rmp440_cmd_vel *cmd_vel, or_genpos_track_mode mode,
           genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_main;
}

/** Codel pumpReference of activity Track.
 *
 * Triggered by rmp440_main.
 * Yields to rmp440_pause_main, rmp440_end.
 * Throws rmp440_poster_not_found, rmp440_bad_ref,
 *        rmp440_cmd_stop_track, rmp440_motors_off,
 *        rmp440_emergency_stop, rmp440_power_cord_connected.
 */
genom_event
pumpReference(const or_genpos_cart_state *robot, rmp440_mode rs_mode,
              or_genpos_track_mode track_mode,
              const rmp440_cmd_vel *cmd_vel, or_genpos_cart_ref *ref,
              genom_context self)
{
	/* Check if mode changed */
	switch (rs_mode) {
	case rmp440_mode_emergency:
		return rmp440_emergency_stop(self);
	case rmp440_mode_motors_off:
		return rmp440_motors_off(self);
	}

	switch (track_mode) {
#ifdef notyet
	case or_genpos_track_pos:
		return pumpConfigReference(robot, cmd_vel, ref, self);
	case or_genpos_track_config:
		return pumpConfigReference(robot, cmd_vel, ref, self);
#endif
	case or_genpos_track_speed:
		pumpSpeedReference(robot, cmd_vel, ref, self);
		return rmp440_pause_main;
	default:
		return  rmp440_pause_main;

	}
}

/** Codel smoothStopTrack of activity Track.
 *
 * Triggered by rmp440_end.
 * Yields to rmp440_ether.
 * Throws rmp440_poster_not_found, rmp440_bad_ref,
 *        rmp440_cmd_stop_track, rmp440_motors_off,
 *        rmp440_emergency_stop, rmp440_power_cord_connected.
 */
genom_event
smoothStopTrack(const or_genpos_cart_state *robot,
                const rmp440_dynamic_str *dynamics,
                rmp440_mode *rs_mode, or_genpos_track_mode *track_mode,
                or_genpos_cart_ref *ref, genom_context self)
{
	double dtArret;                /* Temps d'arret */
	double dtRetard;               /* Temps de prise en compte de la consigne */
	double dsArret;                /* Longueur d'arret */
	double dtLoco;                 /* Periode d'asserv de la loco */
	double angle;                  /* Moyenne entre angles d'init et d'arret */
	double theta;

	printf("rmp440 smoothStopTrack\n");

	if (*track_mode == or_genpos_track_speed) {
		/* Set a null speed and stop the tracking */
		track_mode = or_genpos_no_tracking;
		ref->dataType = or_genpos_speed_data;
		ref->x = robot->xRob;
		ref->y = robot->yRob;
		ref->theta = robot->theta;
		ref->v = 0.0;
		ref->w = 0.0;
	} else {
		ref->dataType = or_genpos_pos_and_speed_data;

		/* Determination de la periode d'asserv de la loco */
		dtLoco = rmp440_sec_period;

		/* Temps d'arret minimal (tps de decel) */
		dtArret = MAX (fabs(robot->v) / dynamics->linAccelMax,
			fabs(robot->w) / dynamics->angAccelMax);

		/* Temps de prise en compte (v = cst) */
		dtRetard =  1 * dtLoco;

		/* Angle final (avec periode de prise en compte) */
		theta = robot->theta + robot->w * (dtArret / 2 + dtRetard);

		/* Angle moyen */
		angle = (robot->theta + theta) / 2;

		/* Longueur d'arret */
		dsArret = robot->v * (dtArret / 2 + dtRetard);

		/* Prevision position du centre */
		ref->x = robot->xRob + dsArret * cos (angle);
		ref->y = robot->yRob + dsArret * sin (angle);

		/* Vitesses de consigne nulles */
		ref->v = 0. ;
		ref->w = 0. ;

		/* Indiquer l'arret du tracking */
	}
	*track_mode = or_genpos_no_tracking;
	*rs_mode = rmp440_mode_idle;
	return rmp440_ether;
}
