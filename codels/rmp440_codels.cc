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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"
#include "codels.h"

#include "rmp440_Log.h"

/* --- Activity Track --------------------------------------------------- */

/** Validation codel trackControl of activity Track.
 *
 * Returns genom_ok.
 * Throws rmp440_not_connected, rmp440_port_not_found, rmp440_bad_ref,
 * rmp440_cmd_stop_track, rmp440_motors_off, rmp440_emergency_stop,
 * rmp440_power_cord_connected.
 */
genom_event
trackControl(const rmp440_io *rmp, const rmp440_feedback *rs_data,
             const genom_context self)
{
	if (rmp == NULL)
		return rmp440_not_connected(self);

	return genom_ok;
}


/* --- Activity DirectTrack --------------------------------------------- */

/** Validation codel trackControl of activity DirectTrack.
 *
 * Returns genom_ok.
 * Throws rmp440_not_connected, rmp440_port_not_found, rmp440_bad_ref,
 * rmp440_cmd_stop_track, rmp440_motors_off, rmp440_emergency_stop,
 * rmp440_power_cord_connected, rmp440_invalid_parameter.
 */
/* already defined in service Track validation */



/* --- Function toggleInfuseTrackMode ----------------------------------- */

/** Codel toggleInfuseTrackMode of function toggleInfuseTrackMode.
 *
 * Returns genom_ok.
 */
genom_event
toggleInfuseTrackMode(rmp440_mode rs_mode, uint8_t *infuseTrackMode,
                      const genom_context self)
{
    if(*infuseTrackMode)
    {
        if(rs_mode == rmp440_mode_track)
        {
            printf("Error, rmp440 is in track activity. Deactivate track activity before toggling track mode.\nIgnoring command.");;
            return genom_ok;
        }
        *infuseTrackMode = 0;
        printf("Using idl track mode : cmd_vel\n");
    }
    else
    {
        *infuseTrackMode = 1;
        printf("Using infuse track mode : cmd_vel_Infuse\n");
    }
    return genom_ok;
}


/* --- Function log ----------------------------------------------------- */

/** Codel log_start of function log.
 *
 * Returns genom_ok.
 * Throws rmp440_sys_error.
 */
genom_event
log_start(const char path[64], rmp440_log_str **log,
          const genom_context self)
{
	FILE *f;

	log_stop(log, self);

	f = fopen(path, "w");
	if (f == NULL) 
		return rmp440_sys_error(self);
	fprintf(f, rmp440_feedback_header "\n");

	*log = (rmp440_log_str*)malloc(sizeof(**log));
	if (*log == NULL) {
		fclose(f);
		unlink(path);
		errno = ENOMEM;
		return rmp440_sys_error(self);
	}
	
	(*log)->out = f;
	return genom_ok;
}


/* --- Function log_stop ------------------------------------------------ */

/** Codel log_stop of function log_stop.
 *
 * Returns genom_ok.
 */
genom_event
log_stop(rmp440_log_str **log, const genom_context self)
{
	if (*log == NULL)
		return genom_ok;

	fclose((*log)->out);
	free(*log);
	*log = NULL;
	return genom_ok;
}
