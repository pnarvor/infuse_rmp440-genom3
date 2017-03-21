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
#ifndef _CODELS_H
#define _CODELS_H

#define DEBUG 1

/*----------------------------------------------------------------------*/

/*
 * Prototypes
 */

/* joystick.c */
extern void getJoystickSpeeds(struct or_joystick_state *joy, 
    rmp440_feedback *data, double *v, double *w, double *avmax, double *awmax);

/* motion_helpers.c */
extern void rmp440DataUpdate(rmp440_feedback *data, FE_STR *fe,
    rmp440_status_str *status, rmp_status_str *statusgen);
extern void rmp440VelocityGet(const rmp440_feedback *data, 
    or_genpos_cart_state *robot);
extern void rmp440VelocitySet(const rmp440_io *rmp, const rmp440_feedback *data,
    double v, double w, double *vCommand, double *wCommand);

/* track.c */
extern genom_event track(const or_genpos_cart_speed *ref,
    or_genpos_track_mode track_mode,
    double *vRef, double *wRef, genom_context self);
extern void bound_accels(rmp440_max_accel *acc, double t,
    double *vel_reference, double *ang_reference);
extern void control_yaw(rmp440_gyro_asserv *gyro,
    double t, double vel_reference, double yawr_reference,
    double yawr_measure, double yaw_measure, double *yawr_command);
extern bool joystickQuit(struct or_joystick_state *joy);



#endif /* _CODELS_H */
