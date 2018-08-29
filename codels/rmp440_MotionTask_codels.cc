/*
 * Copyright (c) 2009-2017 CNRS/LAAS
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
 ** rmp440MotionTaskCodels.c
 **
 ** Codels called by execution task rmp440MotionTask
 **
 ** Author: Matthieu Herrb
 ** Date: April 2009 for rmp400, updated for rmp440, May 2013,
 **       updated for genom3, January 2017
 **
 **/

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern "C" {
#include <rmp440/rmp440.h>
#include <fe/ftdi-emergency.h>
#include <gyroLib/gyro.h>
}

#include <MTI-clients/MTI.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"

#include "orMathLib.h"
#include "odoProba.h"
#include "codels.h"
#include "rmp440_Log.h"
#include "odo3d.h"

////////////////////////////////////////////////////////////////////////////////
#include <sys/time.h>
#include <infuse_asn1_types/TransformWithCovariance.h>
////////////////////////////////////////////////////////////////////////////////

/* --- Task MotionTask -------------------------------------------------- */

static void
gyroUpdate(GYRO_DATA **gyroId, rmp440_gyro *gyro,
    rmp440_gyro_asserv *gyro_asserv, or_genpos_cart_state *robot)
{
	uint32_t date;
#if DEBUG>1
	static int count = 0;
#endif
	/* Gyro */
	if (gyroId != NULL /* && gyro->currentMode != RMP440_GYRO_OFF */) {
		if (gyroRead(*gyroId, &(gyro->gyroTheta),
			    &(gyro->gyroOmega), &date) != 0) {
			gyro->currentMode = rmp440_gyro_off;
			gyroEnd(*gyroId);
			*gyroId = NULL;
		}
		else {
#if DEBUG>1
	if (count==10) {
          printf("Read gyro, theta : %5.2f, omega : %5.2f\n", gyro->gyroTheta, gyro->gyroOmega);
    }
#endif
		  gyro->gyroTheta = - gyro->gyroTheta;
		  gyro->gyroOmega = - gyro->gyroOmega;
		}
	}
	if (gyro->currentMode == rmp440_gyro_off ||
	    (gyro->currentMode == rmp440_gyro_on_if_motion
		&& fabs(robot->w) < 0.017
		&& fabs(robot->v) < 0.01)) {
		/* -- odo is reference */

		/* reset gyro offset to match odo */
		gyro->gyroToRobotOffset = angleLimit(robot->theta
		    - gyro->gyroTheta);
		/* switch back to odo */
		if (gyro->gyroOn) {
#if DEBUG>=1
			printf ("-> odo (%5.2f d) v %5.2f w%5.3f\n",
			    RAD_TO_DEG(robot->theta), robot->v, robot->w);
#endif
			gyro->gyroOn = false;
		}
	} else {
		/* -- gyro is reference */
		robot->theta = angleLimit(gyro->gyroTheta +
		    gyro->gyroToRobotOffset);
		/* switch back to gyro */
		if (!gyro->gyroOn) {
#if DEBUG>=1
			printf ("-> gyro (%5.2f d) v %5.2f w%5.3f\n",
			    RAD_TO_DEG(robot->theta), robot->v, robot->w);
#endif
			gyro->gyroOn = true;
			gyro_asserv->first = 1;
		}
	}

	/*debug */
#if DEBUG>1
	if (count==10) {
		printf ("MODE %s. gyro %5.3f + offset %5.3f = %5.3f =? %5.3f\n",
		    gyro->gyroOn ? "GYRO" : "ODO",
		    RAD_TO_DEG(gyro->gyroTheta),
		    RAD_TO_DEG(gyro->gyroToRobotOffset),
		    RAD_TO_DEG(angleLimit(gyro->gyroTheta +
			    gyro->gyroToRobotOffset)),
		    RAD_TO_DEG(robot->theta));
		count=0;
	}
	count++;
#endif
}

static void
yawToQuaternion(double yaw, or_t3d_pos *pos)
{
	pos->qw = cos(yaw * 0.5);
	pos->qx = 0.0;
	pos->qy = 0.0;
	pos->qz = sin(yaw * 0.5);
}

static void
eulerToQuaternion(double roll, double pitch, double yaw, or_t3d_pos *pos)
{
    double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	pos->qw = cy * cr * cp + sy * sr * sp;
	pos->qx = cy * sr * cp - sy * cr * sp;
	pos->qy = cy * cr * sp + sy * sr * cp;
	pos->qz = sy * cr * cp - cy * sr * sp;
}

/*----------------------------------------------------------------------*/

/** Codel initOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether, rmp440_odo.
 * Throws rmp440_emergency_stop.
 */
genom_event
initOdoAndAsserv(rmp440_ids *ids, const rmp440_PoseInfuse *PoseInfuse,
                 uint8_t *infuseTrackMode,
                 const rmp440_StatusGeneric *StatusGeneric,
                 const genom_context self)
{
	rmp_status_str *statusgen = StatusGeneric->data(self);
	rmp440_kinematics_str *kinematics = &ids->kinematics;
	or_genpos_cart_state *robot = &ids->robot;
	or_genpos_cart_3dstate *robot3d = &ids->robot3d;
	or_genpos_cart_speed *ref = &ids->ref;
	rmp440_gyro *gyro = &ids->gyro;
	rmp440_gyro_asserv *gyro_asserv = &ids->gyro_asserv;
	rmp440_max_accel *max_accel = &ids->max_accel;

	memset(statusgen, 0, sizeof(rmp_status_str));
	statusgen->robot_model = rmp_model_440;

	ids->rs_mode = rmp440_mode_idle;

	/* Kinematics */
	kinematics->leftWheelRadius = RMP_X2_TIRE_DIAMETER/2.0;
	kinematics->rightWheelRadius = RMP_X2_TIRE_DIAMETER/2.0;
	kinematics->axisWidth = RMP_DEFAULT_WHEEL_TRACK_WIDTH;

	/* configuration */
	robot->xRob = 0.;
	robot->yRob = 0.;
	robot->theta = 0.;
	robot->xRef = 0;
	robot->yRef = 0;
	robot->v = 0;
	robot->w = 0;

	/* Ref Data */
	ref->v = 0.;
	ref->w = 0.;
	/* XXX These 4 values are not used. */
	ref->vmax = RMP_DEFAULT_MAXIMUM_VELOCITY;
	ref->wmax = RMP_DEFAULT_MAXIMUM_YAW_RATE;
	ref->linAccelMax = RMP_DEFAULT_MAXIMUM_ACCEL;
	ref->angAccelMax = RMP_DEFAULT_MAXIMUM_YAW_ACCEL;

	/* gyro */
	gyro->currentMode = rmp440_gyro_off;
	gyro->gyroOn = false;
	gyro->gyroToRobotOffset = 0.0;
	gyro->gyroTheta = 0.0;

	/* gyro asserv */
	gyro_asserv->enabled = 0;
	gyro_asserv->first = 1;
	gyro_asserv->straight = 0;

	/* max accel */
	max_accel->prev_vel_command = 0.;
	max_accel->prev_vel_command_t = -1.;

    ////////////////////////////////////////////////////////////////////////////////////
    //preparing bitstream output
    asn1_bitstream* gbstream = PoseInfuse->data(self);
    if(!gbstream)
    {
        printf("Error getting port bstream at initialization !\n");
		return rmp440_pause_init_main;
    }
    
    // Init bitstream header
    struct timeval tv;
    gettimeofday(&tv,NULL);
    long long timeNow = tv.tv_sec*1000000 + tv.tv_usec;
    gbstream->header.seq = 0;
    gbstream->header.stamp.sec = timeNow / 1000000;
    gbstream->header.stamp.nsec = (timeNow % 1000000) * 1000;
    gbstream->header.frame_id = (char*)malloc(sizeof(char)*(1 + strlen("RoverBodyFrame")));
    sprintf(gbstream->header.frame_id, "RoverBodyFrame");
   
    //// Init bistream type
    gbstream->type = (char*)malloc(sizeof(char)*(1 + strlen("TransformWithCovariance")));
    sprintf(gbstream->type, "TransformWithCovariance");
    gbstream->serialization_method = 0; //uPER
    //reserve memory for serialized data 
    genom_sequence_reserve(&(gbstream->data), asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    gbstream->data._length = 0;

    *infuseTrackMode = 0;
    ////////////////////////////////////////////////////////////////////////////////////

    ids->mtiHandle = NULL;

    ids->mti.mtiOn = false; 
    ids->mti.currentMode = rmp440_mti_off;
    ids->mti.data.acc[0] = 0.0;
    ids->mti.data.acc[1] = 0.0;
    ids->mti.data.acc[2] = 0.0;

    ids->mti.data.gyr[0] = 0.0;
    ids->mti.data.gyr[1] = 0.0;
    ids->mti.data.gyr[2] = 0.0;

    ids->mti.data.mag[0] = 0.0;
    ids->mti.data.mag[1] = 0.0;
    ids->mti.data.mag[2] = 0.0;

    ids->mti.data.euler[0] = 0.0;
    ids->mti.data.euler[1] = 0.0;
    ids->mti.data.euler[2] = 0.0;

    ids->mti.data.count = 0;
    
    ids->mti.data.timeStampRaw       = 0.0;
    ids->mti.data.timeStampUndelayed = 0.0;
    ids->mti.data.timeStampFiltered  = 0.0;

    ids->odoMode = rmp440_odometry_2d;

	robot3d->xRef  = 0.;
	robot3d->yRef  = 0.;
	robot3d->zRef  = 0.;
	robot3d->xRob  = 0.;
	robot3d->yRob  = 0.;
	robot3d->zRob  = 0.;
	robot3d->roll  = 0.;
	robot3d->pitch = 0.;
	robot3d->theta = 0.;
	robot3d->v     = 0.;
	robot3d->vt    = 0.;
	robot3d->w     = 0.;

	return rmp440_odo;
}

/*----------------------------------------------------------------------*/

/** Codel odoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_odo.
 * Yields to rmp440_ether, rmp440_pause_odo, rmp440_end.
 * Throws rmp440_emergency_stop.
 */
genom_event
odoAndAsserv(const rmp440_io *rmp,
             const rmp440_kinematics_str *kinematics,
             const rmp440_var_params *var_params,
             const rmp440_log_str *log,
             const rmp440_Joystick *Joystick, GYRO_DATA **gyroId,
             FE_STR **fe, or_genpos_cart_state *robot,
             or_genpos_cart_config_var *var, or_genpos_cart_speed *ref,
             rmp440_max_accel *max_accel, rmp440_feedback **rs_data,
             rmp440_mode *rs_mode, rmp440_gyro *gyro,
             rmp440_gyro_asserv *gyro_asserv, MTI_DATA **mtiHandle,
             rmp440_mti *mti, rmp440_odometry_mode *odoMode,
             or_genpos_cart_3dstate *robot3d, const rmp440_Pose *Pose,
             const rmp440_PoseInfuse *PoseInfuse,
             const rmp440_Status *Status,
             const rmp440_StatusGeneric *StatusGeneric,
             const genom_context self)
{
	rmp440_feedback *data = *rs_data;
	double direction;
	rmp440_status_str *status = Status->data(self);
	rmp_status_str *statusgen = StatusGeneric->data(self);
	or_pose_estimator_state *pose = Pose->data(self);
	genom_event report = genom_ok;
	struct cmd_str cmd;

	if (rmp == NULL) {
		StatusGeneric->write(self);
		return rmp440_pause_odo; /* not initialized yet */
	}

	if (rmp440ReceiveAndDecode(rmp, data) < 0) {
		/* Probably motors OFF - no communication on the RMP440 */
		status->rs_mode = statusgen->rs_mode = rmp440_mode_motors_off;
		rmp440CmdNone(rmp);
		/* Publish Pose and Status Generic */
		Pose->write(self);
		StatusGeneric->write(self);
		return rmp440_pause_odo;
	}

	rmp440DataUpdate(data, *fe, status, statusgen);

	/* Read config */
	rmp440VelocityGet(data, robot);
	robot->xRef = robot->xRob;
	robot->yRef = robot->yRob;

	odoProba(robot, var,
	    kinematics->axisWidth, var_params->coeffLinAng,
	    rmp440_sec_period);	/* XXX could use the actual measured period */
	gyroUpdate(gyroId, gyro, gyro_asserv, robot);

    ////////////////////////////////////////////////////////////////////////
 
    if(*odoMode == rmp440_odometry_3d)
    {
        if(rmp440odo3d(mtiHandle, mti, robot, robot3d, odoMode, rmp440_sec_period))
        {
            //printf("acc  : %2.2f %2.2f %2.2f\ngyr  : %2.2f %2.2f %2.2f\nmag  : %2.2f %2.2f %2.2f\neuler: %2.2f %2.2f %2.2f\nperiod: %2.2lf\n\n",
            //    mti->data.acc[0], 
            //    mti->data.acc[1], 
            //    mti->data.acc[2], 
            //    mti->data.gyr[0], 
            //    mti->data.gyr[1], 
            //    mti->data.gyr[2], 
            //    mti->data.mag[0], 
            //    mti->data.mag[1], 
            //    mti->data.mag[2],
            //    mti->data.euler[0], 
            //    mti->data.euler[1], 
            //    mti->data.euler[2],
            //    rmp440_sec_period);
            ////fflush(stdout);
            //printf("euler rpy: %2.2f %2.2f %2.2f\nperiod : %2.2lf\n\n",
            //    robot3d->roll, 
            //    robot3d->pitch, 
            //    robot3d->theta,
            //    rmp440_sec_period);
            //fflush(stdout);
        }
    }

    ////////////////////////////////////////////////////////////////////////


    /* fill pose */
	pose->ts.sec = data->timestamp.tv_sec;
	pose->ts.nsec = data->timestamp.tv_nsec;
	pose->intrinsic = true;
	pose->pos._present = true;

    if(*odoMode != rmp440_odometry_3d)
    {
	    pose->pos._value.x = robot->xRob;
	    pose->pos._value.y = robot->yRob;
	    pose->pos._value.z = 0.0; 	/* XXX */
	    yawToQuaternion(robot->theta, &pose->pos._value); /* XXX */
    }
    else
    {
	    pose->pos._value.x = robot3d->xRob;
	    pose->pos._value.y = robot3d->yRob;
	    pose->pos._value.z = robot3d->zRob;
	    eulerToQuaternion(robot3d->roll,
                          robot3d->pitch,
                          robot3d->theta,
                          &pose->pos._value);
    }
	
    pose->vel._present = true;
	pose->vel._value.vx = robot->v;
	pose->vel._value.vy = 0;
	pose->vel._value.vz = 0;
	pose->vel._value.wx = 0;	/* XXX */
	pose->vel._value.wy = 0;	/* XXX */
	pose->vel._value.wz = robot->w;

	/*
	 * Asserv
	 */
	switch (*rs_mode) {
	case rmp440_mode_motors_off:
		/* No motion possible */
		return rmp440_pause_odo;

	case rmp440_mode_emergency:
	case rmp440_mode_idle:
		ref->v = 0.0;
		ref->w = 0.0;
		break;

	case rmp440_mode_manual:
		Joystick->read(self);
		getJoystickSpeeds(Joystick->data(self), data,
		    &ref->v, &ref->w, &ref->linAccelMax, &ref->angAccelMax);
		break;

	case rmp440_mode_track:
		/* ref has beed updated by the trackTask */
		break;

	default:
		printf("-- invalid rs_mode %d\n", *rs_mode);
		return rmp440_end;
	}
	if (report != genom_ok) {
		/* In case an error occured,
		   stop the robot and the tracking */
		*rs_mode = statusgen->rs_mode = rmp440_mode_idle;

		ref->v = 0;
		ref->w = 0;
		return report;
	}

	/* keep theoretical values */
	cmd.vReference = ref->v;
	cmd.wReference = ref->w;

	/* Adjustements depending on the robot */
	double t = data->timestamp.tv_sec + data->timestamp.tv_nsec*1e-9;
	if (*rs_mode == rmp440_mode_track)
		bound_accels(data, max_accel, t, &ref->v, &ref->w);

	if (gyro->gyroOn)
		control_yaw(gyro_asserv, t, ref->v, ref->w,
		    gyro->gyroOmega, gyro->gyroTheta, &ref->w);

	/* Send  commands to the wheels */
	rmp440VelocitySet(rmp, data, ref->v, ref->w,
	    &cmd.vCommand, &cmd.wCommand);

	/* log */
	if (log != NULL)
		rmp440LogFeedback(log, gyro, gyro_asserv, &cmd, data);

	/* publish */
	Pose->write(self);
	Status->write(self);
	StatusGeneric->write(self);
    
    
    
    
    //////////////////////////////////////////////////////////////////////////////////////
    ///////////////////// Infuse : Publish pose as asn1::bitstream
    if(!pose)
        return rmp440_pause_odo;

    struct timeval tv;
    gettimeofday(&tv,NULL);
    long long timeNow = tv.tv_sec*1000000 + tv.tv_usec;
    
    asn1SccTransformWithCovariance asnPose;
    asnPose.metadata.msgVersion = transformWithCovariance_version;

    //strcpy(asnPose.metadata.parentFrameId.arr, "LocalTerrainFrame");
    sprintf((char*)asnPose.metadata.parentFrameId.arr, "LocalTerrainFrame");
    asnPose.metadata.parentFrameId.nCount = strlen((char*)asnPose.metadata.parentFrameId.arr) + 1;
    asnPose.metadata.parentTime.microseconds  = timeNow;
    asnPose.metadata.parentTime.usecPerSec = 1000000;

    //strcpy(asnPose.metadata.childFrameId.arr, "RoverBodyFrame");
    sprintf((char*)asnPose.metadata.childFrameId.arr, "RoverBodyFrame");
    asnPose.metadata.childFrameId.nCount = strlen((char*)asnPose.metadata.childFrameId.arr) + 1;
    asnPose.metadata.childTime.microseconds  = timeNow;
    asnPose.metadata.childTime.usecPerSec = 1000000;
    
    asnPose.data.translation.arr[0] = pose->pos._value.x;
    asnPose.data.translation.arr[1] = pose->pos._value.y;
    asnPose.data.translation.arr[2] = pose->pos._value.z;

    asnPose.data.orientation.arr[0] = pose->pos._value.qx;
    asnPose.data.orientation.arr[1] = pose->pos._value.qy;
    asnPose.data.orientation.arr[2] = pose->pos._value.qz;
    asnPose.data.orientation.arr[3] = pose->pos._value.qw;

    // TODO translate or_pose_estimator covariance to envire covariance
    for(int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            asnPose.data.cov.arr[i].arr[j] = 0;
        }
    }
    // to have a well defined cov matrix :
    for(int i = 0; i < 6; i++)
        asnPose.data.cov.arr[i].arr[i] = 1e-6;
    
    asn1_bitstream* gbstream = PoseInfuse->data(self);
    if(!gbstream || !pose)
        return rmp440_pause_odo;
    if(!gbstream->data._buffer)
        return rmp440_pause_odo;
    
    gbstream->header.seq = gbstream->header.seq + 1;
    gbstream->header.stamp.sec = timeNow / 1000000;
    gbstream->header.stamp.nsec = (timeNow % 1000000) * 1000;
    

    flag res;
    int errorCode;
    BitStream bstream;
    BitStream_Init(&bstream, gbstream->data._buffer, asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    res = asn1SccTransformWithCovariance_Encode(&asnPose, &bstream, &errorCode, TRUE);
    if(!res)
    {
        printf("error, Pose_Infuse encoding error : %d\n", errorCode);
	    return rmp440_pause_odo;
    }
    //Set encoded size in bistream
    gbstream->data._length = asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING;
    gbstream->data._length = bstream.count;

    PoseInfuse->write(self);
    ///////////////////// Infuse : end
    //////////////////////////////////////////////////////////////////////////////////////

	return rmp440_pause_odo;
}


/** Codel endOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_end.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop.
 */
genom_event
endOdoAndAsserv(rmp440_io **rmp, rmp440_feedback **rs_data,
                const genom_context self)
{
	rmp440_feedback *data = *rs_data;

	printf("-- %s\n", __func__);
	if (*rmp == NULL)
		return rmp440_ether;
	if (data->operational_state == 4)
		rmp440SetOperationalMode(*rmp, RMP_STANDBY_REQUEST);
	free(data);
	rmp440End(*rmp);
	*rmp = NULL;
	*rs_data = NULL;
	return rmp440_ether;
}


/* --- Activity Init ---------------------------------------------------- */

/** Codel rmp440InitStart of activity Init.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_init_main.
 * Throws rmp440_emergency_stop, rmp440_already_initialized,
 *        rmp440_malloc_error, rmp440_rmplib_error.
 */
genom_event
rmp440InitStart(const char device[32], rmp440_io **rmp, FE_STR **fe,
                rmp440_feedback **rs_data, const genom_context self)
{
	/* error if already connected */
	if (*rmp != NULL || *rs_data != NULL)
		return rmp440_already_initialized(self);

	/* connect emergency stop */
	*fe = fe_init(NULL);
	if (fe == NULL)
		return rmp440_malloc_error(self);
	*rs_data = rmp440FeedbackInit(NULL);
	if (*rs_data == NULL) {
		fe_end(*fe);
		*fe = NULL;
		return rmp440_malloc_error(self);
	}

	if (device[0] == '/') /* /dev/ttyACM0 */
		*rmp = rmp440Init(RMP440_INTERFACE_USB, device);
	else if (device[0] >= '0' && device[0] <= '9')
		/* ip address:port */
		*rmp = rmp440Init(RMP440_INTERFACE_ETHERNET, device);
	else if (strncmp(device, "fake:", 5) == 0)
		/* simulation with config file name */
		*rmp = rmp440Init(RMP440_INTERFACE_FAKE, device+5);

	if (*rmp == NULL) {
		free(*rs_data);
		*rs_data = NULL;
		fe_end(*fe);
		*fe = NULL;
		return rmp440_rmplib_error(self);
	}
	rmp440SetOperationalMode(*rmp, RMP_TRACTOR_REQUEST);
	printf("-- sent tractor request\n");
	return rmp440_init_main;
}

/** Codel rmp440InitMain of activity Init.
 *

 * Triggered by rmp440_init_main.
 * Yields to rmp440_pause_init_main, rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_already_initialized,
 *        rmp440_malloc_error, rmp440_rmplib_error.
 */
genom_event
rmp440InitMain(rmp440_io **rmp, FE_STR **fe, rmp440_feedback **rs_data,
               rmp440_mode *rs_mode, rmp440_dynamic_str *dynamics,
               rmp440_kinematics_str *kinematics,
               const or_genpos_cart_state *robot,
               const genom_context self)
{
	rmp440_feedback *data = *rs_data;
	static int count = 0;

	printf("-- init_main: receiveAndDecode\n");
	if (rmp440ReceiveAndDecode(*rmp, data) < 0) {
		printf("-- receiveAndDecode return -1 errno: %d %d\n",
		    errno, count);
		if (count++ == 10) {
			rmp440End(*rmp);
			*rmp = NULL;
			free(*rs_data);
			*rs_data = NULL;
			fe_end(*fe);
			*fe = NULL;
			count = 0;
			return rmp440_rmplib_error(self);
		}
	}

	/* Check motors status */
	if (data->operational_state != 4) {
		printf("-- init_main: operational_state != 4 %d\n",
		    data->operational_state);
		rmp440SetOperationalMode(*rmp, RMP_TRACTOR_REQUEST);
		return rmp440_pause_init_main;
	}
	printf("Motors ON\n");
	*rs_mode = rmp440_mode_idle;
	/* init kinematics from robot NVRAM */
	kinematics->leftWheelRadius = data->fram_tire_diameter/2.0;
	kinematics->rightWheelRadius = data->fram_tire_diameter/2.0;
	kinematics->axisWidth = data->fram_wheel_track_width;
	/* dynamics - XXX unused */
	dynamics->vMax = data->vel_limit;
	dynamics->wMax = data->yaw_rate_limit;
	dynamics->linAccelMax = data->fram_accel_limit;
	dynamics->angAccelMax = data->fram_yaw_accel_limit;
	/* check and warn if some values are not coherent */
	if (data->fram_vel_limit != data->vel_limit)
		printf("WARNING fram_vel_limit != vel_limit %f %f\n",
		    data->fram_vel_limit, data->vel_limit);
	if (data->fram_yaw_rate_limit != data->yaw_rate_limit)
		printf("WARNING fram_yaw_rate_limit != yaw_rate_limit %f %f\n",
		    data->fram_yaw_rate_limit, data->yaw_rate_limit);
	if (data->fram_accel_limit != data->fram_decel_limit)
		printf("WARNING fram_accel_limit != fram_decel_limit %f %f\n",
		    data->fram_accel_limit, data->fram_decel_limit);
	rmp440CmdNone(*rmp);
    
    //////////////////////////////////////////////////////////////////////////////////////
    ////preparing bitstream output
    //asn1_bitstream* gbstream = PoseInfuse->data(self);
    //if(!gbstream)
    //{
    //    printf("Error getting port bstream at initialization !\n");
	//	return rmp440_pause_init_main;
    //}
    //
    //// Init bitstream header
    //struct timeval tv;
    //gettimeofday(&tv,NULL);
    //long long timeNow = tv.tv_sec*1000000 + tv.tv_usec;
    //gbstream->header.seq = 0;
    //gbstream->header.stamp.sec = timeNow / 1000000;
    //gbstream->header.stamp.nsec = (timeNow % 1000000) * 1000;
    //gbstream->header.frame_id = (char*)malloc(sizeof(char)*(1 + strlen("RoverBodyFrame")));
    //sprintf(gbstream->header.frame_id, "RoverBodyFrame");
   
    ////// Init bistream type
    //gbstream->type = (char*)malloc(sizeof(char)*(1 + strlen("TransformWithCovariance")));
    //sprintf(gbstream->type, "TransformWithCovariance");
    //gbstream->serialization_method = 0; //uPER
    ////reserve memory for serialized data 
    //genom_sequence_reserve(&(gbstream->data), asn1SccTransformWithCovariance_REQUIRED_BYTES_FOR_ENCODING);
    //gbstream->data._length = 0;
    //////////////////////////////////////////////////////////////////////////////////////

	return rmp440_ether;
}


/* --- Activity JoystickOn ---------------------------------------------- */

/** Codel rmp440JoystickOnStart of activity JoystickOn.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_js_main.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnStart(const rmp440_Joystick *Joystick,
                      rmp440_mode *rs_mode, const genom_context self)
{
	struct or_joystick_state *joy;

	if (Joystick->read(self)) {
		printf("%s: read joystick failed\n", __func__);
		return rmp440_joystick_error(self);
	}
	joy = Joystick->data(self);

	if (joy == NULL) {
		printf("%s: joystick data failed\n", __func__);
		return rmp440_joystick_error(self);
	}
	*rs_mode = rmp440_mode_manual;

	return rmp440_js_main;
}

/** Codel rmp440JoystickOnMain of activity JoystickOn.
 *
 * Triggered by rmp440_js_main.
 * Yields to rmp440_pause_js_main, rmp440_inter.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnMain(const rmp440_Joystick *Joystick,
                     rmp440_mode rs_mode, const genom_context self)
{
	struct or_joystick_state *joy;

	if (Joystick->read(self)) {
		printf("%s: read joystick failed\n", __func__);
		return rmp440_joystick_error(self);
	}
	joy = Joystick->data(self);

	if (joy == NULL) {
		printf("%s: joystick data failed\n", __func__);
		return rmp440_joystick_error(self);
	}

	/* Check if mode changed */
	switch (rs_mode) {
	case rmp440_mode_power_coord:
		return rmp440_power_cord_connected(self);
	case rmp440_mode_emergency:
		return  rmp440_emergency_stop(self);
	case rmp440_mode_motors_off:
		return rmp440_motors_off(self);
	default:
		break;
	}
	/* Check for user abort */
	if (joystickQuit(joy)) {
		printf("Stop joystick\n");
		return rmp440_inter;
	}
	return rmp440_pause_js_main;
}

/** Codel rmp440JoystickOnInter of activity JoystickOn.
 *
 * Triggered by rmp440_inter.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnInter(rmp440_mode *rs_mode, or_genpos_cart_speed *ref,
                      const genom_context self)
{

	printf("%s\n", __func__);
	*rs_mode = rmp440_mode_idle; /* XXXXXX */
	return rmp440_ether;
}


/* --- Activity Gyro ---------------------------------------------------- */

/** Codel rmp440GyroExec of activity Gyro.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_gyro_error.
 */
genom_event
rmp440GyroExec(const rmp440_gyro_params *params,
               const or_genpos_cart_state *robot, rmp440_gyro *gyro,
               GYRO_DATA **gyroId, const genom_context self)
{

	if (*gyroId == NULL) {
		*gyroId = gyroInit(params->type, params->port,
			    params->latitude, params->woffset);
		if (*gyroId == NULL) {
			gyro->currentMode = rmp440_gyro_off;
			return rmp440_gyro_error(self);
		}
	}

	/* read gyro once */
	if (gyroReadAngle(*gyroId, &gyro->gyroTheta) != 0) {
		gyro->currentMode = rmp440_gyro_off;
		gyroEnd(*gyroId);
		*gyroId = NULL;
		return rmp440_gyro_error(self);
	} else
		gyro->gyroTheta = - gyro->gyroTheta;
	/* reset gyro offset to match odo */
    /////////////////////////////////////////////////////////////////
    printf("Robot theta : %f\n", robot->theta);
    //////////////////////////////////////////////////////////////
	gyro->gyroToRobotOffset = robot->theta - gyro->gyroTheta;

	/* Finally set gyro mode */
	gyro->currentMode = params->mode;
	return rmp440_ether;
}


/* --- Activity GyroBiasUpdate ------------------------------------------ */

/** Codel rmp440GyroBiasUpdate of activity GyroBiasUpdate.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_gyro_error.
 */
genom_event
rmp440GyroBiasUpdate(int32_t nbMeasures,
                     const or_genpos_cart_state *robot,
                     rmp440_gyro *gyro, GYRO_DATA **gyroId,
                     const genom_context self)
{
    if(gyro->currentMode == rmp440_gyro_off || *gyroId == NULL)
    {
        printf("Error gyroBiasEstimate : gyro must be initialized\n");
        return rmp440_gyro_error(self);
    }

	if (gyroUpdateWOffset(*gyroId, nbMeasures) != 0)
        return rmp440_gyro_error(self);

    return rmp440_ether;
}


/* --- Activity InitMTI ------------------------------------------------- */

/** Codel rmp440MTIopen of activity InitMTI.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_mti_error.
 */
genom_event
rmp440MTIopen(const rmp440_mti_params *params, MTI_DATA **mtiHandle,
              rmp440_mti *mti, rmp440_mti_config *mtiConfig,
              const genom_context self)
{
    MTI* mtiHandleP = (MTI*)*mtiHandle;
    if(mtiHandle == NULL)
        return rmp440_mti_error(self);

    // Checking input parameters
    switch(params->outputMode)
    {
        default:
            printf("Error MTI open : invalid outputMode parameter. Candidates are:\n\
                    MTI_OPMODE_CALIBRATED  = 2\n\
                    MTI_OPMODE_ORIENTATION = 4\n\
                    MTI_OPMODE_BOTH        = 6\n");
            return rmp440_mti_error(self);
            break;
        case (int)MTI_OPMODE_CALIBRATED:
            break;
        case (int)MTI_OPMODE_ORIENTATION:
            break;
        case (int)MTI_OPMODE_BOTH:
            break;
    }
    switch(params->outputFormat)
    {
        default:
            printf("Error MTI open : invalid outputFormat parameter. Candidates are:\n\
                    MTI_OPFORMAT_QUAT  = 0\n\
                    MTI_OPFORMAT_EULER = 4\n\
                    MTI_OPFORMAT_MAT   = 8\n");
            return rmp440_mti_error(self);
            break;
        case (int)MTI_OPFORMAT_QUAT:
            break;
        case (int)MTI_OPFORMAT_EULER:
            break;
        case (int)MTI_OPFORMAT_MAT:
            break;
    }

    // Config copied from MTIinitExec codel in robotpkg/localization/MTI
    mtiConfig->outputMode           = (OutputMode)params->outputMode;
    mtiConfig->outputFormat         = (OutputFormat)params->outputFormat;
    mtiConfig->syncOutMode          = MTI_SYNCOUTMODE_DISABLED;
    mtiConfig->syncOutPulsePolarity = MTI_SYNCOUTPULSE_POS;
    mtiConfig->syncOutSkipFactor    = 0;
    mtiConfig->syncOutOffset        = 0;
    mtiConfig->syncOutPulseWidth    = 29498; //1ms pulse

    //// Same config as mtiTest
    //mtiHandleP = new MTI(params->port,
    //    (OutputMode)mtiConfig->outputMode,
    //    (OutputFormat)mtiConfig->outputFormat,
    //    MTI_SYNCOUTMODE_DISABLED);
        
    
    mtiHandleP = new MTI(params->port,
        (OutputMode)mtiConfig->outputMode,
        (OutputFormat)mtiConfig->outputFormat);
    if(!mtiHandleP)
        return rmp440_mti_error(self);

    //if(!mtiHandleP->set_syncOut((SyncOutMode)mtiConfig->syncOutMode,
    //    (SyncOutPulsePolarity)mtiConfig->syncOutPulsePolarity,
    //    mtiConfig->syncOutSkipFactor,
    //    mtiConfig->syncOutOffset,
    //    mtiConfig->syncOutPulseWidth))
    //{
    //    printf("Error MTI open : set_SyncOut failed\n");
    //    delete mtiHandleP;
    //    return rmp440_mti_error(self);
    //}
    
    *mtiHandle = (MTI_DATA*)mtiHandleP;
    mti->currentMode = params->mode;

    return rmp440_ether;
}


/* --- Activity ToggleOdometryMode -------------------------------------- */

/** Codel rmp440ToggleOdoMode of activity ToggleOdometryMode.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_odo3d_error.
 */
genom_event
rmp440ToggleOdoMode(MTI_DATA **mtiHandle, rmp440_mti *mti,
                    rmp440_odometry_mode *odoMode,
                    const genom_context self)
{
    MTI* mtiHandleP;
    if(*odoMode == rmp440_odometry_2d)
    {
        if(!mtiHandle)
            return rmp440_odo3d_error(self);
        mtiHandleP = (MTI*)*mtiHandle;
        if(!mtiHandleP)
        {
            printf("Error toggleOdoMode, did you initialize the mti ?\n");
            return rmp440_odo3d_error(self);
        }
        // read once to check if ok
        if(!mtiHandleP->read((INERTIAL_DATA*)(&mti->data),false))
            return rmp440_odo3d_error(self);

        *odoMode = rmp440_odometry_3d;
    }
    else
    {
        //carefull with reinit...
        *odoMode = rmp440_odometry_2d;
    }
    return rmp440_ether;
}





