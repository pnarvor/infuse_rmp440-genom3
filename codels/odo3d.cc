#include <MTI-clients/MTI.h>

#include "acrmp440.h"
#include "rmp440_c_types.h"

bool readMTI(MTI_DATA** mtiHandle, rmp440_mti_inertial_data* data)
{
    if(!mtiHandle)
        return false;
    MTI* mtiHandleP = (MTI*)*mtiHandle;
    if(!mtiHandleP)
        return false;

    return mtiHandleP->read((INERTIAL_DATA*)data, false);
}



genom_event rmp440Odo3d(int *report, double period)
{
	GENPOS_CART_STATE *robot = &SDI_F->robot;
	RMP440_CART_3DSTATE *robot3d = &SDI_F->robot3d;
	RMP440_STATUS_STR *status = &SDI_F->status;
	POM_EULER *attitude = &SDI_F->attitude;
	int i;
	static double mpitch = 0.0, oldPitch = -1.0;
	static int pausePitch = 0;
	static double lastVel = 0.0;
	static double odoPitch = 0.0;
	static double lastLargeAccelTime = 0.0;
	static double history_pitch[2*RMP440_ZODOCOR_AVGPITCHSIZE] = {0.};
	static int history_pitch_i = 0;
	static double history_vel[RMP440_ZODOCOR_AVGVELSIZE] = {0.};
	static int history_vel_i = 0;
	double vel, accel;

	/* 3D odometry */
	if (SDI_F->odometryMode == RMP440_ODO_3D) {
		if (readAttitudeEstimation() == ERROR) {
			printf("switching back to odo2D\n");
			SDI_F->odometryMode = RMP440_ODO_2D;
		} else
		{
			/* pause pitch updates if acceleration is too large */
			history_pitch[history_pitch_i++] = attitude->pitch;
			if (history_pitch_i >= 2*RMP440_ZODOCOR_AVGPITCHSIZE) history_pitch_i = 0;
			history_vel[history_vel_i++] = robot->v;
			if (history_vel_i >= RMP440_ZODOCOR_AVGVELSIZE) history_vel_i = 0;

			vel = 0.;
			for(i = 0; i < RMP440_ZODOCOR_AVGVELSIZE; ++i)
				vel += history_vel[i];
			vel /= RMP440_ZODOCOR_AVGVELSIZE;
			
			accel = (vel - lastVel) / period;
			lastVel = vel;

			if (fabs(accel) > RMP440_ZODOCOR_ACCTHRES*RMP_DEFAULT_MAXIMUM_DECEL)
			{
				lastLargeAccelTime = 0.0;
				if (!pausePitch)
				{
					/* Compute pitch value to use next during the pitch update pause.
					 * We don't use the latest RMP440_ZODOCOR_AVGPITCHSIZE values because they are
					 * often already contaminated by the acceleration (and decreasing the
					 * acceleration threshold would be too sensitive to noise).
					 * We average the RMP440_ZODOCOR_AVGPITCHSIZE values before to filter the noise.
					 */
					odoPitch = 0.0;
					for(i = 0; i < RMP440_ZODOCOR_AVGPITCHSIZE; ++i)
						odoPitch += history_pitch[(history_pitch_i+i)%(2*RMP440_ZODOCOR_AVGPITCHSIZE)];
					odoPitch /= (2*RMP440_ZODOCOR_AVGPITCHSIZE);
					pausePitch = 1;
				}
			} else
			{
				if (pausePitch)
				{
					lastLargeAccelTime += period;
					if (lastLargeAccelTime > RMP440_ZODOCOR_LATENCY)
						pausePitch = 0;
				}
				if (!pausePitch)
					odoPitch = attitude->pitch;
			}
		}
	}
	if (SDI_F->odometryMode == RMP440_ODO_2D) {
		mpitch = 0;
		oldPitch = -1;
		memset(attitude, 0, sizeof(POM_EULER));
	}
	robot3d->pitch = attitude->pitch;
	robot3d->roll = attitude->roll;
	/* XXXX For now keep the position in the Z=0 frame */
	robot3d->theta  = robot->theta;
	robot3d->xRob = robot->xRob;
	robot3d->yRob = robot->yRob;
	robot3d->zRob += - (robot->v * period) * sin(odoPitch);
	robot3d->xRef = robot->xRef;
	robot3d->yRef = robot->yRef;
	robot3d->zRef = 0.0;
	robot3d->v = robot->v;
	robot3d->vt = 0.0;
	robot3d->w = robot->w;

	return OK;
}
