#ifndef DEF_ODO3d_H
#define DEF_ODO3d_H

extern bool readMTI(MTI_DATA** mtiHandle, rmp440_mti_inertial_data* data);
extern genom_event rmp440Odo3d(MTI_DATA** mtiHandle, rmp440_mti* mti,
    or_genpos_cart_state *robot);

#endif
