//
//  ahrs.h
//  
//
//  Created by Sebastian Theophil on 26.12.14.
//
//

#ifndef ____ahrs__
#define ____ahrs__

void setupAHRS();
bool updateAHRS();

// in 1/1000 radians:
extern int g_nRoll;
extern int g_nPitch;
extern int g_nYaw;

#include "L3G.h"
#include "LSM303.h"

extern L3G gyro;
extern LSM303 compass;

#endif /* defined(____ahrs__) */
