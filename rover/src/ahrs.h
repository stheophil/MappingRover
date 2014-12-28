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

extern int g_nRoll;
extern int g_nPitch;
extern int g_nYaw;

#endif /* defined(____ahrs__) */
