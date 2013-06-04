#pragma once
#include "resourceppc.h"
int getDialogAngles(
					HWND hDlg,
					int dialogID1, 
					int dialogID2, 
					int dialogID3, 
					int dialogID4, 
					double angles[4],  /* OUT */
					unsigned char* motorMask /* OUT */);
int poseJoints(const double *angles, unsigned char motorMask);
int moveJoints(const double *angles, unsigned char motorMask);
int moveWait();
#if 0
#include <imobotcomms.h>
class CMobot : public BRComms
{
public:
	int poseJoints(double angles[4], unsigned char motorMask);
};
#endif