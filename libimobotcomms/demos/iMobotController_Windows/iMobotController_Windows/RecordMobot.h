#pragma once
#include <mobot.h>

enum motionType_e
{
	MOTION_POS,
	MOTION_SLEEP,
};

struct motion_s
{
	enum motionType_e motionType;
	union data_u {
		double sleepDuration;
		double pos[4];
	} data;
};

class CRecordMobot :
	public CMobot
{
public:
	CRecordMobot(void);
	~CRecordMobot(void);
	int record();
	int play(int index);
private:
	int _numMotions;
	struct motion_s **_motions;
	int _numMotionsAllocated;
};
