#include "StdAfx.h"
#include "RecordMobot.h"

CRecordMobot::CRecordMobot(void)
{
	_numMotions = 0;
	_motions = (struct motion_s**)malloc(sizeof(struct motion_s*) * 100);
	_numMotionsAllocated = 100;
}

CRecordMobot::~CRecordMobot(void)
{
	free(_motions);
}

int CRecordMobot::record(void)
{
	/* Get the robots positions */
	double angles[4];
	getJointAngles(angles[0], angles[1], angles[2], angles[3]);
	struct motion_s* motion;
	motion = (struct motion_s*)malloc(sizeof(struct motion_s));
	motion->motionType = MOTION_POS;
	motion->data.pos[0] = angles[0];
	motion->data.pos[1] = angles[1];
	motion->data.pos[2] = angles[2];
	motion->data.pos[3] = angles[3];
	_motions[_numMotions] = motion;
	_numMotions++;
	return 0;
}

int CRecordMobot::play(int index)
{
	if (index < 0 || index >= _numMotions) {
		return -1;
	}
	return moveToNB(
		_motions[index]->data.pos[0],
		_motions[index]->data.pos[1],
		_motions[index]->data.pos[2],
		_motions[index]->data.pos[3]
	);
}