#include "StdAfx.h"
#include "RecordMobot.h"

CRecordMobot::CRecordMobot(void)
{
	_numMotions = 0;
	_motions = (struct motion_s*)malloc(sizeof(struct motion_s) * 100);
	_numMotionsAllocated = 100;
}

CRecordMobot::~CRecordMobot(void)
{
	free(_motions);
}

int CRecordMobot::record(void)
{
	/* Get the robots positions */

}
