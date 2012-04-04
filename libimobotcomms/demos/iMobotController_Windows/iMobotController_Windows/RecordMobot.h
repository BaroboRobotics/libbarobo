#pragma once
#include "c:\documents and settings\dko\projects\barobo\imobot_gumstix_api\libimobotcomms\mobot.h"

enum motionType_e
{
	MOTION_POS,
	MOTION_SLEEP,
};

struct motion_s
{
	enum motionType_e motionType;
	union data {
		double sleepDuration;
		double pos[4];
	};
};

class CRecordMobot :
	public CMobot
{
public:
	CRecordMobot(void);
	~CRecordMobot(void);
	int record();
	int playback();
	int stopPlayback();
private:
	int _numMotions;
	struct motion_s *_motions;
	int _numMotionsAllocated;
};
