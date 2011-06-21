#include "gait.h"
#include <stdlib.h>

Motion::Motion(enum motion_type_e motion_type, int enc[4])
{
  for(int i = 0; i < 4; i++) {
    _enc[i] = enc[i];
  }
  _motion_type = motion_type;
}

Motion::Motion(enum motion_type_e motion_type, float angles[4])
{
  DEGREE2ENC(angles, _enc);
  _motion_type = motion_type;
}

Motion::Motion(enum motion_type_e motion_type, int enc[4], unsigned char motorMask)
{
  for(int i = 0; i < 4; i++) {
    _enc[i] = enc[i];
  }
  _motion_type = motion_type;
  _motor_mask = motorMask;
}

Motion::Motion(enum motion_type_e motion_type, float angles[4], unsigned char motorMask)
{
  DEGREE2ENC(angles, _enc);
  _motion_type = motion_type;
  _motor_mask = motorMask;
}

const int* Motion::getEncs() const
{
  return _enc;
}

enum motion_type_e Motion::getType() const
{
  return _motion_type;
}

Motion & Motion::operator= (const Motion &rhs)
{
  _motion_type = rhs._motion_type;
  for(int i = 0; i < 4; i++) {
    _enc[i] = rhs._enc[i];
  }
  _motor_mask = rhs._motor_mask;
  return *this;
}

#ifndef _WIN32
Gait::Gait(const char* name) 
#else
Gait::Gait(const WCHAR* name)
#endif
{
  _numMotions = 0;
  _motions = (Motion*)malloc(sizeof(Motion) * MOTION_ALLOC_SIZE);
  _numMotionsAllocated = MOTION_ALLOC_SIZE;
#ifndef _WIN32
  _name = strdup(name);
#else
  _name = (WCHAR*)malloc((wcslen(name)+1) * sizeof(WCHAR));
  wcscpy(_name, name);
#endif
}

int Gait::addMotion(Motion* motion)
{
  /* Allocated more space if needbe */
  if(_numMotions >= _numMotionsAllocated) {
    _numMotionsAllocated += MOTION_ALLOC_SIZE;
    _motions = (Motion*)realloc(_motions, _numMotionsAllocated);
  }

  _motions[_numMotions] = *motion;
  _numMotions++;

  return 0;
}

int Gait::getNumMotions()
{
  return _numMotions;
}

const Motion* Gait::getMotion(int index)
{
  if(index >= _numMotions || index < 0) {
    return NULL;
  } else {
    return &_motions[index];
  }
}
