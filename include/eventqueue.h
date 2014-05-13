#ifndef EVENT_QUEUE_H__
#define EVENT_QUEUE_H__
#ifdef _WIN32
#include <windows.h>
#endif

#ifdef _MSC_VER
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef __int16 int16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#else
#include <stdint.h>
#endif

#include "thread_macros.h"

typedef struct event_s
{
  int event;
  uint32_t millis;
  uint16_t address;
  union {
    struct {
      uint8_t event_mask;
      uint8_t down_mask;
      uint8_t up_mask;
    } button_data;

    struct {
      uint16_t address;
      char serialID[5];
    } reportAddress;
  
    char * debug_data;
    struct {
      float angles[4];
      uint8_t mask;
    }joint_data;

    int16_t accel_data[3];
  }data;
} event_t;

template <class T>
class RingBuf 
{
  public:
    RingBuf(int maxElements = 64);
    ~RingBuf();
    T pop();
    void lock();
    int num() {return num_;};
    void push(T elem);
    void signal();
    void unlock();
    void wait();
    
  private:
    MUTEX_T* lock_;
    COND_T* cond_;
    int index_;
    int num_;
    int max_;
    T *elements_;
};

#endif
