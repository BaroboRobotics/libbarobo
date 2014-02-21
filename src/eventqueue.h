#ifndef EVENT_QUEUE_H__
#define EVENT_QUEUE_H__

#include <stdint.h>

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

    float joint_data[4];

    uint16_t accel_data[3];
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
