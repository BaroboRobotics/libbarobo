#include "eventqueue.h"
#include <stdio.h>
#include <stdlib.h>

template <class T>
RingBuf<T>::RingBuf(int maxElements)
{
  MUTEX_NEW(lock_);
  MUTEX_INIT(lock_);

  COND_NEW(cond_);
  COND_INIT(cond_);

  index_ = 0;
  num_ = 0;
  max_ = maxElements;
  elements_ = new T[maxElements];
}

template <class T>
RingBuf<T>::~RingBuf()
{
  free(lock_);
  free(cond_);
  delete[] elements_;
}

template <class T>
T RingBuf<T>::pop()
{
  T elem = elements_[ index_%max_ ];
  index_++;
  num_--;
  return elem;
}

template <class T>
void RingBuf<T>::push(T elem)
{
  elements_[ (index_ + num_) % max_ ] = elem;
  num_++;
}

template <class T>
void RingBuf<T>::lock()
{
  MUTEX_LOCK(lock_);
}

template <class T>
void RingBuf<T>::unlock()
{
  MUTEX_UNLOCK(lock_);
}

template <class T>
void RingBuf<T>::wait()
{
  COND_WAIT(cond_, lock_);
}

template <class T>
void RingBuf<T>::signal()
{
  COND_SIGNAL(cond_);
}

template class RingBuf<event_t*>;
