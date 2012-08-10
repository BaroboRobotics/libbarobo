#!/bin/bash

TESTS="standtest continuousmotiontest"

for t in $TESTS; do
  ./$t
  if [ $? -ne 0 ] ; then
    echo Test Failed
    break;
  fi
done
