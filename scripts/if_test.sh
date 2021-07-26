#!/usr/bin/env bash

A="apple"
B="banana"

if [[ "$A" == "$B" ]]; then
  echo "Strings are equal"
else
  echo "Strings are not equal"
fi

if [[ "$A" == "$B" || "$B" == "$A" ]]; then
  echo "Strings are equal"
else
  echo "Strings are not equal"
fi