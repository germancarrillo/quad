#!/bin/bash

export value=`wc -l $1 | awk '{print $1}'`
echo $value
cat XYquad.template | sed -e "s|-N-|$value|g" >  XYquad.C
root -b XYquad.C
display XYquad.png
