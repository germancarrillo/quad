#!/bin/bash
cat ../test/log.txt | uniq > log.txt
export value=`wc -l log.txt | awk '{print $1}'`
export TMIN=`head -n 2 log.txt | tail -n 1 | awk '{print $9}'`
export TMAX=`tail -n 1 log.txt | awk '{print $9}'`
echo $value $TMIN  $TMAX
cat XYquad.template | sed -e "s|-N-|$value|g" -e "s|-TMIN-|$TMIN|g" -e "s|-TMAX-|$TMAX|g" >  XYquad.C
root -b XYquad.C
export file=`ls -1 -tr XYquad*.png | tail -n 1`
display $file

