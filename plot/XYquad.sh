#!/bin/bash
rm log.txt
scp root@192.168.42.1:code/log.txt .
export value=`wc -l log.txt | awk '{print $1}'`
echo $value
cat XYquad.template | sed -e "s|-N-|$value|g" >  XYquad.C
root -b XYquad.C
export file=`ls -1 -tr XYquad*.png | tail -n 1`
display $file
