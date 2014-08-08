#!/bin/bash
scp root@192.168.42.1:code/$1 .
export value=`wc -l $1 | awk '{print $1}'`
echo $value
cat XYquad.template | sed -e "s|-N-|$value|g" >  XYquad.C
root -b XYquad.C
display XYquad.png
