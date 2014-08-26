killall -e code
echo  FORCE STOPPING
echo 0=750 > /dev/servoblaster
echo 1=750 > /dev/servoblaster
echo 2=750 > /dev/servoblaster
echo 3=750 > /dev/servoblaster
sleep 1s
echo 0=0 > /dev/servoblaster
echo 1=0 > /dev/servoblaster
echo 2=0 > /dev/servoblaster
echo 3=0 > /dev/servoblaster
