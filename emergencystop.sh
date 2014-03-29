echo  FORCE STOPPING
echo 0=1500 > /dev/servoblaster
echo 4=1500 > /dev/servoblaster
echo 5=1500 > /dev/servoblaster
echo 7=1500 > /dev/servoblaster
sleep 1s
echo 0=0 > /dev/servoblaster
echo 4=0 > /dev/servoblaster
echo 5=0 > /dev/servoblaster
echo 7=0 > /dev/servoblaster
