#S  0 on P1-7           GPIO-4
##     1 on P1-11          GPIO-17
#     2 on P1-12          GPIO-18
#     3 on P1-13          GPIO-27
#W  4 on P1-15          GPIO-22
#E  5 on P1-16          GPIO-23
#     6 on P1-18          GPIO-24
#N  7 on P1-22          GPIO-25
echo  START 
killall -e servod
killall -e servod_new
#/root/code/servoblaster/servod --cycle-time=5000us --step-size=2us --p1pins="7,15,16,22"
/root/code/servoblaster/servod --cycle-time=5000us --step-size=2us --p1pins="7,22"
echo 0=0 > /dev/servoblaster
echo 1=0 > /dev/servoblaster
echo 2=0 > /dev/servoblaster
echo 3=0 > /dev/servoblaster
sleep 1s
echo 0=750 > /dev/servoblaster
echo 1=750 > /dev/servoblaster
echo 2=750 > /dev/servoblaster
echo 3=750 > /dev/servoblaster
sleep 1s
echo 0=790 > /dev/servoblaster
echo 1=790 > /dev/servoblaster
echo 2=790 > /dev/servoblaster
echo 3=790 > /dev/servoblaster
sleep 3s
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
