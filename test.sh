echo 0=0 > /dev/servoblaster ; 
echo 4=0 > /dev/servoblaster ; 
echo 5=0 > /dev/servoblaster ; 
echo 7=0 > /dev/servoblaster ; 
sleep 2s; 

for i in `seq 1500 1 1610`; 
	do 
	echo 0=$i > /dev/servoblaster; 
	echo 4=$i > /dev/servoblaster; 
	echo 5=$i > /dev/servoblaster; 
#	echo 7=$i > /dev/servoblaster; 
	echo $i;  
	sleep 0.3s; 
done

