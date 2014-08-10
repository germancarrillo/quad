date
echo "Your IP:"
ifconfig | grep -A 1 wlan |tail -n 1 | awk '{print $2}'
sleep 3
git clone https://github.com/camilocarrillo/quad
cd quad
git rev-parse HEAD >> hash.txt
cd ../
scp -r * root@192.168.42.1:code/
ssh root@192.168.42.1 'cd code;make clean;make'
date
