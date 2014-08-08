date
git clone https://github.com/camilocarrillo/quad
cd quad
git rev-parse HEAD >> hash.txt
cd ../
scp -r quad/* root@192.168.42.1:code/
ssh root@192.168.42.1 'cd code;make clean;make'
date
