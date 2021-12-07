cd /home/ubuntu/init

git clone https://github.com/ethz-asl/libnabo.git

cd libnabo

mkdir build
cd build
cmake ..
make 
sudo make install