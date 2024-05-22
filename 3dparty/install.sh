unzip eigen-3.3.9.zip 
cd eigen-3.3.9
mkdir build && cd build
cmake ..
sudo make install


cd ../../
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev
unzip ceres-solver-1.14.x.zip
cd ceres-solver-1.14.x
mkdir build && cd build
cmake ..
make -j4
make test
sudo make install

cd ../../../
catkin_make -j4
