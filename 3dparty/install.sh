unzip eigen-3.3.9.zip 
cd eigen-3.3.9
sudo rm -rf build
mkdir build && cd build
cmake ..
sudo make install


cd ../../
sudo apt-get install libgoogle-glog-dev libgflags-dev
sudo apt-get install libatlas-base-dev
sudo apt-get install libsuitesparse-dev
unzip ceres-solver-1.14.x.zip
cd ceres-solver-1.14.x
sudo rm -rf build
mkdir build && cd build
cmake ..
make -j4
make test
sudo make install

unzip opencv-3.4.15.zip
cd opencv-3.4.15
sudo rm -rf build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
sudo make -j4
sudo make install
sudo echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf
sudo ldconfig

cd ../../../../../
catkin_make -j4
