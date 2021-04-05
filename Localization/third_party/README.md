# Third Party

## Eigen3

### 版本

3.3.7-2

### 安装

```bash
sudo apt install libeigen3-dev
```

## Pangolin

MIT Liscence

版本 v0.5

```bash
sudo apt install libgl1-mesa-dev libglew-dev cmake
sudo apt install pkg-config libegl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols
sudo apt install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
sudo apt install libdc1394-22-dev libraw1394-dev
sudo apt install libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev

cd Pangolin
mkdir install
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make -j7
make install
cd ../..
```

## OpenCV

BSD Liscence

版本 3.4.14

```bash
sudo apt install build-essential
sudo apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install libpython2-dev libpython3-dev python-numpy python3-numpy
sudo apt install libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev

cd opencv
mkdir install
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ..
make -j7
make install
cd ../..
```

## DBoW2

```bash
echo "Configuring and building Thirdparty/DBoW2 ..."

cd DBoW2
mkdir build && cd build
export OpenCV_DIR=../../opencv/install/share/OpenCV
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j7
cd ../..
```



## g2o

```bash
echo "Configuring and building Thirdparty/g2o ..."

cd g2o
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j7
cd ../..
```



## Vocabulary

```bash
echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..
```



## ORBSLAM

```bash
echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
```

## JSON

nlohmann

3.9.1

```
cd nlohmann_json
mkdir install
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install ..
make -j7
make install
cd ../..
```

