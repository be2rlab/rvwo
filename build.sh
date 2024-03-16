echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2

# cd ../../Sophus

# echo "Configuring and building Thirdparty/Sophus ..."

# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j2

#cd ../../caffe-segnet
#
#echo "Configuring and building Thirdparty/caffe-segnet ..."
#
#mkdir build
#cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release \
#            -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3
#make -j2

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
