#!/bin/sh
# Clones Ros-package dependencies to build directory.
# All files are source and build along with dotnect_stack.
sudo -v;
result=${PWD##*/}

# Run in original directory
if [ "$result" = "install" ]; then
  echo "Current directory ok"
else
  echo "Please run script inside $result folder"
exit
fi;

echo "\nThis script git clones required ROS-packages for dotnect_stack."
echo "List of folders (each yes/no): \nimage_common\nvision_opencv"

echo "\nDownload image_common? ...yes/no";
read -r inp1;
if [ "$inp1" = yes ] || [ "$inp1" = y ]; then
  cd ../..
  git clone https://github.com/ros-perception/image_common.git;
  chmod -R 777 image_common;
  cd ./dotnect_stack/install
  echo "successful."
else
  echo "skipped."
fi

echo "\nDownload vision_opencv? ...yes/no";
read -r inp1;
if [ "$inp1" = yes ] || [ "$inp1" = y ]; then
  cd ../..
  git clone https://github.com/ros-perception/vision_opencv.git;
  chmod -R 777 vision_opencv;
  cd ./dotnect_stack/install
  echo "successful."
else
  echo "skipped."
fi

echo "Done.\n"
# EOF
