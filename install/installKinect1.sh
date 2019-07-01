#!/bin/sh
# Script installs Kinectv1 driver and OpenCV
#
sudo -v
result=${PWD##*/}

# Run in original directory
if [ "$result" = "install" ] then
	echo "Current directory ok"
else
	echo "Please run script inside $result folder"
	exit
fi

echo "\nThis script git clones ros-packages / installs the driver for use of kinectv1 with dotnect_stack."
echo "List: \nfreenect_stack\nrgbd_launch\nlibfreenect"

echo "\nDownload freenect_stack (Ros-wrapper for kinect1)? ...yes/no"
read -r inp1
if [ "$inp1" = yes ] || [ "$inp1" = y ] then
	cd ../..
	git clone https://github.com/ros-drivers/freenect_stack.git
	chmod -R 777 freenect_stack
	cd ./dotnect_stack/install
	echo "successful."
else
	echo "skipped."
fi

echo "\nDownload rgbd_launch (required by freenect_stack)? ...yes/no"
read -r inp1
if [ "$inp1" = yes ] || [ "$inp1" = y ] then
	cd ../..
	git clone https://github.com/ros-drivers/rgbd_launch.git
	chmod -R 777 rgbd_launch
	cd ./dotnect_stack/install
	echo "successful."
else
	echo "skipped."
fi

echo "Install libfreenect? <yes/no>"
read -r inp1
if [ "$inp1" = yes ] || [ "$inp1" = y ] then
	git clone https://github.com/OpenKinect/libfreenect.git
	cd libfreenect || exit
	mkdir MYBUILD
	cd MYBUILD || exit
	cmake -DBUILD_EXAMPLES=OFF -L ..
	echo "Check if path is ok? ..."
	read -r inp1
	make
	echo "$ make install ."
	#sudo mkdir -p /usr/local/lib/libfreenect #let itself decide
	make install #DESTDIR=/usr/local/lib/libfreenect
	cd ../.. || exit
	echo "Remove temporary libfreenect folder? <yes/no>"
	read -r inp1
	if [ "$inp1" = yes ] || [ "$inp1" = y ] then
		rm -r libfreenect
		echo "folder libfreenect removed."
	fi
	echo "successful."
else
	echo "skipped."
fi

#expands escapes
printf "\nDone."
# EOF
