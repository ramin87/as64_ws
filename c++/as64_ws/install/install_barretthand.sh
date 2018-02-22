#!/usr/bin/env bash

MESSAGE="Installing BarrettHand Dependencies: codeblocks build-essential wxgtk2.8-dev freeglut3-dev libpopt-dev libpoco-dev wget..."; blue_echo && \
sudo apt-get update > /dev/null
sudo apt-get install -y codeblocks build-essential wxgtk2.8-dev freeglut3-dev libpopt-dev libpoco-dev wget > /dev/null

# 
MESSAGE="Download and install the PCAN Driver for BarrettHand..."; blue_echo && \
wget http://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-7.15.2.tar.gz
tar xvf peak-linux-driver-7.15.2.tar.gz > /dev/null
cd peak-linux-driver-7.15.2
make NET=NO_NETDEV_SUPPORT
sudo make install
sudo echo <<EOF KERNEL=="pcanusb*", NAME="pcanusb/%n", SYMLINK+="%k", GROUP="plugdev" | sudo tee -a /etc/udev/rules.d/45-pcan.rules
EOF
sudo modprobe pcan
cd ..
rm -rf peak-linux-driver-7.15.2.tar.gz
rm -rf peak-linux-driver-7.15.2
cd ..

if [ $? -eq 0 ]; then
  MESSAGE="BarrettHand drivers successfully installed."; green_echo
  cd ..
else
  echo "BarrettHand drivers failed to install."
  cd ..
  exit 1
fi

