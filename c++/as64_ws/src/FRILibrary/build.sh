#!/usr/bin/env bash

# Install dependencies
sudo apt-get update
sudo apt-get install -y gcc-multilib g++-multilib

path() {
    [[ $1 = /* ]] && echo "$1" || echo "$PWD/${1#./}"
}

sed -i '/The path for FRILibrary/d' ~/.bashrc

SCRIPT_DIR=`path $0`
FRIL_PATH=`dirname $SCRIPT_DIR`

cd Linux
mkdir -p x64/debug/bin
mkdir -p x64/release/bin
mkdir -p x64/debug/lib
mkdir -p x64/release/lib
mkdir -p x64/debug/obj
mkdir -p x64/release/obj
mkdir -p x86/debug/bin
mkdir -p x86/release/bin
mkdir -p x86/debug/lib
mkdir -p x86/release/lib
mkdir -p x86/debug/obj
mkdir -p x86/release/obj
make clean all

if [ $? -eq 0 ]; then
  echo "export FRIL_PATH=$FRIL_PATH  # The path for FRILibrary" >> ~/.bashrc
  echo "FRIL library successfully installed. Please source your bashrc file."
  cd ..
else
  echo "FRIL library failed to be built installed."
  cd ..
  exit 1
fi

