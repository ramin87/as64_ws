#!/bin/bash

AS64_WS_DIR=${PWD}
INSTALL_SCRIPTS_DIR=$AS64_WS_DIR/install
cd $INSTALL_SCRIPTS_DIR

if [ $# -eq 0 ]; then
  source main.sh
else
  source main.sh $1
fi
