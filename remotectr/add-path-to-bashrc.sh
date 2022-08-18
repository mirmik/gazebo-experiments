#!/bin/bash
#add path to bashrc
CURPATH=`pwd`
echo "export PATH=${CURPATH}:\${PATH}" >> ~/.bashrc
