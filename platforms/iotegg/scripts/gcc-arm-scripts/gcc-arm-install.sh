#!/bin/bash
sudo yum -y install glibc.i686
sudo mv gcc-arm-none-eabi-5_2-2015q4 /opt/gcc-arm-none-eabi-5_2-2015q4 
rm gcc-arm-none-eabi-5_2-2015q4-20151219-linux.tar.bz2
/opt/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-gcc --version

