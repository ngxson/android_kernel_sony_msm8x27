#!/bin/bash
rm "/home/ngxson/out/arch/arm/boot/zImage"

export ARCH=arm
export CROSS_COMPILE="/home/ngxson/arm-cortex_a15-linux-gnueabihf-linaro_4.9.3-2015.03/bin/arm-cortex_a15-linux-gnueabihf-"

make O="/home/ngxson/out" cyanogenmod_nicki_defconfig
make O="/home/ngxson/out" -j5

MODULES_DIR="/home/ngxson/modules"
OUT_DIR="/home/ngxson/out"
if [ -a "/home/ngxson/out/arch/arm/boot/zImage" ];
then
rm -rf $MODULES_DIR
mkdir $MODULES_DIR
cd $OUT_DIR
find . -name '*.ko' -exec cp {} $MODULES_DIR/ \;
cd $MODULES_DIR
/home/ngxson/arm-cortex_a15-linux-gnueabihf-linaro_4.9.3-2015.03/bin/arm-cortex_a15-linux-gnueabihf-strip --strip-unneeded *.ko
cd "/home/ngxson/nuik"
else
echo "Failed!"
fi
