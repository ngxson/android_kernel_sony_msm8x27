#!/bin/bash
THIS_USER=ngxson
GCC_TOOLCHAIN=arm-cortex_a15-linux-gnueabihf-linaro_4.9.4-2015.06

rm "/home/$THIS_USER/out/arch/arm/boot/zImage"
rm "/home/$THIS_USER/zImage"

export ARCH=arm
export CROSS_COMPILE="/home/$THIS_USER/$GCC_TOOLCHAIN/bin/arm-cortex_a15-linux-gnueabihf-"

make O="/home/$THIS_USER/out" cyanogenmod_nicki_defconfig
make O="/home/$THIS_USER/out" -j5

MODULES_DIR="/home/$THIS_USER/modules"
OUT_DIR="/home/$THIS_USER/out"
if [ -a "/home/$THIS_USER/out/arch/arm/boot/zImage" ];
cp "/home/$THIS_USER/out/arch/arm/boot/zImage" "/home/$THIS_USER/zImage"
then
rm -rf $MODULES_DIR
mkdir $MODULES_DIR
cd $OUT_DIR
find . -name '*.ko' -exec cp {} $MODULES_DIR/ \;
cd $MODULES_DIR
/home/$THIS_USER/$GCC_TOOLCHAIN/bin/arm-cortex_a15-linux-gnueabihf-strip --strip-unneeded *.ko
cd "/home/$THIS_USER/nuik"
else
echo "Failed!"
fi
