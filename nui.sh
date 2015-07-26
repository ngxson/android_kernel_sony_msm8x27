#!/bin/bash

######################################################
#                                                    #
#              NUI Kernel build script               #
#                                                    #
#                  By ngxson (Nui)                   #
#                                                    #
######################################################

THIS_USER=ngxson
GCC_TOOLCHAIN=arm-cortex_a15-linux-gnueabihf-linaro_4.9.4-2015.06
GCC_PREFIX=arm-cortex_a15-linux-gnueabihf

rm "/home/$THIS_USER/out/arch/arm/boot/zImage"

export ARCH=arm
export CROSS_COMPILE="/home/$THIS_USER/$GCC_TOOLCHAIN/bin/$GCC_PREFIX-"

#make the zImage
make O="/home/$THIS_USER/out" cyanogenmod_nicki_defconfig
make O="/home/$THIS_USER/out" -j5

MODULES_DIR="/home/$THIS_USER/modules"
OUT_DIR="/home/$THIS_USER/out"

if [ -a "/home/$THIS_USER/out/arch/arm/boot/zImage" ]; then
	rm -rf $MODULES_DIR
	mkdir $MODULES_DIR
	cd $OUT_DIR
	find . -name '*.ko' -exec cp {} $MODULES_DIR/ \;
	cd $MODULES_DIR
	/home/$THIS_USER/$GCC_TOOLCHAIN/bin/$GCC_PREFIX-strip --strip-unneeded *.ko
	cd "/home/$THIS_USER/nuik"
	
	#now make boot.img
	ZIMAGE_DIR="/home/$THIS_USER/out/arch/arm/boot/zImage"
	RAMDISK_DIR="/home/$THIS_USER/nuik/ramdisk.tar.gz"
	BOOTIMG_OUT_DIR="/home/$THIS_USER/nui.img"
	BOOTIMG_BK_DIR="/home/$THIS_USER/nui_backup.img"
	BOARD_KERNEL_CMDLINE="panic=3 console=ttyHSL0,115200,n8 androidboot.hardware=qcom user_debug=23 msm_rtb.filter=0x3F ehci-hcd.park=3 androidboot.bootdevice=msm_sdcc.1 androidboot.selinux=permissive"
	
	mv "$BOOTIMG_OUT_DIR" "$BOOTIMG_BK_DIR"
	
	/home/$THIS_USER/nuik/mkbootimg --kernel "$ZIMAGE_DIR" --ramdisk "$RAMDISK_DIR" --board "" \
		--cmdline "$BOARD_KERNEL_CMDLINE" --base 0x80200000 --pagesize 4096 \
		--ramdisk_offset 0x02000000 --output "$BOOTIMG_OUT_DIR"	
else
	echo "Failed!"
fi
