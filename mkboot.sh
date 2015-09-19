#!/bin/bash
THIS_USER="ngxson"
ZIMAGE_DIR="/home/$THIS_USER/out/arch/arm/boot/zImage"
RAMDISK_DIR="/home/$THIS_USER/nuik/ramdisk.tar.gz"
BOOTIMG_OUT_DIR="/home/$THIS_USER/nui.img"
BOARD_KERNEL_CMDLINE="panic=3 console=ttyHSL0,115200,n8 androidboot.hardware=qcom user_debug=23 msm_rtb.filter=0x3F ehci-hcd.park=3 androidboot.bootdevice=msm_sdcc.1"

/home/$THIS_USER/nuik/mkbootimg --kernel "$ZIMAGE_DIR" --ramdisk "$RAMDISK_DIR" --board "" \
	--cmdline "$BOARD_KERNEL_CMDLINE" --base 0x80200000 --pagesize 4096 \
	--ramdisk_offset 0x02000000 --output "$BOOTIMG_OUT_DIR"
