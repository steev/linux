#!/usr/bin/env bash

set -e

DTB=$1
if [ "${DTB}" = "" ] ; then
	echo "usage: $0 <dtb>"
	echo ""
	echo "Note that the DTB for the stock Qualcomm distribution is in"
	echo "snapdragon-auto-gen3-lxc-1-0_hlos_dev/apps/apps_proc/poky/build/tmp-glibc/work/sa81x5_rt-oe-linux/linux-msm/5.4-r0/build/arch/arm64/boot/dts/vendor/qcom/sa8195p-v2-adp-air.dtb"
	exit 1
fi

set -x

NUM_CPUS=$(grep -c ^processor /proc/cpuinfo)
# Note that gcc with binutils 2.35 in Fedora 34 fails to cross compile kernel-ark + upstream at the moment
ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make defconfig
ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- make CC=clang -j"${NUM_CPUS}" Image.gz dtbs

QCOM_BUILDER_DIR=$(dirname "$0")

INITRD=$(mktemp)
pushd "${QCOM_BUILDER_DIR}/initrd"
find . | cpio --create --format="newc" --quiet > "${INITRD}"
popd

ABOOT_IMG=arch/arm/boot/sa8540p-boot.img
BOOT_CFG="${QCOM_BUILDER_DIR}/bootimg.cfg"

ZIMAGE_DTB=$(mktemp)
cat arch/arm64/boot/Image.gz "${DTB}" > "${ZIMAGE_DTB}"

#abootimg --create "${ABOOT_IMG}" -f "${BOOT_CFG}" -k "${ZIMAGE_DTB}" -r "${INITRD}"

~/git/android/mkbootimg/mkbootimg.py --output "${ABOOT_IMG}" --kernel "${ZIMAGE_DTB}" --dtb "${DTB}" --pagesize 2048 --kernel_offset 0x80208000 --second_offset 0x81100000 --tags_offset 0x7d00000 --base 0x1208800 --cmdline "root=/dev/sda1 rw rootwait console=ttyMSM0,115200,n8 no_console_suspend=1 androidboot.hardware=qcom androidboot.console=ttyMSM0  lpm_levels.sleep_disabled=1 msm_rtb.filter=0x237 earlycon=qcom_geni,0x884000 fips=0 notests nokaslr ignore_loglevel"

rm -f "${INITRD}" "${ZIMAGE_DTB}"

set +x

echo ""
echo "You can boot the kernel with:"
echo ""
echo "  sudo fastboot boot ${ABOOT_IMG}"
