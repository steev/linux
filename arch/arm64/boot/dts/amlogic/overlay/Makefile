# SPDX-License-Identifier: GPL-2.0
dtbo-$(CONFIG_ARCH_MESON) += \
	meson-g12a-i2c-ee-m1.dtbo \
	meson-g12a-i2c-ee-m3.dtbo \
	meson-g12a-i2c-ao-m0.dtbo \
	meson-g12a-pwmao-a-on-gpioao-11.dtbo \
	meson-g12a-pwm-c-on-gpiox-8.dtbo \
	meson-g12a-uart-ee-c.dtbo

scr-$(CONFIG_ARCH_MESON) += \
	meson-fixup.scr

dtbotxt-$(CONFIG_ARCH_MESON) += \
	README.meson-overlays

targets += $(dtbo-y) $(scr-y) $(dtbotxt-y)

always			:= $(dtbo-y) $(scr-y) $(dtbotxt-y)
clean-files		:= *.dtbo *.scr
