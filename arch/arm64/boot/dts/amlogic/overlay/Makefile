# SPDX-License-Identifier: GPL-2.0
dtbo-$(CONFIG_ARCH_MESON) += \
	meson-g12a-gpio-8-led.dtbo \
	meson-g12a-gpio-10-led.dtbo \
	meson-g12a-i2c-ao-m0-gpioao-2-gpioao-3.dtbo \
	meson-g12a-i2c-ee-m1-gpioh-6-gpioh-7.dtbo \
	meson-g12a-i2c-ee-m1-gpiox-10-gpiox-11.dtbo \
	meson-g12a-i2c-ee-m3-gpioa-14-gpioa-15.dtbo \
	meson-g12a-opp-2ghz.dtbo \
	meson-g12a-pwmao-a-on-gpioao-11.dtbo \
	meson-g12a-pwm-c-on-gpiox-8.dtbo \
	meson-g12a-spi-spidev.dtbo \
	meson-g12a-uart-ao-a-on-gpioao-0-gpioao-1.dtbo \
	meson-g12a-uart-ao-b-on-gpioao-2-gpioao-3.dtbo \
	meson-g12a-uart-ao-b-on-gpioao-8-gpioao-9.dtbo \
	meson-g12a-uart-ee-c.dtbo

scr-$(CONFIG_ARCH_MESON) += \
	meson-fixup.scr

dtbotxt-$(CONFIG_ARCH_MESON) += \
	README.meson-overlays

targets += $(dtbo-y) $(scr-y) $(dtbotxt-y)

always			:= $(dtbo-y) $(scr-y) $(dtbotxt-y)
clean-files		:= *.dtbo *.scr
