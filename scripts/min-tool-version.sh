#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# Print the minimum supported version of the given tool.

set -e

# When you raise the minimum version, please update
# Documentation/process/changes.rst as well.
min_gcc_version=4.9.0
min_llvm_version=10.0.1
min_icc_version=16.0.3 # temporary
min_binutils_version=2.23.0

# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=63293
# https://lore.kernel.org/r/20210107111841.GN1551@shell.armlinux.org.uk
if [ "$SRCARCH" = arm64 ]; then
	min_gcc_version=5.1.0
fi

eval min_version="\$min_${1}_version"
if [ -z "$min_version" ]; then
	echo "$1: unknown tool" >&2
	exit 1
fi

echo "$min_version"
