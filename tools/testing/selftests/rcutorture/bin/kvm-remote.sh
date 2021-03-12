#!/bin/bash
# SPDX-License-Identifier: GPL-2.0+
#
# Run a series of tests on remote systems under KVM.
#
# Usage: kvm-remote.sh "systems" [ <kvm.sh args> ]
#	 kvm-remote.sh "systems" /path/to/old/run [ <kvm-again.sh args> ]
#
# Copyright (C) 2021 Facebook, Inc.
#
# Authors: Paul E. McKenney <paulmck@kernel.org>

scriptname=$0
args="$*"

if ! test -d tools/testing/selftests/rcutorture/bin
then
	echo $scriptname must be run from top-level directory of kernel source tree.
	exit 1
fi

KVM="`pwd`/tools/testing/selftests/rcutorture"; export KVM
PATH=${KVM}/bin:$PATH; export PATH
. functions.sh

systems="$1"
if test -z "$systems"
then
	echo $scriptname: Empty list of systems will go nowhere good, giving up.
	exit 1
fi
shift

TD=kvm-again.sh.$$
T=${TMPDIR-/tmp}/$TD
trap 'rm -rf $T' 0
mkdir $T

ds=`date +%Y.%m.%d-%H.%M.%S`-remote
resdir="$T/res"
if echo $1 | grep -q '^--'
then
	# Fresh build.  Create a datestamp unless the caller supplied one.
	datestamp="`echo "$@" | awk -v ds="$ds" '{
		for (i = 1; i < NF; i++) {
			if ($i == "--datestamp") {
				ds = "";
				break;
			}
		}
		if (ds != "")
			print "--datestamp " ds;
	}'`"
	if ! kvm.sh "$@" $datestamp --buildonly > $T/kvm.sh.out 2>&1
	then
		echo $scriptname: kvm.sh failed exit code $?
		exit 2
	fi
	oldrun="`grep -m 1 "^Results directory: " $T/kvm.sh.out | awk '{ print $3 }'`"
	rundir=$resdir/$ds
	if ! kvm-again.sh $oldrun --dryrun --remote --rundir "$rundir" > $T/kvm-again.sh.out 2>&1
	then
		echo $scriptname: kvm-again.sh failed exit code $?
		exit 2
	fi
else
	# Re-use old run.
	oldrun="$1"
	if ! echo $oldrun | grep -q '^/'
	then
		oldrun="`pwd`/$oldrun"
	fi
	shift
	rundir=$resdir/$ds
	if ! kvm-again.sh "$oldrun" "$@" --dryrun --remote --rundir "$rundir" > $T/kvm-again.sh.out 2>&1
	then
		echo $scriptname: kvm-again.sh failed exit code $?
		exit 2
	fi
fi

# Copy bin directory.
if ! cp -a tools/testing/selftests/rcutorture/bin "$resdir/$ds"
then
	echo $scriptname: cp -a for bin directory failed exit code $?
	exit 3
fi

# Create the kvm-remote-N.sh scripts in the bin directory.
awk < "$rundir"/scenarios -v dest="$T/bin" -v rundir="$rundir" '
{
	n = $1;
	sub(/\./, "", n);
	fn = dest "/kvm-remote-" n ".sh"
	scenarios = "";
	for (i = 2; i <= NF; i++)
		scenarios = scenarios " " $i;
	print "kvm-test-1-run-batch.sh" scenarios > fn;
	print "rm " rundir "/remote.run" >> fn;
}'
chmod +x $T/bin/kvm-remote-*.sh
( cd "`dirname $T`"; tar -chzf $T/binres.tgz "$TD/bin" "$TD/res" )

echo oldrun = $oldrun # @@@
echo resdir = $resdir # @@@  Contains all scenarios, sibling to "bin".
echo rundir = $rundir # @@@  "build.remoterun" lives here.
echo Tarball in $T/binres.tgz # @@@
echo Hit enter to terminate: # @@@
read a # @@@
echo Terminated. # @@@
exit 0 # @@@

@@@ Need to push the tarball out and run it, but one step at a time...

@@@ Copy-pasta follows.

if ! cp "$oldrun/batches" $T/batches.oldrun
then
	# Later on, can reconstitute this from console.log files.
	echo Prior run batches file does not exist: $oldrun/batches
	exit 1
fi

if test -f "$oldrun/torture_suite"
then
	torture_suite="`cat $oldrun/torture_suite`"
elif test -f "$oldrun/TORTURE_SUITE"
then
	torture_suite="`cat $oldrun/TORTURE_SUITE`"
else
	echo "Prior run torture_suite file does not exist: $oldrun/{torture_suite,TORTURE_SUITE}"
	exit 1
fi

dryrun=
dur=
default_link="cp -R"
rundir="`pwd`/tools/testing/selftests/rcutorture/res/`date +%Y.%m.%d-%H.%M.%S-again`"

startdate="`date`"
starttime="`get_starttime`"

usage () {
	echo "Usage: $scriptname $oldrun [ arguments ]:"
	echo "       --dryrun"
	echo "       --duration minutes | <seconds>s | <hours>h | <days>d"
	echo "       --link hard|soft|copy"
	echo "       --remote"
	echo "       --rundir /new/res/path"
	exit 1
}

while test $# -gt 0
do
	case "$1" in
	--dryrun)
		dryrun=1
		;;
	--duration)
		checkarg --duration "(minutes)" $# "$2" '^[0-9][0-9]*\(s\|m\|h\|d\|\)$' '^error'
		mult=60
		if echo "$2" | grep -q 's$'
		then
			mult=1
		elif echo "$2" | grep -q 'h$'
		then
			mult=3600
		elif echo "$2" | grep -q 'd$'
		then
			mult=86400
		fi
		ts=`echo $2 | sed -e 's/[smhd]$//'`
		dur=$(($ts*mult))
		shift
		;;
	--link)
		checkarg --link "hard|soft|copy" "$#" "$2" 'hard\|soft\|copy' '^--'
		case "$2" in
		copy)
			arg_link="cp -R"
			;;
		hard)
			arg_link="cp -Rl"
			;;
		soft)
			arg_link="cp -Rs"
			;;
		esac
		shift
		;;
	--remote)
		arg_remote=1
		default_link="cp -as"
		;;
	--rundir)
		checkarg --rundir "(absolute pathname)" "$#" "$2" '^/' '^error'
		rundir=$2
		if test -e "$rundir"
		then
			echo "--rundir $2: Already exists."
			usage
		fi
		shift
		;;
	*)
		echo Unknown argument $1
		usage
		;;
	esac
	shift
done
if test -z "$arg_link"
then
	arg_link="$default_link"
fi

echo ---- Re-run results directory: $rundir

# Copy old run directory tree over and adjust.
mkdir -p "`dirname "$rundir"`"
if ! $arg_link "$oldrun" "$rundir"
then
	echo "Cannot copy from $oldrun to $rundir."
	usage
fi
rm -f "$rundir"/*/{console.log,console.log.diags,qemu_pid,qemu-retval,Warnings,kvm-test-1-run.sh.out,kvm-test-1-run-qemu.sh.out,vmlinux} "$rundir"/log
echo $oldrun > "$rundir/re-run"
if ! test -d "$rundir/../../bin"
then
	$arg_link "$oldrun/../../bin" "$rundir/../.."
fi
for i in $rundir/*/qemu-cmd
do
	cp "$i" $T
	qemu_cmd_dir="`dirname "$i"`"
	kernel_dir="`echo $qemu_cmd_dir | sed -e 's/\.[0-9]\+$//'`"
	jitter_dir="`dirname "$kernel_dir"`"
	kvm-transform.sh "$kernel_dir/bzImage" "$qemu_cmd_dir/console.log" "$jitter_dir" $dur < $T/qemu-cmd > $i
	if test -n "$arg_remote"
	then
		echo "# TORTURE_KCONFIG_GDB_ARG=''" >> $i
	fi
done

# Extract settings from the last qemu-cmd file transformed above.
grep '^#' $i | sed -e 's/^# //' > $T/qemu-cmd-settings
. $T/qemu-cmd-settings

grep -v '^#' $T/batches.oldrun | awk '
BEGIN {
	oldbatch = 1;
}

{
	if (oldbatch != $1) {
		print "kvm-test-1-run-batch.sh" curbatch;
		curbatch = "";
		oldbatch = $1;
	}
	curbatch = curbatch " " $2;
}

END {
	print "kvm-test-1-run-batch.sh" curbatch
}' > $T/runbatches.sh

if test -n "$dryrun"
then
	echo ---- Dryrun complete, directory: $rundir | tee -a "$rundir/log"
else
	( cd "$rundir"; sh $T/runbatches.sh )
	kcsan-collapse.sh "$rundir" | tee -a "$rundir/log"
	echo | tee -a "$rundir/log"
	echo ---- Results directory: $rundir | tee -a "$rundir/log"
	kvm-recheck.sh "$rundir" > $T/kvm-recheck.sh.out 2>&1
	ret=$?
	cat $T/kvm-recheck.sh.out | tee -a "$rundir/log"
	echo " --- Done at `date` (`get_starttime_duration $starttime`) exitcode $ret" | tee -a "$rundir/log"
	exit $ret
fi
