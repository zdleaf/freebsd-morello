A=CONF
if [ "$1" = "fast" ]; then
    A=FAST
fi

#export MAKEOBJDIRPREFIX=/home/br/obj/
#make -j6 TARGET=arm64 KERN${A}=GENERIC buildkernel || exit 1

VARS=`make TARGET=arm64 buildenvvars`
eval $VARS make -C lib/libpmcstat || exit 1
eval $VARS make -C lib/libpmc || exit 1

#cp /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/lib/libpmc/libpmc.so.5 /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/tmp/usr/lib/libpmc.so.5

#eval $VARS make -j8 -C lib/libopencsd clean all || exit 1
eval $VARS make -j8 -C lib/libopencsd all || exit 1

eval $VARS make -C usr.sbin/pmcstat || exit 1
eval $VARS make -C usr.sbin/pmctrace clean all || exit 1

sh ./arm64_copy.sh
