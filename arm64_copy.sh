A=CONF
if [ "$1" = "fast" ]; then
    A=FAST
fi

BDIR=/usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/
SYSPATH=${BDIR}/sys/GENERIC/
MODULES=${SYSPATH}/modules/

ssh 10.5.0.3 "mkdir -p /tmp/hwpmc/"
scp	${MODULES}/usr/home/br/dev/freebsd-head/sys/modules/hwpmc/hwpmc.ko \
	${BDIR}/usr.sbin/pmcstat/pmcstat \
	${BDIR}/usr.sbin/pmctrace/pmctrace \
	${BDIR}/lib/libpmc/libpmc.so.5 \
	${BDIR}/lib/libopencsd/libopencsd.so.0 \
	${BDIR}/sys/GENERIC/kernel \
	10.5.0.3:/tmp/hwpmc/

