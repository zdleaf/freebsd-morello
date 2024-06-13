A=CONF
if [ "$1" = "fast" ]; then
	A=FAST
fi

make -j12 TARGET=arm64 KERN${A}=GENERIC buildkernel || exit 1

cp /usr/obj/usr/home/br/dev/freebsd/arm64.aarch64/sys/GENERIC/kernel /tftpboot/root/boot/kernel/kernel

scp /usr/obj/usr/home/br/dev/freebsd/arm64.aarch64/sys/GENERIC/kernel 10.2.0.156:~/modules/boot/kernel/

scp /usr/obj/usr/home/br/dev/freebsd/arm64.aarch64/sys/GENERIC/modules/usr/home/br/dev/freebsd/sys/modules/coresight/*/*ko 10.2.0.156:~/modules/

scp /usr/obj/usr/home/br/dev/freebsd/arm64.aarch64/sys/GENERIC/modules/usr/home/br/dev/freebsd/sys/modules/hwt/hwt.ko 10.2.0.156:~/modules/
