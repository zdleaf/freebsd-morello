A=CONF
if [ "$1" = "fast" ]; then
	A=FAST
fi

# make -j6 TARGET=arm64 NO_ROOT=1 DESTDIR=/home/br/world-arm64 installworld
# make -j6 TARGET=arm64 KERNCONF=GENERIC kernel-toolchain

export MK_CTF=no

make -j8 TARGET=arm64 KERN${A}=EDGE buildkernel || exit 1

#cp /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/EDGE/kernel /mnt/tftpboot/edge/
# cp /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/EDGE/rk3399-khadas-edge-captain-mipi.dtb /tftpboot/edge/rk3399-khadas-edge-captain.dtb

#echo "setenv serverip 10.8.0.1 ; setenv ipaddr 10.8.0.44; usb start; tftpboot edge/kernel; fatwrite mmc 1 0x1000000 kernel \$filesize; tftpboot 0x1000000 loader.efi; tftpboot 0x83000000 dragonboard410c.dtb; usb stop; bootefi 0x81000000 0x83000000"

#echo "tftpboot 0x81000000 edge/loader_lua.efi"
#echo "tftpboot 0x83000000 edge/rk3399-khadas-edge-captain.dtb"

#echo "setenv serverip 10.8.0.1; setenv ipaddr 10.8.0.44; ping 10.8.0.1; sleep 1; tftpboot edge/kernel; fatwrite mmc 1 0x800800 kernel \$filesize; tftpboot 0x81000000 edge/loader_lua.efi; tftpboot 0x83000000 edge/rk3399-khadas-edge-captain.dtb; bootefi 0x81000000 0x83000000"

#echo "set currdev=disk0p3"
#echo "load disk0p1:kernel"
#echo "boot"

#scp /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/EDGE/kernel 10.8.0.1:/tftpboot/edge/
scp /mnt/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/EDGE/kernel 10.8.0.1:/tftpboot/edge/
