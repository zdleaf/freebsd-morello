A=CONF
if [ "$1" = "fast" ]; then
    A=FAST
fi

export MK_CTF=no
# make -j6 TARGET=arm64 NO_ROOT=1 DESTDIR=/home/br/world-arm64 installworld

# make -j6 TARGET=arm64 KERNCONF=MORELLO kernel-toolchain

make -j6 TARGET=arm64 KERN${A}=MORELLO buildkernel || exit 1

# cp /xhome/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel /tftpboot/stratix10/
#cp /home/br/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel /tftpboot/stratix10/

#echo 'setenv serverip 10.5.0.1; setenv ipaddr 10.5.0.33; setenv netmask 255.255.255.0; tftpboot stratix10/kernel'

#echo 'tftpboot 0x2000000 stratix10/kernel; fatwrite mmc 0:1 0x2000000 kernel $filesize'

#echo 'tftpboot 0x2000000 stratix10/loader_lua.efi; bootefi 0x2000000'
#echo 'fatwrite mmc 0:1 0x2000000 kernel $filesize'
#echo 'tftpboot 0x2000000 stratix10/loader_lua.efi'
#echo 'tftpboot 0x3000000 stratix10/socfpga_stratix10_de10_pro.dtb'
#echo 'bootefi 0x2000000 0x3000000'
#echo 'load disk0s1:kernel'

#echo 'tftpboot 0x2000000 stratix10/loader_lua.efi'
#echo 'tftpboot 0x8000000 stratix10/socfpga_stratix10_de10_pro.dtb'
#echo 'bootefi 0x2000000 0x8000000'
#echo 'load disk0s1:kernel'
#echo 'fdt ls'
#echo 'boot'

# echo 'dd if=/root/DE10_Pro-hps.core.rbf of=/dev/fpga0 bs=4m'

#echo 'cp /root/DE10_Pro-hps.core.rbf /var && dd if=/var/DE10_Pro-hps.core.rbf of=/dev/fpga0 bs=4m'

# Uncomment for USB rootfs
#scp /xhome/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel br@10.5.0.3:/tmp/
#ssh 10.5.0.3 "su -m root -c /root/upd.sh"

# Uncomment for tftpboot
if [ $(hostname) = "pie" ]; then
	sudo cp /xhome/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel /tftpboot/root
fi

if [ $(hostname) = "pooh.bsdpad.com" ]; then
	#sudo cp /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel /mnt/tftpboot/root
	#scp /usr/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel 10.8.0.1:/tftpboot/root/
	scp /mnt/obj/usr/home/br/dev/freebsd-head/arm64.aarch64/sys/MORELLO/kernel 10.8.0.1:/tftpboot/root/
fi

# echo sysctl kern.bootfile=/kernel
# echo kldload pmc
# echo pmcstat -T -S INST_RETIRED
