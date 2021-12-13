A=CONF
if [ "$1" = "fast" ]; then
	A=FAST
fi

#./tools/tools/makeroot/makeroot.sh -s 32m -f basic.files riscv.img /home/br/world-riscv/

export MK_CTF=no
make -j8 TARGET=riscv KERN${A}=GENERIC buildkernel || exit 1

#sh ./sys/tools/embed_mfs.sh /usr/obj/usr/home/br/dev/freebsd-head/riscv.riscv64/sys/GENERIC/kernel ./riscv.img || exit 2

# make -j8 TARGET=riscv -DNO_ROOT DESTDIR=/mnt/world-riscv installworld
# makefs -D -f 1000000 -s 8g /mnt/riscv-full.img METALOG

# scp /usr/obj/usr/home/br/dev/freebsd-head/riscv.riscv64/sys/GENERIC/kernel 10.8.0.1:~/virtgl_kernel

#scp /mnt/obj/usr/home/br/dev/freebsd-head/riscv.riscv64/sys/GENERIC/kernel 10.8.0.1:~/virtgl_kernel
scp /mnt/obj/usr/home/br/dev/freebsd-head/riscv.riscv64/sys/GENERIC/kernel 10.4.0.2:~/virtgl_kernel
