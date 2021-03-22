# $FreeBSD$

.include <kmod.opts.mk>

KMOD=	vmm

SRCS=	opt_acpi.h opt_ddb.h device_if.h bus_if.h pci_if.h pcib_if.h acpi_if.h

CFLAGS+= -DVMM_KEEP_STATS
CFLAGS+= -I${SRCTOP}/sys/${MACHINE}/vmm
CFLAGS+= -I${SRCTOP}/sys/${MACHINE}/vmm/io

# generic vmm support
.PATH: ${SRCTOP}/sys/${MACHINE}/vmm
SRCS+=	vmm.c		\
	vmm_dev.c	\
	vmm_instruction_emul.c	\
	vmm_mem.c	\
	vmm_stat.c

.if ${MACHINE_CPUARCH} == "aarch64"
DPSRCS+=	hyp_assym.h

SRCS+=	vmm_arm64.c \
	vmm_psci.c \
	vmm_reset.c \
	vmm_hyp.S \
	vmm_mmu.c

.PATH: ${SRCTOP}/sys/${MACHINE}/vmm/io
SRCS+=	vgic_its.c \
	vgic_v3.c \
	vtimer.c

CLEANFILES=	hyp_assym.h hyp_genassym.o
OBJS_DEPEND_GUESS.vmm_hyp.o+= hyp_assym.h

.else if ${MACHINE_CPUARCH} == "amd64"
DPSRCS+=	vmx_assym.h svm_assym.h
DPSRCS+=	vmx_genassym.c svm_genassym.c offset.inc

CFLAGS+= -I${SRCTOP}/sys/amd64/vmm/intel
CFLAGS+= -I${SRCTOP}/sys/amd64/vmm/amd

SRCS+=	vmm_host.c	\
	vmm_ioport.c	\
	vmm_lapic.c	\
	vmm_util.c	\
	x86.c

.PATH: ${SRCTOP}/sys/${MACHINE}/vmm/io
SRCS+=	iommu.c		\
	ppt.c           \
	vatpic.c	\
	vatpit.c	\
	vhpet.c		\
	vioapic.c	\
	vlapic.c	\
	vpmtmr.c	\
	vrtc.c

# intel-specific files
.PATH: ${SRCTOP}/sys/amd64/vmm/intel
SRCS+=	ept.c		\
	vmcs.c		\
	vmx_msr.c	\
	vmx_support.S	\
	vmx.c		\
	vtd.c

# amd-specific files
.PATH: ${SRCTOP}/sys/amd64/vmm/amd
SRCS+=	vmcb.c		\
	amdiommu.c	\
	ivhd_if.c	\
	ivhd_if.h	\
	svm.c		\
	svm_support.S	\
	npt.c		\
	ivrs_drv.c	\
	amdvi_hw.c	\
	svm_msr.c

.if ${KERN_OPTS:MBHYVE_SNAPSHOT} != ""
SRCS+=	vmm_snapshot.c
.endif

CLEANFILES=	vmx_assym.h vmx_genassym.o svm_assym.h svm_genassym.o

OBJS_DEPEND_GUESS.vmx_support.o+= vmx_assym.h
OBJS_DEPEND_GUESS.svm_support.o+= svm_assym.h
.endif

hyp_assym.h:    hyp_genassym.o
	sh ${SYSDIR}/kern/genassym.sh hyp_genassym.o > ${.TARGET}

vmx_assym.h:    vmx_genassym.o
	sh ${SYSDIR}/kern/genassym.sh vmx_genassym.o > ${.TARGET}

svm_assym.h:    svm_genassym.o
	sh ${SYSDIR}/kern/genassym.sh svm_genassym.o > ${.TARGET}

vmx_support.o:
	${CC} -c -x assembler-with-cpp -DLOCORE ${CFLAGS} \
	    ${.IMPSRC} -o ${.TARGET}

svm_support.o:
	${CC} -c -x assembler-with-cpp -DLOCORE ${CFLAGS} \
	    ${.IMPSRC} -o ${.TARGET}

hyp_genassym.o: offset.inc
	${CC} -c ${CFLAGS:N-flto:N-fno-common} -fcommon ${.IMPSRC}

vmx_genassym.o: offset.inc
	${CC} -c ${CFLAGS:N-flto:N-fno-common} -fcommon ${.IMPSRC}

svm_genassym.o: offset.inc
	${CC} -c ${CFLAGS:N-flto:N-fno-common} -fcommon ${.IMPSRC}

.include <bsd.kmod.mk>
