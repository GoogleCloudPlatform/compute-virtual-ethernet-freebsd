#$FreeBSD$

.PATH:  ${.CURDIR}

KMOD=   gve
SRCS=   gve_main.c gve_adminq.c gve_utils.c gve_qpl.c gve_rx.c gve_tx.c gve_sysctl.c
SRCS+=	${LINUXKPI_GENSRCS}
SRCS+=  device_if.h bus_if.h pci_if.h
SRCTOP= "/usr/src"

CFLAGS+= -I${SRCTOP}/sys/compat/linuxkpi/common/include

clean:
	rm -f *.o *.kld *.ko .*.o

.include <bsd.kmod.mk>
