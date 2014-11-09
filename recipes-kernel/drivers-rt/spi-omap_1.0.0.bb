DESCRIPTION = "RT module spi omap"
SECTION = "kernel/modules"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0;md5=801f80980d171dd6425610833a22dbe6"
RDEPENDS_${PN} = "kernel"
DEPENDS = "virtual/kernel xenomai"
PR = "r0"

SRCREV = "1"
SRC_URI = "file://rt_spi/spi_omap.c \
	file://rt_spi/Makefile \
	file://spi.conf"

S = "${WORKDIR}/rt_spi"

inherit module

COMPATIBLE_MACHINE = "(uav|overo)"

do_compile () {
	unset CFLAGS CPPFLAGS CXXFLAGS LDFLAGS CC LD CPP
	make KSRC="${STAGING_KERNEL_DIR}"
}

do_install () {
	install -d ${D}${base_libdir}/modules/${KERNEL_VERSION}/kernel/drivers/rt
	install -m 0644 ${S}/spi_omap*${KERNEL_OBJECT_SUFFIX} ${D}${base_libdir}/modules/${KERNEL_VERSION}/kernel/drivers/rt
	
	install -d ${D}/etc/modules-load.d
	install -m 0644 ${WORKDIR}/spi.conf ${D}/etc/modules-load.d/
}

FILES_${PN} += "/etc/modules-load.d/*"
