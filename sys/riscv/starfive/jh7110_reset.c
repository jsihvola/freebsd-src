/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2023 Jari Sihvola <jsihv@gmx.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
*/

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/time.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/extres/hwreset/hwreset.h>
#include "hwreset_if.h"

#include <dt-bindings/reset/starfive-jh7110.h>

/* offsets for reset registers */
#define AONCRG_RESET_SELECTOR	0x38
#define ISPCRG_RESET_SELECTOR	0x38
#define VOUTCRG_RESET_SELECTOR	0x48
#define STGCRG_RESET_SELECTOR	0x74
#define AONCRG_RESET_STATUS	0x3c
#define ISPCRG_RESET_STATUS	0x3c
#define VOUTCRG_RESET_STATUS	0x4c
#define STGCRG_RESET_STATUS	0x78
#define SYSCRG_RESET_SELECTOR0	0x2f8
#define SYSCRG_RESET_SELECTOR1	0x2fc
#define SYSCRG_RESET_SELECTOR2	0x300
#define SYSCRG_RESET_SELECTOR3	0x304
#define SYSCRG_RESET_STATUS0	0x308
#define SYSCRG_RESET_STATUS1	0x30c
#define SYSCRG_RESET_STATUS2	0x310
#define SYSCRG_RESET_STATUS3	0x314


static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-reset", 1 },
	{ NULL,                    0 }
};

struct jh7110_reset_softc {
	struct mtx              mtx;
	struct resource         *sys_mem;
	struct resource         *stg_mem;
	struct resource         *aon_mem;
	struct resource         *isp_mem;
	struct resource         *vout_mem;
};

struct jh7110_reset_carrier {
	struct resource         *res;
	uint64_t                status_offset;
	uint64_t                selector_offset;
};

enum JH7110_CRG {
	SYS0,
	SYS1,
	SYS2,
	SYS3,
	STG,
	AON,
	ISP,
	VOUT,
};

#define RESET_READ(iodata, ss)                                  \
          bus_read_4((iodata).res, (iodata).ss##_offset)
#define RESET_WRITE(iodata, ss, val)                            \
          bus_write_4((iodata).res, (iodata).ss##_offset, val)

static int
jh7110_io_assign(struct jh7110_reset_softc *sc,
		 intptr_t id, struct jh7110_reset_carrier *iodata)
{

	uint32_t crg = id / 32;

	if (crg == SYS0) {
		iodata->res = sc->sys_mem;
		iodata->status_offset = SYSCRG_RESET_STATUS0;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR0;
	}
	else if (crg == SYS1) {
		iodata->res = sc->sys_mem;
		iodata->status_offset = SYSCRG_RESET_STATUS1;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR1;
	}
	else if (crg == SYS2) {
		iodata->res = sc->sys_mem;
		iodata->status_offset = SYSCRG_RESET_STATUS2;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR2;
	}
	else if (crg == SYS3) {
		iodata->res = sc->sys_mem;
		iodata->status_offset = SYSCRG_RESET_STATUS3;
		iodata->selector_offset = SYSCRG_RESET_SELECTOR3;
	}
	else if (crg == STG) {
		iodata->res = sc->stg_mem;
		iodata->status_offset = STGCRG_RESET_STATUS;
		iodata->selector_offset = STGCRG_RESET_SELECTOR;
	}
	else if (crg == AON) {
		iodata->res = sc->aon_mem;
		iodata->status_offset = AONCRG_RESET_STATUS;
		iodata->selector_offset = AONCRG_RESET_SELECTOR;
	}
	else if (crg == ISP) {
		iodata->res = sc->isp_mem;
		iodata->status_offset = ISPCRG_RESET_STATUS;
		iodata->selector_offset = ISPCRG_RESET_SELECTOR;
	}
	else if (crg == VOUT) {
		iodata->res = sc->vout_mem;
		iodata->status_offset = VOUTCRG_RESET_STATUS;
		iodata->selector_offset = VOUTCRG_RESET_SELECTOR;
	}

	return (0);
}

static int
jh7110_reset_assert(device_t dev, intptr_t id, bool assert)
{

  	struct jh7110_reset_softc *sc;
	struct jh7110_reset_carrier iodata;
	struct timeval time_end, time_comp;
	uint32_t regvalue, ready = 0;
	uint32_t bitmask = 1UL << id % 32;
	int ret;

	sc = device_get_softc(dev);
	jh7110_io_assign(sc, id, &iodata);

	mtx_lock(&sc->mtx);

	regvalue = RESET_READ(iodata, selector);

	if (assert)
		regvalue |= bitmask;
	else
		regvalue &= ~bitmask;
	RESET_WRITE(iodata, selector, regvalue);

	/* Timeout to prevent perpetual hanging
	   when deasserting with gated clocks */

	getmicrotime(&time_end);
	time_end.tv_usec += 1000; /* Timeout is 1000 microseconds */

	if (!assert)
		ready = bitmask;
	
	for(;;)	{
		regvalue = RESET_READ(iodata, status);

		if ((regvalue & bitmask) == ready)
			break;

		getmicrotime(&time_comp);
		if (time_end.tv_usec <= time_comp.tv_usec) {
			regvalue = RESET_READ(iodata, status);
			break;
		}
	}

	ret = ((regvalue & bitmask) == ready) ? 0 : -ETIMEDOUT;
	
	mtx_unlock(&sc->mtx);
	
	return (ret);
}

static int
jh7110_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
	struct jh7110_reset_softc *sc;
	struct jh7110_reset_carrier iodata;
	uint32_t reg_value;
	uint32_t bitmask;

	sc = device_get_softc(dev);

	jh7110_io_assign(sc, id, &iodata);

	mtx_lock(&sc->mtx);
	reg_value = RESET_READ(iodata, status);
	bitmask = (1UL << id % 32);
	mtx_unlock(&sc->mtx);

	*reset = (reg_value & bitmask) == 0;

	return (0);
}

static int
jh7110_reset_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if(ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 reset driver");

	return (0);
}

static int
jh7110_reset_attach(device_t dev)
{
	struct jh7110_reset_softc *sc;
	phandle_t node;
	int rid, err;

	sc = device_get_softc(dev);
	node = ofw_bus_get_node(dev);

	rid = 0;
	err = ofw_bus_find_string_index(node, "reg-names", "syscrg", &rid);

	if (err != 0) {
		device_printf(dev, "Cannot get 'syscrg' index\n");
		return (ENXIO);
	}
	sc->sys_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->sys_mem == NULL) {
		device_printf(dev, "Cannot allocate 'syscrg'\n");
		return (ENXIO);
	}

	rid = 0;
	err = ofw_bus_find_string_index(node, "reg-names", "stgcrg", &rid);
	if (err != 0) {
		device_printf(dev, "Cannot get 'stg' index\n");
		return (ENXIO);
	}
	sc->stg_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->stg_mem == NULL) {
		device_printf(dev, "Cannot allocate 'stgcrg'\n");
		return (ENXIO);
	}

	rid = 0;
	err = ofw_bus_find_string_index(node, "reg-names", "aoncrg", &rid);
	if (err != 0) {
		device_printf(dev, "Cannot get 'aon' index\n");
		return (ENXIO);
	}
	sc->aon_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->aon_mem == NULL) {
		device_printf(dev, "Cannot allocate 'aoncrg'\n");
		return (ENXIO);
	}

	rid = 0;
	err = ofw_bus_find_string_index(node, "reg-names", "ispcrg", &rid);
	if (err != 0) {
		device_printf(dev, "Cannot get 'isp' index\n");
		return (ENXIO);
	}
	sc->isp_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->isp_mem == NULL) {
		device_printf(dev, "Cannot allocate 'isp'\n");
		return (ENXIO);
	}

	rid = 0;
	err = ofw_bus_find_string_index(node, "reg-names", "voutcrg", &rid);
	if (err != 0) {
		device_printf(dev, "Cannot get 'vout' index\n");
		return (ENXIO);
	}
	sc->vout_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
 	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->vout_mem == NULL) {
		device_printf(dev, "Cannot allocate 'vout'\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	hwreset_register_ofw_provider(dev);

	return (0);
}

static device_method_t jh7110_reset_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,         jh7110_reset_probe),
	DEVMETHOD(device_attach,        jh7110_reset_attach),

	/* Reset interface */
	DEVMETHOD(hwreset_assert,       jh7110_reset_assert),
	DEVMETHOD(hwreset_is_asserted,  jh7110_reset_is_asserted),

	DEVMETHOD_END
};

static driver_t jh7110_reset_driver = {
	"jh7110_reset",
	jh7110_reset_methods,
	sizeof(struct jh7110_reset_softc),
};

EARLY_DRIVER_MODULE(jh7110_reset, simplebus, jh7110_reset_driver, 0, 0,
    BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(jh7110_reset, 1);
