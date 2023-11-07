/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2016 Michal Meloun <mmel@FreeBSD.org>
 * Copyright (c) 2020 Oskar Holmlund <oskar.holmlund@ohdata.se>
 * Copyright (c) 2022 Mitchell Horne <mhorne@FreeBSD.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/resource.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>

#include <dev/extres/clk/clk.h>
#include <dev/extres/clk/clk_div.h>
#include <dev/extres/clk/clk_gate.h>
#include <dev/extres/clk/clk_fixed.h>
#include <dev/extres/clk/clk_mux.h>
#include <dev/extres/clk/clk_link.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/clock/starfive-jh7110-clkgen.h>

#include <riscv/starfive/clk/jh7110_clk.h>
//#include <dev/extres/syscon/syscon.h>

#include "clkdev_if.h"
//#include "syscon_if.h"

/* This file contains general functionality for jh7110 clock generator driver
   plus more specifically initializes the clocks of sys and aon groups */

static struct ofw_compat_data compat_data[] = {
	{ "starfive,jh7110-clkgen",	1 },
	{ NULL,				0 }
};

struct jh7110_clkgen_softc {
	struct mtx		mtx;
	struct clkdom		*clkdom;
	int			*rid;
  
	struct resource		*sys_mem_res;
	struct resource         *stg_mem_res;
	struct resource         *aon_mem_res;
	struct resource         *isp_mem_res;
	struct resource         *vout_mem_res;
  	struct resource         *syscon_mem_res;
};

static struct resource_spec res_spec[] = {
        { SYS_RES_MEMORY, 0, RF_ACTIVE | RF_SHAREABLE },
	RESOURCE_SPEC_END
};

#define	RD4(sc, reg)		bus_read_4(&(sc)->sys_mem_res[0], (reg))
#define	WR4(sc, reg, val)	bus_write_4(&(sc)->sys_mem_res[0], (reg), (val))

struct resource *
jh7110_clk_get_memres(struct clknode *clk, uint32_t flags)
{
	struct jh7110_clkgen_softc *sc;

	sc = device_get_softc(clknode_get_device(clk));

	printf("jh7110_clk_get_memres(), name: %s, flag: %d\n", clknode_get_name(clk), flags);
	
	if (flags & JH7110_CLK_SYS)
		return sc->sys_mem_res;
	else if (flags & JH7110_CLK_AON)
		return sc->aon_mem_res;
	else if (flags & JH7110_CLK_ISP)
		return sc->isp_mem_res;
	else if (flags & JH7110_CLK_STG)
		return sc->stg_mem_res;
	else if (flags & JH7110_CLK_VOUT)
		return sc->vout_mem_res;

	printf("Error, clknode has no memory resource flag\n");
	
	return (NULL);
}

/* parents for non-pll SYS clocks */
static const char *cpu_root_p[] = { "osc", "pll0_out" };
static const char *cpu_core_p[] = { "cpu_root" };
static const char *cpu_bus_p[] = { "cpu_core" };
static const char *perh_root_p[] = { "pll0_out", "pll2_out" };
static const char *bus_root_p[] = { "osc", "pll2_out" };

static const char *apb0_p[] = { "apb_bus" };

static const char *u0_sys_iomux_pclk_p[] = { "apb12" };

static const char *u0_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u0_dw_uart_clk_core_p[] = { "osc" };
static const char *u1_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u1_dw_uart_clk_core_p[] = { "osc" };
static const char *u2_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u2_dw_uart_clk_core_p[] = { "osc" };
static const char *u3_dw_uart_clk_apb_p[] = { "apb0" };
static const char *u3_dw_uart_clk_core_p[] = { "perh_root" };

static const char *apb_bus_func_p[] = { "stg_axiahb" };
static const char *stg_axiahb_p[] = { "axi_cfg0" };
static const char *ahb0_p[] = { "stg_axiahb" };
static const char *axi_cfg0_p[] = { "bus_root" };

static const char *u0_dw_sdio_clk_ahb_p[] = { "ahb0" };
static const char *u1_dw_sdio_clk_ahb_p[] = { "ahb0" };
static const char *u0_dw_sdio_clk_sdcard_p[] = { "axi_cfg0" };
static const char *u1_dw_sdio_clk_sdcard_p[] = { "axi_cfg0" };

static const char *gmac_src_p[] = { "gmacusb_root" };

static const char *gmac0_gtxclk_p[] = { "gmacusb_root" };
static const char *gmac0_ptp_p[] = { "gmac_src" };
static const char *gmac0_gtxc_p[] = { "gmac0_gtxclk" };

static const char *gmac1_gtxclk_p[] = { "gmacusb_root" };

static const char *u1_dw_gmac5_axi64_clk_tx_p[] = { "gmac1_gtxclk", "gmac1_rmii_rtx" };
static const char *u1_dw_gmac5_axi64_clk_ptp_p[] = { "gmac_src" };
static const char *u1_dw_gmac5_axi64_clk_ahb_p[] = { "ahb0" };
static const char *u1_dw_gmac5_axi64_clk_axi_p[] = { "stg_axiahb" };
static const char *gmac1_gtxc_p[] = { "gmac1_gtxclk" };
static const char *gmac1_rmii_rtx_p[] = { "gmac1_rmii_refin" };

/* parents for SYS pll fixed clocks and pll_out clocks */
static const char *gmacusb_root_p[] = { "pll0_out" };		
static const char *apb12_p[] = { "apb_bus" };
static const char *apb_bus_p[] = { "u2_pclk_mux_pclk" };
static const char *u2_pclk_mux_pclk_p[] = { "u2_pclk_mux_func_pclk" };
static const char *u2_pclk_mux_func_pclk_p[] = { "apb_bus_func" };
static const char *aon_ahb_p[] = { "stg_axiahb" };

static const char *pll_parents[] = { "osc" };

/* non-pll SYS clocks */
static const struct jh7110_clk_def sys_clks[] = {
	JH7110_MUX(JH7110_CPU_ROOT, "cpu_root", cpu_root_p),
	JH7110_DIV(JH7110_CPU_CORE, "cpu_core", cpu_core_p, 7),
	JH7110_DIV(JH7110_CPU_BUS, "cpu_bus", cpu_bus_p, 2),
	JH7110_GATEDIV(JH7110_PERH_ROOT, "perh_root", perh_root_p, 2),
	JH7110_MUX(JH7110_BUS_ROOT, "bus_root", bus_root_p),

	JH7110_GATE(JH7110_GMAC5_CLK_AHB, "u1_dw_gmac5_axi64_clk_ahb",
		    u1_dw_gmac5_axi64_clk_ahb_p),
	
	JH7110_GATE(JH7110_APB0, "apb0", apb0_p),
	JH7110_GATE(JH7110_SYS_IOMUX_PCLK, "u0_sys_iomux_pclk",
		    u0_sys_iomux_pclk_p),
	JH7110_GATE(JH7110_UART0_CLK_APB, "u0_dw_uart_clk_apb",
		    u0_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_UART0_CLK_CORE, "u0_dw_uart_clk_core",
		    u0_dw_uart_clk_core_p),
	JH7110_GATE(JH7110_UART1_CLK_APB, "u1_dw_uart_clk_apb",
		    u1_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_UART1_CLK_CORE, "u1_dw_uart_clk_core",
		    u1_dw_uart_clk_core_p),
	JH7110_GATE(JH7110_UART2_CLK_APB, "u2_dw_uart_clk_apb",
		    u2_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_UART2_CLK_CORE, "u2_dw_uart_clk_core",
		    u2_dw_uart_clk_core_p),
	JH7110_GATE(JH7110_UART3_CLK_APB, "u3_dw_uart_clk_apb",
		    u3_dw_uart_clk_apb_p),
	JH7110_GATE(JH7110_UART3_CLK_CORE, "u3_dw_uart_clk_core",
		    u3_dw_uart_clk_core_p),
	
	JH7110_DIV(JH7110_AXI_CFG0, "axi_cfg0", axi_cfg0_p, 3),
	JH7110_DIV(JH7110_STG_AXIAHB, "stg_axiahb", stg_axiahb_p, 2),
	JH7110_GATE(JH7110_AHB0, "ahb0", ahb0_p),
	JH7110_DIV(JH7110_APB_BUS_FUNC, "apb_bus_func",	apb_bus_func_p, 8),

	JH7110_GATE(JH7110_SDIO0_CLK_AHB, "u0_dw_sdio_clk_ahb",
		    u0_dw_sdio_clk_ahb_p),
	JH7110_GATE(JH7110_SDIO1_CLK_AHB, "u1_dw_sdio_clk_ahb",
		    u1_dw_sdio_clk_ahb_p),
	JH7110_GATEDIV(JH7110_SDIO0_CLK_SDCARD, "u0_dw_sdio_clk_sdcard",
		    u0_dw_sdio_clk_sdcard_p, 15),
	JH7110_GATEDIV(JH7110_SDIO1_CLK_SDCARD, "u1_dw_sdio_clk_sdcard",
		    u1_dw_sdio_clk_sdcard_p, 15),
	
	JH7110_DIV(JH7110_GMAC_SRC, "gmac_src", gmac_src_p, 7),
	
	JH7110_GATEDIV(JH7110_GMAC0_GTXCLK, "gmac0_gtxclk", gmac0_gtxclk_p, 15),
	JH7110_GATEDIV(JH7110_GMAC0_PTP, "gmac0_ptp", gmac0_ptp_p, 31),
	JH7110_GATE(JH7110_GMAC0_GTXC, "gmac0_gtxc", gmac0_gtxc_p),

	JH7110_DIV(JH7110_GMAC1_GTXCLK, "gmac1_gtxclk", gmac1_gtxclk_p, 15),
	JH7110_GATEMUX(JH7110_GMAC5_CLK_TX, "u1_dw_gmac5_axi64_clk_tx",
		       u1_dw_gmac5_axi64_clk_tx_p),
	JH7110_GATEDIV(JH7110_GMAC5_CLK_PTP, "u1_dw_gmac5_axi64_clk_ptp",
		       u1_dw_gmac5_axi64_clk_ptp_p, 31),
	JH7110_GATE(JH7110_GMAC5_CLK_AXI, "u1_dw_gmac5_axi64_clk_axi",
		    u1_dw_gmac5_axi64_clk_axi_p),
	JH7110_GATE(JH7110_GMAC1_GTXC, "gmac1_gtxc", gmac1_gtxc_p),
	JH7110_DIV(JH7110_GMAC1_RMII_RTX, "gmac1_rmii_rtx",
		   gmac1_rmii_rtx_p, 30),
};

/* pll_out clks (SYS) */
static struct jh7110_pll_def pll_out_clks[] = {
	{
		.clkdef.id = JH7110_PLL0_OUT,
		.clkdef.name = "pll0_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
	{
		.clkdef.id = JH7110_PLL1_OUT,
		.clkdef.name = "pll1_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
	{
		.clkdef.id = JH7110_PLL2_OUT,
		.clkdef.name = "pll2_out",
		.clkdef.parent_names = pll_parents,
		.clkdef.parent_cnt = nitems(pll_parents),
		.clkdef.flags = CLK_NODE_STATIC_STRINGS,
	},
};

/* SYS fixed pll clocks */
static struct clk_fixed_def sys_pll_clks[] = {

	JH7110_PLL(JH7110_GMACUSB_ROOT, "gmacusb_root", gmacusb_root_p),
	JH7110_PLL(JH7110_PCLK2_MUX_FUNC_PCLK, "u2_pclk_mux_func_pclk",
		   u2_pclk_mux_func_pclk_p),
	JH7110_PLL(JH7110_U2_PCLK_MUX_PCLK, "u2_pclk_mux_pclk",
		   u2_pclk_mux_pclk_p),
	JH7110_PLL(JH7110_APB_BUS, "apb_bus", apb_bus_p),
	JH7110_PLL(JH7110_APB12, "apb12", apb12_p),

	JH7110_PLL(JH7110_AON_AHB, "aon_ahb", aon_ahb_p),
};

/* non-pll AON clocks & parents */
static const char *u0_dw_gmac5_axi64_clk_ahb_p[] = { "aon_ahb" };
static const char *u0_dw_gmac5_axi64_clk_axi_p[] = { "aon_ahb" };
static const char *u0_dw_gmac5_axi64_clk_tx_p[] = { "gmac0_gtxclk", "gmac0_rmii_rtx" };
static const char *gmac0_rmii_rtx_p[] = { "gmac0_rmii_refin" };

static const struct jh7110_clk_def aon_clks[] = {
	JH7110_GATE(JH7110_U0_GMAC5_CLK_AHB, "u0_dw_gmac5_axi64_clk_ahb",
		    u0_dw_gmac5_axi64_clk_ahb_p),
	JH7110_GATE(JH7110_U0_GMAC5_CLK_AXI, "u0_dw_gmac5_axi64_clk_axi", 
		    u0_dw_gmac5_axi64_clk_axi_p),
	JH7110_DIV(JH7110_GMAC0_RMII_RTX, "gmac0_rmii_rtx",
		    gmac0_rmii_rtx_p, 30),
	JH7110_GATEMUX(JH7110_U0_GMAC5_CLK_TX, "u0_dw_gmac5_axi64_clk_tx",
		    u0_dw_gmac5_axi64_clk_tx_p),
};

/* Default DT mapper. */
static int
jh7110_ofw_map(struct clkdom *clkdom, uint32_t ncells,
    phandle_t *cells, struct clknode **clk)
{

	int id = cells[0];

	printf("jh7110_ofw_map(), ncells: %u, cells[0]: %u\n", ncells, cells[0]);
  
	if (ncells == 1) {
		printf("jh7110_ofw_map, ncells == 1\n");
		*clk = clknode_find_by_id(clkdom, id);
	}
	else {
		printf("jh7110_ofw_map, nells != 1\n");
		return  (ERANGE);
	}
	if (*clk == NULL) {
		printf("jh7110_ofw_map 3\n");
		return (ENXIO);
	}
	return (0);
}

static int
jh7110_clkgen_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive Clock Controller");

	return (BUS_PROBE_DEFAULT);
}

static void
jh7110_print_clkreg(const char *name, uint32_t reg)
{
	//There seems to be surprisingly little difference between
	//different types of clocks here...
	printf("Register clock %s:, register: %u\n", name, reg);
}

static int
jh7110_clkgen_attach(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	cell_t reg;
	int i, rid, error;
	pcell_t *sysprop;
	pcell_t pll_offsets[8];
	struct syscon *sysregs = NULL;
	phandle_t node;
	clk_t osc, cpu_core, cpu_root, pll0_out;
	uint64_t cpu_core_freq;
	uint64_t rstart;

	sc = device_get_softc(dev);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	node = ofw_bus_get_node(dev);

	/* Allocate memory groups */
	
	error = bus_alloc_resources(dev, res_spec, &sc->sys_mem_res);
	if (error) {
		device_printf(dev, "Couldn't allocate sys group resources\n");
		return (ENXIO);
	}

	error = ofw_bus_find_string_index(node, "reg-names", "stg", &rid);
	if (error != 0) {
		device_printf(dev, "Cannot get 'stg' memory\n");
		return (ENXIO);
	}
	sc->stg_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->stg_mem_res == NULL) {
		device_printf(dev, "Cannot allocate 'stg' (rid: %d)\n",
		    rid);
		return (ENXIO);
	}
	error =	ofw_bus_find_string_index(node, "reg-names", "aon", &rid);
	if (error != 0) {
		device_printf(dev, "Cannot get 'aon' memory\n");
		return (ENXIO);
	}
	sc->aon_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->aon_mem_res == NULL) {
		device_printf(dev, "Cannot allocate 'aon' (rid: %d)\n",
		    rid);
		return (ENXIO);
	}
	
	/* Sanity check */
	error = OF_searchencprop(node, "#clock-cells", &reg, sizeof(reg));
	if (error == -1) {
		device_printf(dev, "Failed to get #clock-cells\n");
		return (ENXIO);
	}
	if (reg != 1) {
		device_printf(dev, "clock cells(%d) != 1\n", reg);
		return (ENXIO);
	}

	/* Create clock domain */
	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL) {
		device_printf(dev, "Failed to create clkdom\n");
		return (ENXIO);
	}
	clkdom_set_ofw_mapper(sc->clkdom, jh7110_ofw_map);

	/*  ISP and VOUT can be added later. Not exactly sure if they will
	    be allocated here or does it happen on their init functions
	
	error =	ofw_bus_find_string_index(node, "reg-names", "isp", &rid);
	if (error != 0) {
		device_printf(dev, "Cannot get 'isp' memory\n");
		return (ENXIO);
	}
	sc->isp_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->isp_mem_res == NULL) {
		device_printf(dev, "Cannot allocate 'isp' (rid: %d)\n",
		    rid);
		return (ENXIO);
	}
	error =	ofw_bus_find_string_index(node, "reg-names", "vout", &rid);
	if (error != 0) {
		device_printf(dev, "Cannot get 'vout' memory\n");
		return (ENXIO);
	}
	sc->vout_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->vout_mem_res == NULL) {
		device_printf(dev, "Cannot allocate 'vout' (rid: %d)\n",
		    rid);
		return (ENXIO);
	}
	*/

	/* Temporary solution for syscon registers */
	sc->syscon_mem_res = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid, 0x13030000,
	    0x1303FFFF, 0x1000, RF_ACTIVE );

	rstart = rman_get_start(sc->syscon_mem_res);
	printf("%s, rman_get_start returns %lu\n", __func__, rstart);
	rstart = rman_get_end(sc->syscon_mem_res);
	printf("%s, rman_get_end returns %lu\n", __func__, rstart);
	
	/* Get syscon property for pll_out clocks */
	error = OF_getencprop_alloc_multi(node, "starfive,sys-syscon",
					  sizeof(pcell_t), (void**)&sysprop);
	
	/* Element 0 is a phandle, 1-8 are offset values */
	for (int i = 0; i != 8; i++) {
		pll_offsets[i] = sysprop[i+1];
		printf("i, %d, pll_offsets[i]: %d, ", i, pll_offsets[i]); //debug
	}
	printf("\n");

	/* Registering clocks */

	for (i = 0; i < nitems(pll_out_clks); i++) {
		pll_out_clks[i].pll_offsets = pll_offsets;
		pll_out_clks[i].sysregs = sysregs;
		pll_out_clks[i].syscon_mem_res = sc->syscon_mem_res;
		pll_out_clks[i].mtx = &sc->mtx;
 	        error = jh7110_clk_pll_register(sc->clkdom, &pll_out_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    pll_out_clks[i].clkdef.name, error);
		else
			printf("%s clk registered\n", pll_out_clks[i].clkdef.name);
	}

	for (i = 0; i < nitems(sys_clks); i++) {
	        error = jh7110_clk_register(sc->clkdom, &sys_clks[i], JH7110_CLK_SYS);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
			    sys_clks[i].clkdef.name, error);
		else
			jh7110_print_clkreg(sys_clks[i].clkdef.name,
					    RD4(sc, sys_clks[i].offset));
	}

	for (i = 0; i < nitems(sys_pll_clks); i++) {

		error = clknode_fixed_register(sc->clkdom, &sys_pll_clks[i]);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
				      sys_pll_clks[i].clkdef.name, error);
		else
			printf("%s reg\t%x\n", sys_pll_clks[i].clkdef.name,
			 RD4(sc, sys_pll_clks[i].clkdef.id * sizeof(uint32_t)));
	}

	for (int i = 0; i < nitems(aon_clks); i++) {
		error = jh7110_clk_register(sc->clkdom, &aon_clks[i],
					    JH7110_CLK_AON);
		if (error != 0)
			device_printf(dev, "Failed to register clock %s: %d\n",
				      aon_clks[i].clkdef.name, error);
		else
			printf("jh7110_clkgen_attach(), Registered %s\n",
			       aon_clks[i].clkdef.name);
	}
	
	error = clkdom_finit(sc->clkdom);
	if (error) {
		device_printf(dev, "Clk domain finit fails %x.\n", error);
	}

	/* Adjusting clocks */
	
	error = clk_get_by_name(dev, "osc", &osc);
	if (error != 0)
		device_printf(dev, "Failed to get osc by name\n");

	error = clk_get_by_id(dev, sc->clkdom, JH7110_CPU_CORE, &cpu_core);
	if (error != 0)
		device_printf(dev, "Failed to get cpu_core\n");
	
	error = clk_get_by_id(dev, sc->clkdom, JH7110_CPU_ROOT, &cpu_root);
	if (error != 0)
		device_printf(dev, "Failed to get cpu_root\n");

	error = clk_get_by_id(dev, sc->clkdom, JH7110_PLL0_OUT, &pll0_out);
	if (error != 0)
		device_printf(dev, "Failed to get pll0_out\n");

        error = clk_get_freq(pll0_out, &cpu_core_freq);
	if (error != 0)
		device_printf(dev, "Failed to set cpu_root freq\n");
	
        error = clk_set_freq(cpu_core, cpu_core_freq/2, 0);
	if (error != 0)
		device_printf(dev, "Failed to set cpu_core freq\n");

	error = clk_set_parent_by_clk(cpu_root, osc);
	if (error != 0)
		device_printf(dev, "Failed to set cpu_root parent (2)\n");
	
	error = clk_set_freq(pll0_out, PLL0_DEFAULT_FREQ, 0);
	if (error != 0)
		device_printf(dev, "Failed to set freq for pll0_out\n");

	error = clk_set_parent_by_clk(cpu_root, pll0_out);
	if (error != 0)
		device_printf(dev, "Failed to set pll0_out parent\n");
	
	if (bootverbose)
		printf("%s, clockdom_dump()\n", __func__);
	clkdom_dump(sc->clkdom);

	return (0);
}

static int
jh7110_clkgen_detach(device_t dev)
{
	printf("%s\n", __func__);
	return (EBUSY);
}


static int
jh7110_clkgen_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
	struct jh7110_clkgen_softc *sc;
	printf("%s\n", __func__);
	
	sc = device_get_softc(dev);

	*val = RD4(sc, addr);
	return (0);
}

static int
jh7110_clkgen_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
	struct jh7110_clkgen_softc *sc;

	printf("%s\n", __func__);
	sc = device_get_softc(dev);
	WR4(sc, addr, val);
	return (0);
}

static int
jh7110_clkgen_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
	struct jh7110_clkgen_softc *sc;
	uint32_t reg;

	printf("%s\n", __func__);
	sc = device_get_softc(dev);

	reg = RD4(sc, addr);
	reg &= ~clr;
	reg |= set;
	WR4(sc, addr, reg);

	return (0);
}


static void
jh7110_clkgen_device_lock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	sc = device_get_softc(dev);
	mtx_lock(&sc->mtx);
}

static void
jh7110_clkgen_device_unlock(device_t dev)
{
	struct jh7110_clkgen_softc *sc;
	sc = device_get_softc(dev);
	mtx_unlock(&sc->mtx);
}

static device_method_t jh7110_clkgen_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		jh7110_clkgen_probe),
        DEVMETHOD(device_attach,	jh7110_clkgen_attach),
	DEVMETHOD(device_detach,	jh7110_clkgen_detach),

	/* clkdev interface */
	DEVMETHOD(clkdev_read_4,	jh7110_clkgen_read_4),
	DEVMETHOD(clkdev_write_4,	jh7110_clkgen_write_4),
	DEVMETHOD(clkdev_modify_4,	jh7110_clkgen_modify_4),
	DEVMETHOD(clkdev_device_lock,	jh7110_clkgen_device_lock),
	DEVMETHOD(clkdev_device_unlock,	jh7110_clkgen_device_unlock),

	DEVMETHOD_END
};

DEFINE_CLASS_0(jh7110_clkgen, jh7110_clkgen_driver, jh7110_clkgen_methods,
    sizeof(struct jh7110_clkgen_softc));

EARLY_DRIVER_MODULE(jh7110_clkgen, simplebus, jh7110_clkgen_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);

/* TODO: MODULE_DEPEND? */
MODULE_VERSION(jh7110_clkgen, 1);
