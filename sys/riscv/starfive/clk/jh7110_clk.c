/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Emmanuel Vadot <manu@freebsd.org>
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
#include <sys/rman.h>

#include <dev/extres/clk/clk.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dt-bindings/clock/starfive-jh7110-clkgen.h>

#include <riscv/starfive/clk/jh7110_clk.h>

#include "clkdev_if.h"

/*
 * Layout of the clock registers:
 * +-----------------------------+
 * | 31 | 30  | 29 - 24 | 23 - 0 |
 * | EN | INV | MUX     | DIV    |
 * +-----------------------------+
 */

#define	JH7110_DIV_MASK		0xffffff
#define	JH7110_MUX_SHIFT	24
#define	JH7110_MUX_MASK		0x3f000000
#define	JH7110_ENABLE_SHIFT	31
#define REG_SIZE                4

struct jh7110_clk_sc {
        uint32_t	           offset;
	uint32_t	           flags;
        uint64_t                   d_max;
	int                           id;
};

#define DIV_ROUND_CLOSEST(n,d)  (((n) + (d) / 2) / (d))

#define	READ4_MEMRES(_clk, off)			                      \
        bus_read_4(jh7110_clk_get_memres(_clk, sc->flags), off)		      
#define WRITE4_MEMRES(_clk, off, _val)		                      \
        bus_write_4(jh7110_clk_get_memres(_clk, sc->flags), off, _val)	      
#define	DEVICE_LOCK(_clk)					      \
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)					      \
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

static int
jh7110_clk_init(struct clknode *clk, device_t dev)
{
	struct jh7110_clk_sc *sc;
	uint32_t reg;
	int idx = 0;

	sc = clknode_get_softc(clk);
	
	printf("jh7110_clk_init(), clk->name: %s\n", clknode_get_name(clk));

	if (sc->flags & JH7110_CLK_HAS_MUX) {
	  DEVICE_LOCK(clk);
	  reg = READ4_MEMRES(clk, sc->offset);
	  DEVICE_UNLOCK(clk);
	  idx = (reg & JH7110_MUX_MASK) >> JH7110_MUX_SHIFT;
	}

	clknode_init_parent_idx(clk, idx);

	return (0);
}

static int
jh7110_clk_set_gate(struct clknode *clk, bool enable)
{
	struct jh7110_clk_sc *sc;
	uint32_t reg;

	printf("jh7110_clk_set_gate\n");

	sc = clknode_get_softc(clk);

	if ((sc->flags & JH7110_CLK_HAS_GATE) == 0) {
		printf("jh7110_clk_set_gate 2\n");
		return (0);
	}

	printf("jh7110_clk_set_gate(), id: %d, offset: %u\n", sc->id, sc->offset);

	DEVICE_LOCK(clk);
	reg = READ4_MEMRES(clk, sc->offset);
	if (enable)
		reg |= (1 << JH7110_ENABLE_SHIFT);
	else
		reg &= ~(1 << JH7110_ENABLE_SHIFT);
        WRITE4_MEMRES(clk, sc->offset, reg);
	DEVICE_UNLOCK(clk);

	return (0);
}

static int
jh7110_clk_set_mux(struct clknode *clk, int idx)
{
	struct jh7110_clk_sc *sc;
	uint32_t reg;

	printf("jh7110_clk_set_mux\n");
	
	sc = clknode_get_softc(clk);

	if ((sc->flags & JH7110_CLK_HAS_MUX) == 0) {
		printf("jh7110_clk_setmux 2\n");
		return (ENXIO);
	}

	/* Checking index size */
	if ((idx & (JH7110_MUX_MASK >> JH7110_MUX_SHIFT)) != idx) {
		printf("jh7110_clk_setmux 3\n");	
		return (EINVAL);
	}

	DEVICE_LOCK(clk);
	reg = READ4_MEMRES(clk, sc->offset) & ~JH7110_MUX_MASK;
	reg |= idx << JH7110_MUX_SHIFT;
        WRITE4_MEMRES(clk, sc->offset, reg);
	DEVICE_UNLOCK(clk);

	return (0);
}


static int
jh7110_clk_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct jh7110_clk_sc *sc;
	uint32_t divisor;

	printf("jh7110_clk_recalc_freq\n");
	sc = clknode_get_softc(clk);

	if ((sc->flags & JH7110_CLK_HAS_DIV) == 0) {
		return (0);
	}

	DEVICE_LOCK(clk);
	divisor = READ4_MEMRES(clk, sc->offset) & JH7110_DIV_MASK;
	DEVICE_UNLOCK(clk);

	if (sc->id >= JH7110_UART3_CLK_CORE &&
	    sc->id <= JH7110_UART5_CLK_CORE &&
	    sc->id %2 == 0)
		divisor >>= 8;

	printf("jh7110_clk_recalc_freq() 5, divisor: %u\n", divisor);
	if (divisor)
		*freq = *freq / divisor;
	else
		*freq = 0;

	return (0);
}

static int
jh7110_clk_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{

	struct jh7110_clk_sc *sc;
	uint64_t divisor;

	printf("jh7110_clk_set_freq\n");
	sc = clknode_get_softc(clk);

	if ((sc->flags & JH7110_CLK_HAS_DIV) == 0) {
		return (0);
	}
	
	divisor = MIN(MAX(DIV_ROUND_CLOSEST(fin, *fout), 1UL), sc->d_max);

	if (sc->id >= JH7110_UART3_CLK_CORE &&
	    sc->id <= JH7110_UART5_CLK_CORE &&
	    sc->id %2 == 0)
	  divisor <<= 8;

	if (flags & CLK_SET_DRYRUN) {
		*done = 1;
		*fout = divisor;
		return (0);
	}

	DEVICE_LOCK(clk);
	divisor |= READ4_MEMRES(clk, sc->offset) & ~JH7110_DIV_MASK;
	WRITE4_MEMRES(clk, sc->offset, divisor);
	DEVICE_UNLOCK(clk);

	*fout = divisor;
	*done = 1;
	
	return (0);
}


static clknode_method_t jh7110_clknode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		jh7110_clk_init),
	CLKNODEMETHOD(clknode_set_gate,		jh7110_clk_set_gate),
	CLKNODEMETHOD(clknode_set_mux,		jh7110_clk_set_mux),
	CLKNODEMETHOD(clknode_recalc_freq,	jh7110_clk_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,	        jh7110_clk_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(jh7110_clknode, jh7110_clknode_class, jh7110_clknode_methods,
    sizeof(struct jh7110_clk_sc), clknode_class);

int
jh7110_clk_register(struct clkdom *clkdom,
			const struct jh7110_clk_def *clkdef, int mem_group)
{
	struct clknode *clk;
	struct jh7110_clk_sc *sc;

	clk = clknode_create(clkdom, &jh7110_clknode_class, &clkdef->clkdef);
	if (clk == NULL) {
		return (-1);
	}

	sc = clknode_get_softc(clk);
	
	if(mem_group == JH7110_CLK_AON) {
	  sc->offset = (clkdef->clkdef.id - JH7110_CLK_STG_REG_END) * REG_SIZE;
	}
	else if(mem_group == JH7110_CLK_STG) {
	  sc->offset = (clkdef->clkdef.id - JH7110_CLK_SYS_REG_END) * REG_SIZE;
	}
	else {
	  sc->offset = clkdef->clkdef.id * REG_SIZE;
	}
	
	sc->flags = clkdef->flags | mem_group;
	sc->id = clkdef->clkdef.id;
	sc->d_max = clkdef->d_max;

	clknode_register(clkdom, clk);

	printf("jh7110_clk_register(), name: %s, flag: %d\n",
	                           clknode_get_name(clk), sc->flags);

	return (0);
}
