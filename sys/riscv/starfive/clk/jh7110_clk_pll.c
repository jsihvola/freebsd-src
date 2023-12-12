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
 *
 * $FreeBSD$
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/fbio.h>
#include <sys/rman.h>
#include <sys/module.h>
#include <sys/resource.h>
#include <machine/fpe.h>
#include <machine/bus.h>

#include <riscv/starfive/clk/jh7110_clk_pll.h>
#include <riscv/starfive/clk/jh7110_clk.h>
#include <dev/extres/clk/clk.h>

#include "clkdev_if.h"

#define	SYSCON_READ_4(reg)		   bus_read_4(sc->syscon_mem_res, (reg))
#define	SYSCON_MODIFY_4(reg, cbits, sbits)	                               \
        jh7110_syscon_modify_4(sc->syscon_mem_res, (reg), (cbits), (sbits))

struct jh7110_clk_pll_sc {
	struct mtx		   *mtx;
	struct syscon              *sysregs;
      	struct resource            *syscon_mem_res;
	const struct
	starfive_pll_syscon_value  *syscon_arr;
	int                        syscon_arr_nitems;

  	uint32_t	           flags;
  
	uint32_t                   dacpd_offset;
	uint32_t                   dsmpd_offset;
	uint32_t                   fbdiv_offset;
	uint32_t                   frac_offset;
	uint32_t                   prediv_offset;
	uint32_t                   postdiv1_offset;

	uint32_t                   dacpd_mask;
	uint32_t                   dsmpd_mask;
	uint32_t                   fbdiv_mask;
	uint32_t                   frac_mask;
	uint32_t                   prediv_mask;
	uint32_t                   postdiv1_mask;

	uint32_t                   dacpd_shift;
	uint32_t                   dsmpd_shift;
	uint32_t                   fbdiv_shift;
	uint32_t                   frac_shift;
	uint32_t                   prediv_shift;
	uint32_t                   postdiv1_shift;
};

/* Since syscon pseudo device does not currently support more than
   one syscon memory region, we use this temporary function instead */

static int
jh7110_syscon_modify_4(struct resource *res, bus_size_t offset,
                                         uint32_t clear_bits, uint32_t set_bits)
{
	uint32_t val;
	uint32_t new_val;

	val = bus_read_4(res, offset);

	new_val = val & ~clear_bits;
	new_val = set_bits | new_val;

	if(new_val != val)
		bus_write_4(res, offset, new_val);

	return (0);
}

static int
jh7110_clk_pll_init(struct clknode *clk, device_t dev)
{
	clknode_init_parent_idx(clk, 0);

	return (0);
}

static int
jh7110_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct jh7110_clk_pll_sc *sc;

	uint32_t dacpd, dsmpd, fbdiv, prediv, postdiv1;
	uint64_t frac;

	sc = clknode_get_softc(clk);

	mtx_lock(sc->mtx);
	dacpd = (SYSCON_READ_4(sc->dacpd_offset)
		          & sc->dacpd_mask) >> sc->dacpd_shift;
	dsmpd = (SYSCON_READ_4(sc->dsmpd_offset)
		          & sc->dsmpd_mask) >> sc->dsmpd_shift;
	fbdiv = (SYSCON_READ_4(sc->fbdiv_offset)
		          & sc->fbdiv_mask) >> sc->fbdiv_shift;
	prediv = (SYSCON_READ_4(sc->prediv_offset)
		          & sc->prediv_mask) >> sc->prediv_shift;
	postdiv1 = (SYSCON_READ_4(sc->postdiv1_offset)
		          & sc->postdiv1_mask) >> sc->postdiv1_shift;
	frac = (SYSCON_READ_4(sc->frac_offset)
		          & sc->frac_mask) >> sc->frac_shift;
	mtx_unlock(sc->mtx);
	
	/* dacpd and dsmpd both being 0 entails Fraction Multiple Mode */
	if ((dacpd == 0) && (dsmpd == 0)) {
		*freq = *freq / FRAC_PATR_SIZE * (fbdiv * FRAC_PATR_SIZE +
   	        (frac * FRAC_PATR_SIZE / (1 << 24))) / prediv / (1 << postdiv1);
	}		
	else {
		*freq = *freq / FRAC_PATR_SIZE * (fbdiv * FRAC_PATR_SIZE)
                                   		     / prediv / (1 << postdiv1);
	}
		
	return (0);
}

static int
jh7110_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{

	struct jh7110_clk_pll_sc *sc;
	const struct starfive_pll_syscon_value *syscon_val = NULL;

	sc = clknode_get_softc(clk);

	for (int i = 0; i != sc->syscon_arr_nitems; i++) {
		if (*fout == sc->syscon_arr[i].freq) {
			syscon_val = &sc->syscon_arr[i];
		}
	}

	if (syscon_val == NULL) {
		printf("%s: tried to set an unknown frequency %ju for %s\n",
		       __func__, *fout, clknode_get_name(clk));
		return (EINVAL);
	}

	if (flags & CLK_SET_DRYRUN) {
		*done = 1;
		return (0);
	}

	mtx_lock(sc->mtx);
	SYSCON_MODIFY_4(sc->dacpd_offset, sc->dacpd_mask,
		syscon_val->dacpd << sc->dacpd_shift & sc->dacpd_mask);
	SYSCON_MODIFY_4(sc->dsmpd_offset, sc->dsmpd_mask,
		syscon_val->dsmpd << sc->dsmpd_shift & sc->dsmpd_mask);
	SYSCON_MODIFY_4(sc->prediv_offset, sc->prediv_mask,
		syscon_val->prediv << sc->prediv_shift & sc->prediv_mask);
	SYSCON_MODIFY_4(sc->fbdiv_offset, sc->fbdiv_mask,
		syscon_val->fbdiv << sc->fbdiv_shift & sc->fbdiv_mask);
	SYSCON_MODIFY_4(sc->postdiv1_offset,
			sc->postdiv1_mask, (syscon_val->postdiv1 >> 1)
			<< sc->postdiv1_shift & sc->postdiv1_mask);

	if (!syscon_val->dacpd && !syscon_val->dsmpd) {
		SYSCON_MODIFY_4(sc->frac_offset, sc->frac_mask,
		            syscon_val->frac << sc->frac_shift & sc->frac_mask);
	}

	mtx_unlock(sc->mtx);
	*done = 1;

	return (0);
}

static clknode_method_t jh7110_clk_pll_clknode_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,		jh7110_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	jh7110_clk_pll_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		jh7110_clk_pll_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(jh7110_clk_pll_clknode, jh7110_clk_pll_clknode_class,
                  jh7110_clk_pll_clknode_methods,
                      sizeof(struct jh7110_clk_pll_sc), clknode_class);

int
jh7110_clk_pll_register(struct clkdom *clkdom, struct jh7110_pll_def *clkdef)
{
	struct clknode *clk;
	struct jh7110_clk_pll_sc *sc;

	clk = clknode_create(clkdom, &jh7110_clk_pll_clknode_class,
	       &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->sysregs = clkdef->sysregs;
	sc->syscon_mem_res = clkdef->syscon_mem_res;
	sc->mtx = clkdef->mtx;

	if(!strncmp(clknode_get_name(clk), "pll0_out", 8)) {
		sc->syscon_arr = jh7110_pll0_syscon_freq;
		sc->syscon_arr_nitems = nitems(jh7110_pll0_syscon_freq);

		sc->dacpd_offset = clkdef->pll_offsets[0];
		sc->dsmpd_offset = clkdef->pll_offsets[0];
		sc->fbdiv_offset = clkdef->pll_offsets[1];
		sc->frac_offset = clkdef->pll_offsets[2];
		sc->prediv_offset = clkdef->pll_offsets[3];
		sc->postdiv1_offset = clkdef->pll_offsets[2];

		sc->dacpd_mask = PLL0_DACPD_MASK;
		sc->dsmpd_mask = PLL0_DSMPD_MASK;
		sc->fbdiv_mask = PLL0_FBDIV_MASK;
		sc->frac_mask = PLL0_FRAC_MASK;
		sc->prediv_mask = PLL0_PREDIV_MASK;
		sc->postdiv1_mask = PLL0_POSTDIV1_MASK;

		sc->dacpd_shift = PLL0_DACPD_SHIFT;
		sc->dsmpd_shift = PLL0_DSMPD_SHIFT;
		sc->fbdiv_shift = PLL0_FBDIV_SHIFT;
		sc->frac_shift = PLL0_FRAC_SHIFT;
		sc->prediv_shift = PLL0_PREDIV_SHIFT;
		sc->postdiv1_shift = PLL0_POSTDIV1_SHIFT;
	}

	else if (!strncmp(clknode_get_name(clk), "pll1_out", 8)) {
		sc->syscon_arr = jh7110_pll1_syscon_freq;
		sc->syscon_arr_nitems = nitems(jh7110_pll1_syscon_freq);

		sc->dacpd_offset = clkdef->pll_offsets[3];
		sc->dsmpd_offset = clkdef->pll_offsets[3];
		sc->fbdiv_offset = clkdef->pll_offsets[3];
		sc->frac_offset = clkdef->pll_offsets[4];
		sc->prediv_offset = clkdef->pll_offsets[5];
		sc->postdiv1_offset = clkdef->pll_offsets[4];

		sc->dacpd_mask = PLL1_DACPD_MASK;
		sc->dsmpd_mask = PLL1_DSMPD_MASK;
		sc->fbdiv_mask = PLL1_FBDIV_MASK;
		sc->frac_mask = PLL1_FRAC_MASK;
		sc->prediv_mask = PLL1_PREDIV_MASK;
		sc->postdiv1_mask = PLL1_POSTDIV1_MASK;

		sc->dacpd_shift = PLL1_DACPD_SHIFT;
		sc->dsmpd_shift = PLL1_DSMPD_SHIFT;
		sc->fbdiv_shift = PLL1_FBDIV_SHIFT;
		sc->frac_shift = PLL1_FRAC_SHIFT;
		sc->prediv_shift = PLL1_PREDIV_SHIFT;
		sc->postdiv1_shift = PLL1_POSTDIV1_SHIFT;
	}

	else if (!strncmp(clknode_get_name(clk), "pll2_out", 8)) {
		sc->syscon_arr = jh7110_pll2_syscon_freq;
		sc->syscon_arr_nitems = nitems(jh7110_pll2_syscon_freq);

		sc->dacpd_offset = clkdef->pll_offsets[5];
		sc->dsmpd_offset = clkdef->pll_offsets[5];
		sc->fbdiv_offset = clkdef->pll_offsets[5];
		sc->frac_offset = clkdef->pll_offsets[6];
		sc->prediv_offset = clkdef->pll_offsets[7];
		sc->postdiv1_offset = clkdef->pll_offsets[6];

		sc->dacpd_mask = PLL2_DACPD_MASK;
		sc->dsmpd_mask = PLL2_DSMPD_MASK;
		sc->fbdiv_mask = PLL2_FBDIV_MASK;
		sc->frac_mask = PLL2_FRAC_MASK;
		sc->prediv_mask = PLL2_PREDIV_MASK;
		sc->postdiv1_mask = PLL2_POSTDIV1_MASK;

		sc->dacpd_shift = PLL2_DACPD_SHIFT;
		sc->dsmpd_shift = PLL2_DSMPD_SHIFT;
		sc->fbdiv_shift = PLL2_FBDIV_SHIFT;
		sc->frac_shift = PLL2_FRAC_SHIFT;
		sc->prediv_shift = PLL2_PREDIV_SHIFT;
		sc->postdiv1_shift = PLL2_POSTDIV1_SHIFT;
	}

	clknode_register(clkdom, clk);

	return (0);
}
