/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright 2022 Mitchell Horne <mhorne@FreeBSD.org>
 * Copyright 2023 Jari Sihvola <jsihv@gmx.com>
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

#include <dev/extres/clk/clk.h>

#ifndef _CLK_JH7110_CLK_H_
#define	_CLK_JH7110_CLK_H_

#define	JH7110_CLK_HAS_GATE	0x01
#define	JH7110_CLK_HAS_MUX	0x02
#define	JH7110_CLK_HAS_DIV	0x04
#define	JH7110_CLK_HAS_FRAC	0x08

#define JH7110_CLK_AON          0x10 
#define JH7110_CLK_ISP          0x20 
#define JH7110_CLK_STG          0x40 
#define JH7110_CLK_SYS          0x80 
#define JH7110_CLK_VOUT         0x100

#define PLL0_DEFAULT_FREQ	1500000000


struct jh7110_clk_def {
	struct clknode_init_def	clkdef;
	uint32_t		offset;
	uint32_t		flags;
  	uint64_t		d_max;
};

struct jh7110_pll_def {
	struct clknode_init_def	   clkdef;
    	struct mtx		   *mtx;
  	struct syscon              *sysregs;
  	struct resource            *syscon_mem_res;  
  	uint32_t                   *pll_offsets;

};

#define	JH7110_PLL(_idx, _name, _pn)			        \
{								\
	.clkdef.id =	_idx,					\
	.clkdef.name =	_name,					\
	.clkdef.parent_names = _pn,				\
	.clkdef.parent_cnt = nitems(_pn),			\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,		\
	.fixed_flags = 0,					\
	.mult = 1,                                              \
        .div = 1,                                               \
}

#define	JH7110_CLK(_idx, _name, _pn, _d_max, _flags)	        \
{								\
	.clkdef.id =	_idx,					\
	.clkdef.name =	_name,					\
	.clkdef.parent_names = _pn,				\
	.clkdef.parent_cnt = nitems(_pn),			\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,		\
	.flags = _flags,					\
        .d_max = _d_max,                                        \
}

#define	JH7110_GATE(_idx, _name, _pn)		                                \
    JH7110_CLK(_idx, _name, _pn, 0, JH7110_CLK_HAS_GATE)
#define	JH7110_MUX(_idx, _name, _pn)			                        \
    JH7110_CLK(_idx, _name, _pn, 0, JH7110_CLK_HAS_MUX)
#define	JH7110_DIV(_idx, _name, _pn, _d_max)				        \
    JH7110_CLK(_idx, _name, _pn, _d_max, JH7110_CLK_HAS_DIV)
#define	JH7110_GATEMUX(_idx, _name, _pn)			                \
    JH7110_CLK(_idx, _name, _pn, 0, JH7110_CLK_HAS_GATE | JH7110_CLK_HAS_MUX)
#define	JH7110_GATEDIV(_idx, _name, _pn, _d_max)			        \
    JH7110_CLK(_idx, _name, _pn, _d_max, JH7110_CLK_HAS_GATE                    \
                                                        | JH7110_CLK_HAS_DIV)
/*
#define	JH7110_MUXDIV(_idx, _name, _pn, _d_max)		\
    JH7110_CLK(_idx, _name, _pn, JH7110_CLK_HAS_MUXDIV)
*/

struct resource * jh7110_clk_get_memres(struct clknode *clk, uint32_t flags);

int jh7110_clk_register(struct clkdom *clkdom,
			const struct jh7110_clk_def *clkdef, int mem_group);

int jh7110_clk_pll_register(struct clkdom *clkdom,
			    struct jh7110_pll_def *clkdef);

#endif	/* _CLK_JH7110_CLK_H_ */
