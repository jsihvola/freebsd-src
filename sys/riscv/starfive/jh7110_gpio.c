/*-
 * SPDX-License-Identifier: BSD-2-Clause
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
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/gpio.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/intr.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/clk/clk.h>

#include "gpio_if.h"

#define JH7110_GPIO_PINS        64

#define GPIOEN                  0xdc
#define GPIOE_0                 0x100
#define GPIOE_1                 0x104
#define GPIO_DIN_LOW	        0x118
#define GPIO_DIN_HIGH	        0x11c
#define GP0_DOUT_CFG	        0x0
#define GP0_DOEN_CFG            0x40

#define ENABLE_MASK             0x3f
#define DATA_OUT_MASK           0x7f

#define ALIGN_4(_val) (_val & ~3)
#define MOD_4(_val) (_val & 3)


struct jh7110_gpio_softc {
	device_t              dev;
	device_t              busdev;
	struct mtx            mtx;
	struct resource       *res;
	clk_t                 clk;
};

static struct ofw_compat_data compat_data[] = {
	{"starfive,jh7110-sys-pinctrl", 1},
	{NULL,                          0}
};

static struct resource_spec jh7110_gpio_spec[] = {
	{ SYS_RES_MEMORY,     0,      RF_ACTIVE },
	{ -1, 0 }
};

#define JH7110_GPIO_LOCK(_sc)               mtx_lock(&(_sc)->mtx)
#define JH7110_GPIO_UNLOCK(_sc)             mtx_unlock(&(_sc)->mtx)

#define JH7110_GPIO_READ(sc, reg)           bus_read_4((sc)->res, (reg))
#define JH7110_GPIO_WRITE(sc, reg, val)     bus_write_4((sc)->res, (reg), (val))

static device_t
jh7110_gpio_get_bus(device_t dev)
{
	struct jh7110_gpio_softc *sc;
	//breakpoint();
	sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
jh7110_gpio_pin_max(device_t dev, int *maxpin)
{
	*maxpin = JH7110_GPIO_PINS - 1;

	return (0);
}

static int
jh7110_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;
	//breakpoint();
	sc = device_get_softc(dev);

	if (pin >= JH7110_GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	if (pin < 32) {
		reg = JH7110_GPIO_READ(sc, GPIO_DIN_LOW);
		*val = (reg >> pin) & 0x1;
	}
	else {
		reg = JH7110_GPIO_READ(sc, GPIO_DIN_HIGH);
		*val = (reg >> (pin - 32)) & 0x1;
	}
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;
	//breakpoint();

	printf("%s, dev, %s, pin: %u, value: %u\n", __func__, device_get_name(dev), pin, value);
	
	sc = device_get_softc(dev);

	if (pin >= JH7110_GPIO_PINS)
		return (EINVAL);

	JH7110_GPIO_LOCK(sc);
	reg = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + ALIGN_4(pin));
	reg &= ~(DATA_OUT_MASK << MOD_4(pin) * 8);
	reg |= reg << MOD_4(pin) * 8;
	JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + ALIGN_4(pin), value);
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;
	//breakpoint();
	sc = device_get_softc(dev);

	if (pin >= JH7110_GPIO_PINS)
		return (EINVAL);

	/* Reading the direction */
	JH7110_GPIO_LOCK(sc);
	reg = JH7110_GPIO_READ(sc, GP0_DOEN_CFG + ALIGN_4(pin));
	if ((reg & ENABLE_MASK << MOD_4(pin) * 8) == 0)
		*flags |= GPIO_PIN_OUTPUT;
	else
		*flags |= GPIO_PIN_INPUT;
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}


static int
jh7110_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct jh7110_gpio_softc *sc;
	uint32_t reg;
	//breakpoint();
	sc = device_get_softc(dev);

	if (pin >= JH7110_GPIO_PINS)
		return (EINVAL);

	/* Setting the direction, enable or disable output */
	JH7110_GPIO_LOCK(sc);
	if (flags & GPIO_PIN_INPUT) {
		reg = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + ALIGN_4(pin));
		reg &= ~(ENABLE_MASK << MOD_4(pin) * 8);
		reg |= 1 << MOD_4(pin) * 8;
		JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + ALIGN_4(pin), reg);
	} else if (flags & GPIO_PIN_OUTPUT) {
		reg = JH7110_GPIO_READ(sc, GP0_DOUT_CFG + ALIGN_4(pin));
		reg &= ~(ENABLE_MASK << MOD_4(pin) * 8);
		reg |= 0 << MOD_4(pin) * 8;
		JH7110_GPIO_WRITE(sc, GP0_DOUT_CFG + ALIGN_4(pin), reg);
	}
	JH7110_GPIO_UNLOCK(sc);

	return (0);
}

static int
jh7110_gpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "StarFive JH7110 GPIO generator driver");

	return (BUS_PROBE_DEFAULT);
}

static int
jh7110_gpio_detach(device_t dev)
{
	struct jh7110_gpio_softc *sc;

	sc = device_get_softc(dev);

	bus_release_resources(dev, jh7110_gpio_spec, &sc->res);
	gpiobus_detach_bus(dev);
	mtx_destroy(&sc->mtx);

	return (0);
}

static int
jh7110_gpio_attach(device_t dev)
{
	struct jh7110_gpio_softc *sc;
	int err;

	sc = device_get_softc(dev);
	sc->dev = dev;

	mtx_init(&sc->mtx, device_get_nameunit(sc->dev), NULL, MTX_DEF);

	if (bus_alloc_resources(dev, jh7110_gpio_spec, &sc->res)) {
		device_printf(dev, "Could not allocate resources\n");
		bus_release_resources(dev, jh7110_gpio_spec, &sc->res);
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	if (clk_get_by_ofw_index(dev, 0, 0, &sc->clk) != 0) {
		device_printf(dev, "Cannot get clock\n");
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	err = clk_enable(sc->clk);
	if (err != 0) {
		device_printf(dev, "Could not enable clock %s\n",
			      clk_get_name(sc->clk));
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	sc->busdev = gpiobus_attach_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "Cannot attach gpiobus\n");
		jh7110_gpio_detach(dev);
		return (ENXIO);
	}

	/* Reseting GPIO interrupts */
	JH7110_GPIO_WRITE(sc, GPIOE_0, 0);
	JH7110_GPIO_WRITE(sc, GPIOE_1, 0);
	JH7110_GPIO_WRITE(sc, GPIOEN, 1);

	return (0);
}

static phandle_t
jh7110_gpio_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

static device_method_t jh7110_gpio_methods[] = {
        /* Device interface */
	DEVMETHOD(device_probe,         jh7110_gpio_probe),
	DEVMETHOD(device_attach,        jh7110_gpio_attach),
	DEVMETHOD(device_detach,        jh7110_gpio_detach),

        /* GPIO protocol */
	DEVMETHOD(gpio_get_bus,         jh7110_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,         jh7110_gpio_pin_max),
	DEVMETHOD(gpio_pin_getflags,    jh7110_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,    jh7110_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,         jh7110_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,         jh7110_gpio_pin_set),

        /* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,     jh7110_gpio_get_node),

	DEVMETHOD_END
};

DEFINE_CLASS_0(gpio, jh7110_gpio_driver, jh7110_gpio_methods,
    sizeof(struct jh7110_gpio_softc));
EARLY_DRIVER_MODULE(jh7110_gpio, simplebus, jh7110_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
MODULE_DEPEND(jh7110_gpio, gpiobus, 1, 1, 1);
