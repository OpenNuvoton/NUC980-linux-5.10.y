// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2024 Nuvoton Corporation
 *
 * Author: twjiang@nuvoton.com
 */

#include <linux/slab.h>
#include "internals.h"

static void winbond_fixup_onfi_param_page(struct nand_chip *chip,
					 struct nand_onfi_params *p)
{
	if  (strncmp(p->model, "W29N02KV", 8) == 0) {
		/* W29N02KVxxAF reports wrong oobsize */
		p->spare_bytes_per_page = cpu_to_le16(64);
	}
}

const struct nand_manufacturer_ops winbond_nand_manuf_ops = {
	.fixup_onfi_param_page = winbond_fixup_onfi_param_page,
};
