/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas on Board Oy.
 *
 * RkISP2 IPA Module
 */

#pragma once

#include <linux/media/rockchip/rkisp2-config.h>

#include <libcamera/ipa/rkisp2_ipa_interface.h>

#include <libipa/module.h>

#include "ipa_context.h"
#include "params.h"
#include "stats.h"

namespace libcamera {

namespace ipa::rkisp2 {

using Module = ipa::Module<IPAContext, IPAFrameContext, IPACameraSensorInfo,
			   RkISP2Params, RkISP2Stats>;

} /* namespace ipa::rkisp2 */

} /* namespace libcamera*/
