/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 control algorithm interface
 */

#pragma once

#include <linux/media/rockchip/rkisp2-config.h>

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::rkisp2 {

class Algorithm : public libcamera::ipa::Algorithm<Module>
{
};

} /* namespace ipa::rkisp2 */

} /* namespace libcamera */

