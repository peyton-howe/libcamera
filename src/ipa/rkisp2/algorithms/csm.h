/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Color space conversion
 */

#pragma once

#include "libcamera/internal/matrix.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp2::algorithms {

class ColorSpaceConversion : public Algorithm
{
public:
	ColorSpaceConversion() = default;
	~ColorSpaceConversion() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP2Params *params) override;
};

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
