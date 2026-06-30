/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Black Level Subtraction control
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp2::algorithms {

class BlackLevelSubtraction : public Algorithm
{
public:
	BlackLevelSubtraction() = default;
	~BlackLevelSubtraction() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP2Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const RkISP2Stats *stats,
		     ControlList &metadata) override;

private:
	int16_t blackLevelRed_;
	int16_t blackLevelGreenR_;
	int16_t blackLevelGreenB_;
	int16_t blackLevelBlue_;
};

} /* namespace ipa::rkisp2::algorithms */
} /* namespace libcamera */
