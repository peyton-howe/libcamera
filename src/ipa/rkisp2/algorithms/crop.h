/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Crop
 */

#pragma once

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp2::algorithms {

class Crop : public Algorithm
{
public:
	Crop() = default;
	~Crop() = default;

	void queueRequest(IPAContext &context,
			  const uint32_t frame,
			  IPAFrameContext &frameContext,
			  const ControlList &controls) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     RkISP2Params *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const RkISP2Stats *stats,
		     ControlList &metadata) override;

private:
	Rectangle defaultCrop_;
};

} /* namespace ipa::rkisp2::algorithms */
} /* namespace libcamera */
