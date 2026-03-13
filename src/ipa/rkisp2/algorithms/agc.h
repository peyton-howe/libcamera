/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board Oy.
 *
 * RkISP2 AGC/AEC mean-based control algorithm
 */

#pragma once

#include <vector>

#include <linux/media/rockchip/rkisp2-config.h>

#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include "libipa/agc_mean_luminance.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp2::algorithms {

class Agc : public Algorithm, public AgcMeanLuminance
{
public:
	Agc() = default;
	~Agc() = default;

	int init(IPAContext &context, [[maybe_unused]] const ValueNode &tuningData) override;
	int configure(IPAContext &context, const IPACameraSensorInfo &configInfo) override;
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
	void fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
			  ControlList &metadata, const RkISP2Stats *stats);
	double estimateLuminance(double gain) const override;
	void processFrameDuration(IPAContext &context,
				  IPAFrameContext &frameContext,
				  utils::Duration frameDuration);

	std::vector<uint16_t> expMeans_;
	Span<const uint8_t> weights_;
};

} /* namespace ipa::rkisp2::algorithms */
} /* namespace libcamera */
