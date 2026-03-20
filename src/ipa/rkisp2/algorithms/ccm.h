/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Color Correction Matrix control algorithm
 */

#pragma once

#include <linux/media/rockchip/rkisp2-config.h>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

#include "libipa/ccm.h"
#include "libipa/fixedpoint.h"

#include "algorithm.h"
#include "ipa_context.h"
#include "params.h"

namespace libcamera {

namespace ipa::rkisp2::algorithms {

class Ccm : public Algorithm
{
public:
	Ccm() {}
	~Ccm() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
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
	void parseYaml(const ValueNode &tuningData);
	void setParameters(RkISP2Params *params, IPAFrameContext &context);

	CcmAlgorithm<Q<4, 7>> ccmAlgo_;
};

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
