/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Color Correction Matrix control algorithm
 */

#include "ccm.h"

#include <map>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/interpolator.h"

/**
 * \file ccm.h
 * \brief RkISP2 CCM algorithm implementation
 */

namespace libcamera {

namespace ipa::rkisp2::algorithms {

/**
 * \class Ccm
 * \brief RkISP2 color correction matrix algorithm
 */

LOG_DEFINE_CATEGORY(RkISP2Ccm)

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int Ccm::init([[maybe_unused]] IPAContext &context, const ValueNode &tuningData)
{
	return ccmAlgo_.init(tuningData, context.ctrlMap);
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int Ccm::configure(IPAContext &context,
		   [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	return ccmAlgo_.configure(context.activeState.ccm,
				  context.activeState.awb.automatic.colourTemperature);
}

void Ccm::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	ccmAlgo_.queueRequest(context.activeState.ccm, frameContext.ccm, controls);
}

void Ccm::setParameters(RkISP2Params *params, IPAFrameContext &context)
{
	const Matrix<float, 3, 3> &matrix = context.ccm.ccm;
	const Matrix<int16_t, 3, 1> &offsets = context.ccm.offsets;

	auto config = params->block<RkISP2ParamsBlocks::Ccm>();
	config.setEnabled(true);

	/*
	 * 4 bit integer and 7 bit fractional, ranging from -8 (0x400) to
	 * +7.9921875 (0x3ff)
	 */
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
			config->coeff[i][j] = Q<4, 7>(matrix[i][j]).quantized();
	}

	for (unsigned int i = 0; i < 3; i++)
		config->offset[i] = offsets[i][0] & 0xfff;

	config->y_coeff[0] = 0x26;
	config->y_coeff[1] = 0x4b;
	config->y_coeff[2] = 0x0f;
	config->inflection_point = 0xa;

	for (unsigned int i = 0; i < 17; i++)
		config->alp[i] = 0x400;

	/* \todo Support alp */

	LOG(RkISP2Ccm, Debug) << "Setting matrix " << matrix;
	LOG(RkISP2Ccm, Debug) << "Setting offsets " << offsets;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Ccm::prepare(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, RkISP2Params *params)
{
	if (frameContext.awb.autoEnabled)
		ccmAlgo_.prepare(context.activeState.ccm, frameContext.ccm,
				 frame, frameContext.awb.colourTemperature);

	setParameters(params, frameContext);
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Ccm::process([[maybe_unused]] IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  [[maybe_unused]] const RkISP2Stats *stats,
		  ControlList &metadata)
{
	ccmAlgo_.process(frameContext.ccm, metadata);
}

REGISTER_IPA_ALGORITHM(Ccm, "Ccm")

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
