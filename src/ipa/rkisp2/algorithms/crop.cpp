/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Crop
 */

#include "crop.h"

#include <cmath>

#include <linux/media/rockchip/rkisp2-config.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

/**
 * \file crop.h
 */

namespace libcamera {

namespace ipa::rkisp2::algorithms {

/**
 * \class Crop
 * \brief RkISP2 Crop
 *
 * This is a thin algorithm that implements ScalerCrop for the RkISP2.
 *
 * As the cropping of the RkISP2 is controlled by parameter buffers instead of
 * by V4L2 crop rectangles, it is more practical to implement the ScalerCrop
 * control here instead of in the pipeline handler. This also has the positive
 * side effect of supporting per-frame ScalerCrop.
 */

LOG_DEFINE_CATEGORY(RkISP2Crop)

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Crop::queueRequest(IPAContext &context,
			[[maybe_unused]] const uint32_t frame,
			IPAFrameContext &frameContext,
			const ControlList &controls)
{
	frameContext.crop.crop = context.activeState.crop.crop;

	const auto &scalerCrop = controls.get(controls::ScalerCrop);
	if (!scalerCrop)
		return;

	context.activeState.crop.crop = *scalerCrop;

	frameContext.crop.crop = context.activeState.crop.crop;
	frameContext.crop.set = true;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Crop::prepare([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const uint32_t frame,
		   IPAFrameContext &frameContext,
		   [[maybe_unused]] RkISP2Params *params)
{
	if (!frameContext.crop.set)
		return;

	auto config = params->block<RkISP2ParamsBlocks::Crop>();
	config.setEnabled(true);

	config->crop_en = RKISP2_ISP_CROP_ENABLE_MAIN;
	config->mp_crop.h_offs = frameContext.crop.crop.x;
	config->mp_crop.v_offs = frameContext.crop.crop.y;
	config->mp_crop.h_size = frameContext.crop.crop.width;
	config->mp_crop.v_size = frameContext.crop.crop.height;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void Crop::process([[maybe_unused]] IPAContext &context,
		   [[maybe_unused]] const uint32_t frame,
		   IPAFrameContext &frameContext,
		   [[maybe_unused]] const RkISP2Stats *stats,
		   ControlList &metadata)
{
	/* \todo Adjust this to match the spec */
	metadata.set(controls::ScalerCrop, frameContext.crop.crop);
}

REGISTER_IPA_ALGORITHM(Crop, "Crop")

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
