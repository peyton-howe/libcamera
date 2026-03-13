/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board Oy.
 *
 * AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <span>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

namespace ipa::rkisp2::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 */

LOG_DEFINE_CATEGORY(RkISP2Agc)

/**
 * \brief Initialise the AGC algorithm from tuning files
 * \param[in] context The shared IPA context
 * \param[in] tuningData The ValueNode containing Agc tuning data
 *
 * \return 0 on success or errors from the base class
 */
int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	return agc_.init(tuningData, context.camHelper.get(), {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
}

/**
 * \brief Configure the AGC given a configInfo
 * \param[in] context The shared IPA context
 * \param[in] configInfo The IPA configuration data
 *
 * \return 0
 */
int Agc::configure(IPAContext &context, const IPACameraSensorInfo &configInfo)
{
	int ret = agc_.configure(context.configuration.agc, context.activeState.agc, {
		.sensorInfo = context.sensorInfo,
		.sensorControls = context.sensorControls,
		.ctrlMap = context.ctrlMap,
	});
	if (ret)
		return ret;

	context.configuration.agc.measureWindow.h_offs = 0;
	context.configuration.agc.measureWindow.v_offs = 0;
	context.configuration.agc.measureWindow.h_size = (configInfo.outputSize.width / 5);
	/*
	 * ae lite needs the -2 because the total window height must be
	 * divisible by 2, and it cannot be equal to or greater than the frame
	 * size, or else the hardware hangs
	 *
	 * \todo Move this to the kernel?
	 * \todo Check if hist lite also needs this
	 */
	context.configuration.agc.measureWindow.v_size = (configInfo.outputSize.height / 5) - 2;

	context.configuration.agc.measureWindow15.h_size = (configInfo.outputSize.width / 15);
	context.configuration.agc.measureWindow15.v_size = (configInfo.outputSize.height / 15) - 2;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void Agc::queueRequest(IPAContext &context,
		       [[maybe_unused]] const uint32_t frame,
		       IPAFrameContext &frameContext,
		       const ControlList &controls)
{
	agc_.queueRequest(context.configuration.agc, context.activeState.agc,
			  frameContext.agc, controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext, RkISP2Params *params)
{
	agc_.prepare(context.configuration.agc, context.activeState.agc, frameContext.agc);

	if (frame > 1)
		return;

	/*
	 * Configure the AEC measurements. Set the window, measure
	 * continuously, and estimate Y as (R + G + B) x (85/256).
	 */
	auto aeLiteConfig = params->block<RkISP2ParamsBlocks::AeLite>();
	aeLiteConfig.setEnabled(true);

	aeLiteConfig->window_num = 1;
	aeLiteConfig->meas_window = context.configuration.agc.measureWindow;

	auto hstConfig = params->block<RkISP2ParamsBlocks::HistBig0>();
	hstConfig.setEnabled(true);

	hstConfig->window_num = 0;
	/* \todo choose this based on the bitdepth */
	hstConfig->data_sel = RKISP2_ISP_HISTOGRAM_DATA_SEL_9_2;
	hstConfig->mode = RKISP2_ISP_HISTOGRAM_MODE_Y_HISTOGRAM;
	/* waterline means to exclude everything above this value */
	hstConfig->waterline = 0x0;
	hstConfig->stepsize = 0;
	hstConfig->coeffs.r = 0x21;
	hstConfig->coeffs.g = 0x20;
	hstConfig->coeffs.b = 0x0d;

	hstConfig->meas_window = context.configuration.agc.measureWindow15;

	/* \todo Support configuring the weights */
	for (size_t i = 0; i < RKISP2_ISP_HIST_WEIGHT_GRIDS_SIZE_BIG; i++)
		hstConfig->weights[i] = 0x20;

	auto hstConfigLite = params->block<RkISP2ParamsBlocks::HistLite>();
	hstConfigLite.setEnabled(false);
}

namespace {

class AgcTraits : public AgcMeanLuminance::Traits
{
public:
	AgcTraits(std::span<const uint16_t> expMeans)
		: expMeans_(expMeans)
	{
	}

	/*
	 * \brief Estimate the relative luminance of the frame with a given gain
	 * \param[in] gain The gain to apply to the frame
	 *
	 * The estimation is based on the AE statistics for the current frame. The
	 * averages for all cells are first multiplied by the gain, and then
	 * saturated to approximate the sensor behaviour at high brightness values.
	 *
	 * The values are normalized to the [0.0, 1.0] range.
	 *
	 * \return The relative luminance
	 */
	double estimateLuminance(double gain) const override
	{
		double ySum = 0.0;

		/* Sum the averages, saturated to 4095. */
		for (uint16_t mean : expMeans_)
			ySum += std::min(mean * gain, 4095.0);

		/*
		 * \todo Support configuring the weights, and weight with the
		 * AWB gains
		 */

		return ySum / expMeans_.size() / 4095;
	}

private:
	std::span<const uint16_t> expMeans_;
};

} /* namespace */

/**
 * \brief Process RkISP2 statistics, and run AGC operations
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The current frame context
 * \param[in] stats The RKISP2 statistics and ISP results
 * \param[out] metadata Metadata for the frame, to be filled by the algorithm
 *
 * Identify the current image brightness, and use that to estimate the optimal
 * new exposure and gain for the scene.
 */
void Agc::process(IPAContext &context, const uint32_t frame,
		  IPAFrameContext &frameContext, const RkISP2Stats *stats,
		  ControlList &metadata)
{
	/* The first frame has no stats so skip processing */
	if (frame < 1) {
		agc_.process(context.configuration.agc, context.activeState.agc,
			     frameContext.agc, {}, metadata);
		return;
	}

	auto aeLite = stats->block<RkISP2StatsBlocks::AeLite>();
	auto histBig = stats->block<RkISP2StatsBlocks::HistBig0>();

	/*
	 * \todo Verify that the exposure and gain applied by the sensor for
	 * this frame match what has been requested. This isn't a hard
	 * requirement for stability of the AGC (the guarantee we need in
	 * automatic mode is a perfect match between the frame and the values
	 * we receive), but is important in manual mode.
	 */

	/* The lower 5 bits are fractional and meant to be discarded. */
	Histogram hist({ histBig->hist_bins, RKISP2_ISP_HIST_BIN_N_MAX },
		       [](uint32_t x) { return x >> 5; });

	std::vector<uint16_t> expMeans(RKISP2_ISP_AE_MEAN_MAX_LITE);
	for (size_t i = 0; i < RKISP2_ISP_AE_MEAN_MAX_LITE; i++) {
		/* r and b are 0~1023; g is 255~4095 so multiply r and b to match g */
		uint16_t r = aeLite->exp_mean_r[i] * 4;
		uint16_t g = aeLite->exp_mean_g[i];
		uint16_t b = aeLite->exp_mean_b[i] * 4;
		expMeans[i] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
	}

	if (frameContext.sensor.exposure * frameContext.sensor.gain == 0) {
		LOG(RkISP2Agc, Warning)
			<< "frame " << frame << ": Effective exposure value is 0: sensor exposure: "
			<< frameContext.sensor.exposure << ", analogue gain: "
			<< frameContext.sensor.gain;
	}

	/*
	 * \todo Support lux estimation, and setting the constraint and
	 * exposure modes
	 */
	AgcTraits traits(expMeans);

	agc_.process(context.configuration.agc, context.activeState.agc, frameContext.agc, {{
		.traits = traits,
		.yHist = hist,
		.exposure = frameContext.sensor.exposure,
		.gain = frameContext.sensor.gain,
	}}, metadata);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
