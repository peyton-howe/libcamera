/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board Oy.
 *
 * AGC/AEC mean-based control algorithm
 */

#include "agc.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <tuple>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/ipa/core_ipa_interface.h>

#include "libipa/histogram.h"

/**
 * \file agc.h
 */

namespace libcamera {

using namespace std::literals::chrono_literals;

namespace ipa::rkisp2::algorithms {

/**
 * \class Agc
 * \brief A mean-based auto-exposure algorithm
 */

LOG_DEFINE_CATEGORY(RkISP2Agc)

int Agc::init(IPAContext &context, const ValueNode &tuningData)
{
	int ret = parseTuningData(tuningData);
	if (ret)
		return ret;

	context.ctrlMap[&controls::ExposureTimeMode] =
		ControlInfo({ { ControlValue(controls::ExposureTimeModeAuto),
				ControlValue(controls::ExposureTimeModeManual) } },
			    ControlValue(controls::ExposureTimeModeAuto));
	context.ctrlMap[&controls::AnalogueGainMode] =
		ControlInfo({ { ControlValue(controls::AnalogueGainModeAuto),
				ControlValue(controls::AnalogueGainModeManual) } },
			    ControlValue(controls::AnalogueGainModeAuto));
	context.ctrlMap[&controls::ExposureValue] = ControlInfo(-8.0f, 8.0f, 0.0f);
	/* \todo Support AnalogueGain and ExposureTime in raw mode */
	context.ctrlMap.merge(controls());

	return 0;
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

	/* Configure the default exposure and gain. */
	context.activeState.agc.automatic.gain = context.configuration.sensor.minAnalogueGain;
	context.activeState.agc.automatic.exposure =
		10ms / context.configuration.sensor.lineDuration;
	context.activeState.agc.automatic.quantizationGain = 1.0;
	context.activeState.agc.manual.gain = context.activeState.agc.automatic.gain;
	context.activeState.agc.manual.exposure = context.activeState.agc.automatic.exposure;
	context.activeState.agc.autoExposureEnabled = true;
	context.activeState.agc.autoGainEnabled = true;
	context.activeState.agc.exposureValue = 0.0;

	/* Limit the frame duration to match current initialisation */
	ControlInfo &frameDurationLimits = context.ctrlMap[&controls::FrameDurationLimits];
	context.activeState.agc.minFrameDuration = std::chrono::microseconds(frameDurationLimits.min().get<int64_t>());
	context.activeState.agc.maxFrameDuration = std::chrono::microseconds(frameDurationLimits.max().get<int64_t>());

	AgcMeanLuminance::configure(context.configuration.sensor.lineDuration,
				    context.camHelper.get());

	setLimits(context.configuration.sensor.minExposureTime,
		  context.configuration.sensor.maxExposureTime,
		  context.configuration.sensor.minAnalogueGain,
		  context.configuration.sensor.maxAnalogueGain, {});

	context.activeState.agc.automatic.yTarget = effectiveYTarget();

	resetFrameCount();

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
	auto &agc = context.activeState.agc;

	const auto &aeEnable = controls.get(controls::ExposureTimeMode);
	if (aeEnable &&
	    (*aeEnable == controls::ExposureTimeModeAuto) != agc.autoExposureEnabled) {
		agc.autoExposureEnabled = (*aeEnable == controls::ExposureTimeModeAuto);

		LOG(RkISP2Agc, Debug)
			<< (agc.autoExposureEnabled ? "Enabling" : "Disabling")
			<< " AGC (exposure)";

		/*
		 * If we go from auto -> manual with no manual control
		 * set, use the last computed value, which we don't
		 * know until prepare() so save this information.
		 *
		 * \todo Check the previous frame at prepare() time
		 * instead of saving a flag here
		 */
		if (!agc.autoExposureEnabled && !controls.get(controls::ExposureTime))
			frameContext.agc.autoExposureModeChange = true;
	}

	const auto &agEnable = controls.get(controls::AnalogueGainMode);
	if (agEnable &&
	    (*agEnable == controls::AnalogueGainModeAuto) != agc.autoGainEnabled) {
		agc.autoGainEnabled = (*agEnable == controls::AnalogueGainModeAuto);

		LOG(RkISP2Agc, Debug)
			<< (agc.autoGainEnabled ? "Enabling" : "Disabling")
			<< " AGC (gain)";
		/*
		 * If we go from auto -> manual with no manual control
		 * set, use the last computed value, which we don't
		 * know until prepare() so save this information.
		 */
		if (!agc.autoGainEnabled && !controls.get(controls::AnalogueGain))
			frameContext.agc.autoGainModeChange = true;
	}

	const auto &exposure = controls.get(controls::ExposureTime);
	if (exposure && !agc.autoExposureEnabled) {
		agc.manual.exposure = *exposure * 1.0us
				    / context.configuration.sensor.lineDuration;

		LOG(RkISP2Agc, Debug)
			<< "Set exposure to " << agc.manual.exposure;
	}

	const auto &gain = controls.get(controls::AnalogueGain);
	if (gain && !agc.autoGainEnabled) {
		agc.manual.gain = *gain;

		LOG(RkISP2Agc, Debug) << "Set gain to " << agc.manual.gain;
	}

	frameContext.agc.autoExposureEnabled = agc.autoExposureEnabled;
	frameContext.agc.autoGainEnabled = agc.autoGainEnabled;

	if (!frameContext.agc.autoExposureEnabled)
		frameContext.agc.exposure = agc.manual.exposure;
	if (!frameContext.agc.autoGainEnabled)
		frameContext.agc.gain = agc.manual.gain;

	if (!frameContext.agc.autoExposureEnabled &&
	    !frameContext.agc.autoGainEnabled)
		frameContext.agc.quantizationGain = 1.0;

	const auto &exposureValue = controls.get(controls::ExposureValue);
	if (exposureValue)
		agc.exposureValue = *exposureValue;
	frameContext.agc.exposureValue = agc.exposureValue;

	const auto &frameDurationLimits = controls.get(controls::FrameDurationLimits);
	if (frameDurationLimits) {
		/* Limit the control value to the limits in ControlInfo */
		ControlInfo &limits = context.ctrlMap[&controls::FrameDurationLimits];
		int64_t minFrameDuration =
			std::clamp((*frameDurationLimits).front(),
				   limits.min().get<int64_t>(),
				   limits.max().get<int64_t>());
		int64_t maxFrameDuration =
			std::clamp((*frameDurationLimits).back(),
				   limits.min().get<int64_t>(),
				   limits.max().get<int64_t>());

		agc.minFrameDuration = std::chrono::microseconds(minFrameDuration);
		agc.maxFrameDuration = std::chrono::microseconds(maxFrameDuration);
	}
	frameContext.agc.minFrameDuration = agc.minFrameDuration;
	frameContext.agc.maxFrameDuration = agc.maxFrameDuration;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void Agc::prepare(IPAContext &context, [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext, RkISP2Params *params)
{
	uint32_t activeAutoExposure = context.activeState.agc.automatic.exposure;
	double activeAutoGain = context.activeState.agc.automatic.gain;
	double activeAutoQGain = context.activeState.agc.automatic.quantizationGain;

	/* Populate exposure and gain in auto mode */
	if (frameContext.agc.autoExposureEnabled) {
		frameContext.agc.exposure = activeAutoExposure;
		frameContext.agc.quantizationGain = activeAutoQGain;
	}
	if (frameContext.agc.autoGainEnabled) {
		frameContext.agc.gain = activeAutoGain;
		frameContext.agc.quantizationGain = activeAutoQGain;
	}

	/*
	 * Populate manual exposure and gain from the active auto values when
	 * transitioning from auto to manual
	 */
	if (!frameContext.agc.autoExposureEnabled && frameContext.agc.autoExposureModeChange) {
		context.activeState.agc.manual.exposure = activeAutoExposure;
		frameContext.agc.exposure = activeAutoExposure;
	}
	if (!frameContext.agc.autoGainEnabled && frameContext.agc.autoGainModeChange) {
		context.activeState.agc.manual.gain = activeAutoGain;
		frameContext.agc.gain = activeAutoGain;
		frameContext.agc.quantizationGain = activeAutoQGain;
	}

	frameContext.agc.yTarget = context.activeState.agc.automatic.yTarget;

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

void Agc::fillMetadata(IPAContext &context, IPAFrameContext &frameContext,
		       ControlList &metadata, [[maybe_unused]] const RkISP2Stats *stats)
{
	utils::Duration exposureTime = context.configuration.sensor.lineDuration
				     * frameContext.sensor.exposure;
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::FrameDuration, frameContext.agc.frameDuration.get<std::micro>());
	metadata.set(controls::ExposureTimeMode,
		     frameContext.agc.autoExposureEnabled
		     ? controls::ExposureTimeModeAuto
		     : controls::ExposureTimeModeManual);
	metadata.set(controls::AnalogueGainMode,
		     frameContext.agc.autoGainEnabled
		     ? controls::AnalogueGainModeAuto
		     : controls::AnalogueGainModeManual);

	metadata.set(controls::ExposureValue, frameContext.agc.exposureValue);
}

double Agc::estimateLuminance(double gain) const
{
	/*
	 * \todo Enforce this check, since lite-like is 5x5 while big is 15x15,
	 * but I haven't yet figured out how to use all 15x15 weights. At the
	 * moment we're running everything lite-like
	 */
	ASSERT(expMeans_.size() == weights_.size());
	double ySum = 0.0;
	double wSum = 0.0;

	/* Sum the averages, saturated to 4095. */
	for (unsigned i = 0; i < expMeans_.size(); i++) {
		double w = weights_[i] / 0x10;
		ySum += std::min(expMeans_[i] * gain, 4095.0) * w;
		wSum += w;
	}

	/* \todo Weight with the AWB gains */

	return ySum / wSum / 4095;
}

void Agc::processFrameDuration(IPAContext &context,
			       IPAFrameContext &frameContext,
			       utils::Duration frameDuration)
{
	IPACameraSensorInfo &sensorInfo = context.sensorInfo;
	utils::Duration lineDuration = context.configuration.sensor.lineDuration;

	frameContext.agc.vblank = (frameDuration / lineDuration) - sensorInfo.outputSize.height;

	/* Update frame duration accounting for line length quantization. */
	frameContext.agc.frameDuration = (sensorInfo.outputSize.height + frameContext.agc.vblank) * lineDuration;
}

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
void Agc::process([[maybe_unused]] IPAContext &context,
		  const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  const RkISP2Stats *stats,
		  ControlList &metadata)
{
	/* The first frame has no stats so skip processing */
	if (frame < 1)
		return;

	const utils::Duration &lineDuration = context.configuration.sensor.lineDuration;

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

	expMeans_.resize(RKISP2_ISP_AE_MEAN_MAX_LITE);
	for (size_t i = 0; i < RKISP2_ISP_AE_MEAN_MAX_LITE; i++) {
		/* r and b are 0~1023; g is 255~4095 so multiply r and b to match g */
		uint16_t r = aeLite->exp_mean_r[i] * 4;
		uint16_t g = aeLite->exp_mean_g[i];
		uint16_t b = aeLite->exp_mean_b[i] * 4;
		expMeans_[i] = 0.2126 * r + 0.7152 * g + 0.0722 * b;
	}

	/* \todo Support configuring the weights */
	std::vector<uint8_t> modeWeights(25, 0x10);
	weights_ = { modeWeights.data(), modeWeights.size() };

	/*
	 * Set the AGC limits using the fixed exposure time and/or gain in
	 * manual mode, or the sensor limits in auto mode.
	 */
	utils::Duration minExposureTime;
	utils::Duration maxExposureTime;
	double minAnalogueGain;
	double maxAnalogueGain;

	if (frameContext.agc.autoExposureEnabled) {
		minExposureTime = context.configuration.sensor.minExposureTime;
		maxExposureTime = std::clamp(frameContext.agc.maxFrameDuration,
					     context.configuration.sensor.minExposureTime,
					     context.configuration.sensor.maxExposureTime);
	} else {
		minExposureTime = context.configuration.sensor.lineDuration
				* frameContext.agc.exposure;
		maxExposureTime = minExposureTime;
	}

	if (frameContext.agc.autoGainEnabled) {
		minAnalogueGain = context.configuration.sensor.minAnalogueGain;
		maxAnalogueGain = context.configuration.sensor.maxAnalogueGain;
	} else {
		minAnalogueGain = frameContext.agc.gain;
		maxAnalogueGain = frameContext.agc.gain;
	}

	setLimits(minExposureTime, maxExposureTime, minAnalogueGain, maxAnalogueGain, {});

	/*
	 * The Agc algorithm needs to know the effective exposure value that was
	 * applied to the sensor when the statistics were collected.
	 */
	utils::Duration exposureTime = lineDuration * frameContext.sensor.exposure;
	double analogueGain = frameContext.sensor.gain;
	utils::Duration effectiveExposureValue = exposureTime * analogueGain;
	if (effectiveExposureValue == 0ms) {
		LOG(RkISP2Agc, Warning)
			<< "frame " << frame << ": Effective exposure value is 0: sensor exposure: "
			<< exposureTime << ", analogue gain: " << analogueGain;
	}

	/* \todo Support lux estimation */

	/* \todo Support setting constraint and exposure modes */
	utils::Duration newExposureTime;
	double aGain, qGain, dGain;
	std::tie(newExposureTime, aGain, qGain, dGain) =
		calculateNewEv(0, 0, hist, effectiveExposureValue);

	LOG(RkISP2Agc, Debug)
		<< "Divided up exposure time, analogue gain, quantization gain"
		<< " and digital gain are " << newExposureTime << ", " << aGain
		<< ", " << qGain << " and " << dGain;

	IPAActiveState &activeState = context.activeState;
	/* Update the estimated exposure and gain. */
	activeState.agc.automatic.exposure = newExposureTime / lineDuration;
	activeState.agc.automatic.gain = aGain;
	activeState.agc.automatic.quantizationGain = qGain;
	activeState.agc.automatic.yTarget = effectiveYTarget();
	/*
	 * Expand the target frame duration so that we do not run faster than
	 * the minimum frame duration when we have short exposures.
	 */
	processFrameDuration(context, frameContext,
			     std::max(frameContext.agc.minFrameDuration, newExposureTime));

	fillMetadata(context, frameContext, metadata, stats);
	expMeans_ = {};
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
