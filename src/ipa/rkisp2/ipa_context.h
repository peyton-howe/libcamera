/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas on Board Oy.
 *
 * RkISP2 IPA Context
 *
 */

#pragma once

#include <memory>

#include <linux/media/rockchip/rkisp2-config.h>

#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include <libcamera/ipa/core_ipa_interface.h>

#include "libcamera/internal/debug_controls.h"
#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include "libipa/agc_mean_luminance.h"
#include "libipa/awb.h"
#include "libipa/camera_sensor_helper.h"
#include "libipa/ccm.h"
#include "libipa/fc_queue.h"
#include "libipa/fixedpoint.h"
#include "libipa/lsc.h"

namespace libcamera {

namespace ipa::rkisp2 {

struct IPAHwSettings {
	unsigned int numAeCells;
	unsigned int numHistogramBins;
	unsigned int numHistogramWeights;
	unsigned int numGammaOutSamples;
	uint32_t supportedBlocks;
	bool compand;
};

struct RKISP2AwbSession {
	struct rkisp2_isp_window measureWindow;
	bool enabled;
};

struct IPASessionConfiguration {
	struct {
		struct rkisp2_isp_window measureWindow;
		struct rkisp2_isp_window measureWindow15;
	} agc;

	struct RKISP2AwbSession awb;

	struct {
		utils::Duration minExposureTime;
		utils::Duration maxExposureTime;
		double minAnalogueGain;
		double maxAnalogueGain;

		int32_t defVBlank;
		utils::Duration lineDuration;
		Size size;
	} sensor;

	struct {
		int32_t colorSpaceEncoding;
		int32_t colorSpaceRange;
	} csm;

	bool raw;
};

struct IPAActiveState {
	struct {
		struct {
			uint32_t exposure;
			double gain;
		} manual;
		struct {
			uint32_t exposure;
			double gain;
			double quantizationGain;
			double yTarget;
		} automatic;

		bool autoExposureEnabled;
		bool autoGainEnabled;
		double exposureValue;
		controls::AeConstraintModeEnum constraintMode;
		controls::AeExposureModeEnum exposureMode;
		controls::AeMeteringModeEnum meteringMode;
		utils::Duration minFrameDuration;
		utils::Duration maxFrameDuration;
	} agc;

	ipa::awb::ActiveState awb;

	struct {
		double gamma;
	} goc;

	ipa::ccm::ActiveState ccm;

	struct {
		double lux;
	} lux;

	struct {
		controls::WdrModeEnum mode;
		AgcMeanLuminance::AgcConstraint constraint;
		double gain;
		double strength;
	} wdr;

	ipa::lsc::ActiveState lsc;

	struct {
		Matrix<uint16_t, 3, 3> csm;
		bool update;
	} csm;

	struct {
		Rectangle crop;
		bool set;
	} crop;
};

struct IPAFrameContext : public FrameContext {
	struct {
		uint32_t exposure;
		double gain;
		double exposureValue;
		double quantizationGain;
		uint32_t vblank;
		double yTarget;
		bool autoExposureEnabled;
		bool autoGainEnabled;
		controls::AeConstraintModeEnum constraintMode;
		controls::AeExposureModeEnum exposureMode;
		controls::AeMeteringModeEnum meteringMode;
		utils::Duration minFrameDuration;
		utils::Duration maxFrameDuration;
		utils::Duration frameDuration;
		bool updateMetering;
		bool autoExposureModeChange;
		bool autoGainModeChange;
	} agc;

	ipa::awb::FrameContext awb;

	struct {
		double gamma;
		bool update;
	} goc;

	struct {
		uint32_t exposure;
		double gain;
	} sensor;

	ipa::ccm::FrameContext ccm;

	struct {
		double lux;
	} lux;

	struct {
		controls::WdrModeEnum mode;
		double strength;
		double gain;
	} wdr;

	ipa::lsc::FrameContext lsc;

	struct {
		Rectangle crop;
		bool set;
	} crop;
};

struct IPAContext {
	IPAContext(unsigned int frameContextSize)
		: frameContexts(frameContextSize)
	{
	}

	IPAHwSettings hw;
	IPACameraSensorInfo sensorInfo;
	IPASessionConfiguration configuration;
	IPAActiveState activeState;

	FCQueue<IPAFrameContext> frameContexts;

	ControlInfoMap::Map ctrlMap;

	DebugMetadata debugMetadata;

	/* Interface to the Camera Helper */
	std::unique_ptr<CameraSensorHelper> camHelper;
};

} /* namespace ipa::rkisp2 */

} /* namespace libcamera*/
