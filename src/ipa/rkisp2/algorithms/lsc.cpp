/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * RkISP2 Lens Shading Correction control
 */

#include "lsc.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

/**
 * \file lsc.h
 */

namespace libcamera {

namespace ipa::rkisp2::algorithms {

/**
 * \class LensShadingCorrection
 * \brief RkISP2 Lens Shading Correction control
 */

LOG_DEFINE_CATEGORY(RkISP2Lsc)

namespace {

constexpr int kColourTemperatureQuantization = 10;

unsigned int quantize(unsigned int value, unsigned int step)
{
	return std::lround(value / static_cast<double>(step)) * step;
}

} /* namespace */

LensShadingCorrection::LensShadingCorrection()
	: lastAppliedCt_(0), lastAppliedQuantizedCt_(0)
{
}

std::vector<double> LensShadingCorrection::parseSizes(const ValueNode &tuningData,
						      const char *prop)
{
	std::vector<double> sizes =
		tuningData[prop].get<std::vector<double>>().value_or(utils::defopt);
	/* Nobody cares about 8x8 mirrored mode; we'll just use 16x16 mode */
	if (sizes.size() != RKISP2_ISP_LSC_SECTORS_TBL_SIZE_MAX) {
		LOG(RkISP2Lsc, Error)
			<< "Invalid '" << prop << "' values: expected "
			<< RKISP2_ISP_LSC_SECTORS_TBL_SIZE_MAX
			<< " elements, got " << sizes.size();
		return {};
	}

	/*
	 * The sum of all elements must be 1 to satisfy hardware constraints.
	 * Validate it here, allowing a 1% tolerance as rounding errors may
	 * prevent an exact match (further adjustments will be performed in
	 * LensShadingCorrection::prepare()).
	 *
	 * If we were in 8x8 mode then we'd have to mirror the quadrants like
	 * in rkisp1, but in 16x16 mode we get to configure the entire table.
	 * Since 8x8 table support is a todo, we only need to handle the 16x16
	 * case here thus the sum should be 1.
	 *
	 * \todo Support 8x8 mode?
	 */
	double sum = std::accumulate(sizes.begin(), sizes.end(), 0.0);
	if (sum < 0.95 || sum > 1.05) {
		LOG(RkISP2Lsc, Error)
			<< "Invalid '" << prop << "' values: sum of the elements"
			<< " should be 1.0, got " << sum;
		return {};
	}

	return sizes;
}

std::vector<double> LensShadingCorrection::sizesToPositions(Span<const double> sizes)
{
	std::vector<double> positions(sizes.size() + 1);

	positions[0] = 0.0;
	for (size_t i = 1; i < positions.size(); i++)
		positions[i] = positions[i - 1] + sizes[i - 1];

	return positions;
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int LensShadingCorrection::init(IPAContext &context,
				const ValueNode &tuningData)
{
	xSize_ = parseSizes(tuningData, "x-size");
	ySize_ = parseSizes(tuningData, "y-size");

	if (xSize_.empty() || ySize_.empty())
		return -EINVAL;

	xPos_ = sizesToPositions(xSize_);
	yPos_ = sizesToPositions(ySize_);

	return lscAlgo_.init(tuningData, context.ctrlMap, LscDescriptor{
				.keys = { "r", "gr", "gb", "b" },
				.numHSamples = RKISP2_ISP_LSC_SAMPLES_MAX,
				.numVSamples = RKISP2_ISP_LSC_SAMPLES_MAX,
				.sensorSize = context.sensorInfo.activeAreaSize
			     });
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int LensShadingCorrection::configure(IPAContext &context,
				     const IPACameraSensorInfo &configInfo)
{
	const Size &size = context.configuration.sensor.size;
	Size totalSize{};

	/* Calculate gradients. */
	for (unsigned int i = 0; i < RKISP2_ISP_LSC_SECTORS_TBL_SIZE_MAX; ++i) {
		xSizes_[i] = xSize_[i] * size.width;
		ySizes_[i] = ySize_[i] * size.height;

		/*
		 * To prevent unexpected behavior of the ISP, the sum of
		 * x_sizes and y_sizes items shall be equal to
		 * respectively size.width and size.height. Enforce it by
		 * computing the last tables value to avoid
		 * rounding-induced errors.
		 */
		if (i == RKISP2_ISP_LSC_SECTORS_TBL_SIZE_MAX - 1) {
			xSizes_[i] = size.width - totalSize.width;
			ySizes_[i] = size.height - totalSize.height;
		}

		totalSize.width += xSizes_[i];
		totalSize.height += ySizes_[i];

		xGrad_[i] = std::round(32768 / xSizes_[i]);
		yGrad_[i] = std::round(32768 / ySizes_[i]);
	}

	return lscAlgo_.configure(context.activeState.lsc, configInfo.analogCrop,
				  xPos_, yPos_);
}

void LensShadingCorrection::setParameters(rkisp2_params_lsc &config)
{
	memcpy(config.x_grads, xGrad_, sizeof(config.x_grads));
	memcpy(config.y_grads, yGrad_, sizeof(config.y_grads));
	memcpy(config.x_sizes, xSizes_, sizeof(config.x_sizes));
	memcpy(config.y_sizes, ySizes_, sizeof(config.y_sizes));
}

void LensShadingCorrection::copyTable(rkisp2_params_lsc &config,
				      const ipa::lsc::Components<uint16_t> &set)
{
	const auto &r = set.at("r");
	std::copy(r.begin(), r.end(), &config.r_data_tbl[0][0][0]);
	const auto &gr = set.at("gr");
	std::copy(gr.begin(), gr.end(), &config.gr_data_tbl[0][0][0]);
	const auto &gb = set.at("gb");
	std::copy(gb.begin(), gb.end(), &config.gb_data_tbl[0][0][0]);
	const auto &b = set.at("b");
	std::copy(b.begin(), b.end(), &config.b_data_tbl[0][0][0]);
}

/**
 * \copydoc libcamera::ipa::Algorithm::queueRequest
 */
void LensShadingCorrection::queueRequest(IPAContext &context,
					 [[maybe_unused]] const uint32_t frame,
					 IPAFrameContext &frameContext,
					 const ControlList &controls)
{
	lscAlgo_.queueRequest(context.activeState.lsc, frameContext.lsc,
			      controls);
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void LensShadingCorrection::prepare([[maybe_unused]] IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    IPAFrameContext &frameContext,
				    RkISP2Params *params)
{
	uint32_t ct = frameContext.awb.colourTemperature;
	unsigned int quantizedCt = quantize(ct, kColourTemperatureQuantization);

	/* Check if we can skip the update. */
	if (!frameContext.lsc.update) {
		if (!frameContext.lsc.enabled)
			return;

		/*
		 * Add a threshold so that oscillations around a quantization
		 * step don't lead to constant changes.
		 */
		if (utils::abs_diff(ct, lastAppliedCt_) < kColourTemperatureQuantization / 2)
			return;

		if (quantizedCt == lastAppliedQuantizedCt_)
			return;
	}

	auto config = params->block<RkISP2ParamsBlocks::Lsc>();
	config.setEnabled(frameContext.lsc.enabled);

	if (!frameContext.lsc.enabled)
		return;

	/*
	 * \todo Should add support for the lsc table swapping functionality?
	 * Or maybe we don't need it because the lsc doesn't change very
	 * frequently. Just use the 0th table for now.
	 */
	config->window_mode = RKISP2_ISP_LSC_CONFIG_16X16;
	config->write_table[0] = 1;
	config->write_table[1] = 0;
	config->active_table = 0;
	config->set_active_table_when = RKISP2_ISP_LSC_SET_ACTIVE_TABLE_AFTER;

	setParameters(*config);

	const auto &set = lscAlgo_.interpolateComponents(quantizedCt);
	copyTable(*config, set);

	lastAppliedCt_ = ct;
	lastAppliedQuantizedCt_ = quantizedCt;

	LOG(RkISP2Lsc, Debug)
		<< "ct is " << ct << ", quantized to "
		<< quantizedCt;
}

/**
 * \copydoc libcamera::ipa::Algorithm::process
 */
void LensShadingCorrection::process([[maybe_unused]] IPAContext &context,
				    [[maybe_unused]] const uint32_t frame,
				    IPAFrameContext &frameContext,
				    [[maybe_unused]] const RkISP2Stats *stats,
				    ControlList &metadata)
{
	lscAlgo_.process(frameContext.lsc, metadata);
}

REGISTER_IPA_ALGORITHM(LensShadingCorrection, "LensShadingCorrection")

} /* namespace ipa::rkisp2::algorithms */

} /* namespace libcamera */
